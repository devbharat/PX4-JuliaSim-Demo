"""PX4Lockstep.Sim.Simulation

Deterministic single-threaded simulation engine.

Key ideas (matching the simulator design principles in the blog you linked):
- The vehicle/body dynamics are a continuous-time ODE integrated with a fixed-step solver.
- The autopilot, estimators, scenarios, and logging are discrete-time tasks ("hybrid system").
- Everything runs in one thread, driven by simulation time (no wall clock).

This file focuses on:
- explicit data flow (truth → estimator → autopilot → actuators → dynamics)
- deterministic multi-rate stepping (sample-and-hold)
- clean extension points (vehicle models, worlds, estimators, integrators, log sinks)
"""
module Simulation

using ..Types: Vec3, Quat, vec3
using ..RigidBody: RigidBodyState
using ..Environment: EnvironmentModel, wind_velocity, air_density
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, step_actuators!, dynamics
using ..Integrators: AbstractIntegrator, step_integrator
using ..Autopilots: AbstractAutopilot, PX4LockstepAutopilot, AutopilotCommand, autopilot_step
using ..Estimators: AbstractEstimator, TruthEstimator, estimate!
using ..Scenario: AbstractScenario, ScriptedScenario, scenario_step
import ..Powertrain
using ..Powertrain: AbstractBatteryModel, IdealBattery, BatteryStatus, status
using ..Scheduling: PeriodicTrigger, due!
using ..Logging: AbstractLogSink, SimLog, log!, close!

using Random
using StaticArrays

export VehicleInstance,
       SimulationConfig,
       SimulationInstance,
       step!, run!

############################
# Simulation types
############################

"""Vehicle instance = model + actuator models + state."""
mutable struct VehicleInstance{M<:AbstractVehicleModel, AM, AS}
    model::M
    motor_actuators::AM
    servo_actuators::AS
    state::RigidBodyState
end

"""Simulation configuration."""
Base.@kwdef struct SimulationConfig
    # Physics integration step (continuous-time plant).
    dt::Float64 = 0.002
    t0::Float64 = 0.0
    t_end::Float64 = 20.0

    # Multi-rate tasks. Set <= 0 to use `dt`.
    dt_autopilot::Float64 = -1.0
    dt_log::Float64 = -1.0

    # Determinism.
    seed::UInt64 = 0

    # Simple ground plane (z=0 in NED). This is intentionally minimal; terrain/contact
    # dynamics are a future model component.
    enable_ground_plane::Bool = true
end

mutable struct SimulationInstance{E,V,A,EST,I,S,B,L,R}
    cfg::SimulationConfig
    env::E
    vehicle::V
    autopilot::A
    estimator::EST
    integrator::I
    scenario::S
    battery::B
    log::L
    rng::R

    # simulation time
    t::Float64

    # multi-rate triggers
    ap_trig::PeriodicTrigger
    log_trig::PeriodicTrigger

    # last autopilot output (sample-and-hold)
    last_out::Any
end

function SimulationInstance(; cfg::SimulationConfig=SimulationConfig(),
                              env::EnvironmentModel=EnvironmentModel(),
                              vehicle::VehicleInstance,
                              autopilot::AbstractAutopilot,
                              estimator::AbstractEstimator=TruthEstimator(),
                              integrator::AbstractIntegrator,
                              scenario::AbstractScenario=ScriptedScenario(),
                              battery::AbstractBatteryModel=IdealBattery(),
                              log::AbstractLogSink=SimLog())
    dt_ap = (cfg.dt_autopilot > 0) ? max(cfg.dt_autopilot, cfg.dt) : cfg.dt
    dt_log = (cfg.dt_log > 0) ? max(cfg.dt_log, cfg.dt) : cfg.dt

    rng = MersenneTwister(cfg.seed)

    sim = SimulationInstance(cfg, env, vehicle, autopilot, estimator, integrator, scenario,
                             battery, log, rng, cfg.t0,
                             PeriodicTrigger(dt_ap, cfg.t0),
                             PeriodicTrigger(dt_log, cfg.t0),
                             nothing)

    return sim
end

############################
# Helpers
############################

@inline function _ground_plane(x::RigidBodyState)
    if x.pos_ned[3] > 0.0
        # Clamp to ground plane z=0 (NED). This prevents the simulation from
        # "falling through" when you aren't modeling contact yet.
        return RigidBodyState(
            pos_ned = vec3(x.pos_ned[1], x.pos_ned[2], 0.0),
            vel_ned = vec3(x.vel_ned[1], x.vel_ned[2], min(x.vel_ned[3], 0.0)),
            q_bn    = x.q_bn,
            ω_body  = x.ω_body,
        )
    end
    return x
end

@inline function _sanitize_cmd(x, lo::Float64, hi::Float64)
    isfinite(x) ? clamp(x, lo, hi) : 0.0
end

@inline function _extract_actuator_command(out)::ActuatorCommand
    # Default: assume PX4 LockstepOutputs struct.
    m_raw = getproperty(out, :actuator_motors)
    s_raw = getproperty(out, :actuator_servos)
    motors = SVector{12,Float64}(ntuple(i -> _sanitize_cmd(Float64(m_raw[i]), 0.0, 1.0), 12))
    servos = SVector{8,Float64}(ntuple(i -> _sanitize_cmd(Float64(s_raw[i]), -1.0, 1.0), 8))
    return ActuatorCommand(motors=motors, servos=servos)
end

############################
# Main step
############################

"""Advance the simulation by one physics step (`cfg.dt`)."""
function step!(sim::SimulationInstance)
    dt = sim.cfg.dt
    t = sim.t

    # Scenario produces high-level autopilot commands and an approximate landed flag.
    cmd, landed = scenario_step(sim.scenario, t, sim.vehicle.state)

    # Battery model -> PX4 battery_status injection.
    batt = status(sim.battery)

    # Estimator + autopilot multi-rate step.
    if (sim.last_out === nothing) || due!(sim.ap_trig, t)
        est = estimate!(sim.estimator, sim.rng, t, sim.vehicle.state, sim.ap_trig.period)
        sim.last_out = autopilot_step(sim.autopilot, t, est.pos_ned, est.vel_ned, est.q_bn, est.ω_body, cmd;
                                      landed=landed, battery=batt)
    end
    out = sim.last_out

    # Extract actuator commands from the autopilot output.
    u_cmd = _extract_actuator_command(out)

    # Apply actuator dynamics (motors and servos independently).
    m_dyn = step_actuators!(sim.vehicle.motor_actuators, u_cmd.motors, dt)
    s_dyn = step_actuators!(sim.vehicle.servo_actuators, u_cmd.servos, dt)
    u = ActuatorCommand(motors=m_dyn, servos=s_dyn)

    # Integrate rigid body over [t, t+dt) with piecewise-constant input u.
    f = (τ, state, uu) -> dynamics(sim.vehicle.model, sim.env, τ, state, uu)
    new_state = step_integrator(sim.integrator, f, t, sim.vehicle.state, u, dt)

    if sim.cfg.enable_ground_plane
        new_state = _ground_plane(new_state)
    end
    sim.vehicle.state = new_state

    # Update battery based on *applied* motor commands.
    Powertrain.step!(sim.battery, u.motors, dt)

    # Logging (multi-rate).
    if due!(sim.log_trig, t)
        wind = wind_velocity(sim.env.wind, new_state.pos_ned, t)
        rho = air_density(sim.env.atmosphere, -new_state.pos_ned[3])

        # PX4 setpoints (if present).
        pos_sp = (Float64(getproperty(out, :trajectory_setpoint_position)[1]),
                  Float64(getproperty(out, :trajectory_setpoint_position)[2]),
                  Float64(getproperty(out, :trajectory_setpoint_position)[3]))
        vel_sp = (Float64(getproperty(out, :trajectory_setpoint_velocity)[1]),
                  Float64(getproperty(out, :trajectory_setpoint_velocity)[2]),
                  Float64(getproperty(out, :trajectory_setpoint_velocity)[3]))
        acc_sp = (Float64(getproperty(out, :trajectory_setpoint_acceleration)[1]),
                  Float64(getproperty(out, :trajectory_setpoint_acceleration)[2]),
                  Float64(getproperty(out, :trajectory_setpoint_acceleration)[3]))

        yaw_sp = Float64(getproperty(out, :trajectory_setpoint_yaw))
        yawspeed_sp = Float64(getproperty(out, :trajectory_setpoint_yawspeed))

        log!(sim.log, t, new_state, u;
             wind_ned=wind,
             rho=rho,
             battery=batt,
             nav_state=Int32(getproperty(out, :nav_state)),
             arming_state=Int32(getproperty(out, :arming_state)),
             mission_seq=Int32(getproperty(out, :mission_seq)),
             mission_count=Int32(getproperty(out, :mission_count)),
             mission_finished=Int32(getproperty(out, :mission_finished)),
             pos_sp=pos_sp,
             vel_sp=vel_sp,
             acc_sp=acc_sp,
             yaw_sp=yaw_sp,
             yawspeed_sp=yawspeed_sp)
    end

    sim.t = t + dt
    return nothing
end

"""Run until `cfg.t_end`."""
function run!(sim::SimulationInstance; max_steps::Int=typemax(Int), close_log::Bool=true)
    steps = 0
    while sim.t < sim.cfg.t_end && steps < max_steps
        step!(sim)
        steps += 1
    end
    if close_log
        close!(sim.log)
    end
    return sim
end

end # module Simulation
