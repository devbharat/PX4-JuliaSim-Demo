"""PX4Lockstep.Sim.Simulation

Deterministic single-threaded simulation engine.

High-level philosophy:

* The simulation is the source of truth for vehicle state.
* The autopilot (PX4 lockstep) is stepped deterministically as a pure function of:
  - time
  - the truth state (with optional injected noise)
  - scenario commands and environment

The engine is structured for extensibility:

* swap integrators (Euler/RK4 now; others later)
* swap worlds (atmosphere, wind, gravity)
* swap vehicle models (quad today; fixed wing later)
* swap powertrain/battery models

This is meant to be reviewed by a simulation expert: code favors clarity, determinism,
and explicit interfaces over cleverness.
"""
module Simulation

using ..Types: Vec3, Quat, vec3
using ..RigidBody: RigidBodyState
using ..Environment: EnvironmentModel
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, step_actuators!, dynamics
using ..Integrators: AbstractIntegrator, step_integrator
using ..Autopilots: AbstractAutopilot, PX4LockstepAutopilot, AutopilotCommand, autopilot_step
using ..Scenario: AbstractScenario, ScriptedScenario, scenario_step
import ..Powertrain
using ..Powertrain: AbstractBatteryModel, IdealBattery, BatteryStatus, status
using ..Logging: SimLog, log!
using StaticArrays

export VehicleInstance,
       SimulationConfig,
       SimulationInstance,
       step!, run!

############################
# Simulation types
############################

"""Vehicle instance = model + state."""
mutable struct VehicleInstance{M<:AbstractVehicleModel, AM, AS}
    model::M
    motor_actuators::AM
    servo_actuators::AS
    state::RigidBodyState
end

"""Simulation configuration."""
Base.@kwdef struct SimulationConfig
    dt::Float64 = 0.002
    t0::Float64 = 0.0
    t_end::Float64 = 20.0
    # Simple ground plane contact.
    enable_ground_plane::Bool = true
end

"""A simulation instance."""
mutable struct SimulationInstance{E<:EnvironmentModel,V<:VehicleInstance,A<:AbstractAutopilot,
                                  I<:AbstractIntegrator,S<:AbstractScenario,B<:AbstractBatteryModel,L}
    cfg::SimulationConfig
    env::E
    vehicle::V
    autopilot::A
    integrator::I
    scenario::S
    battery::B
    log::L
    t::Float64
end

function SimulationInstance(; cfg::SimulationConfig=SimulationConfig(),
                              env::EnvironmentModel=EnvironmentModel(),
                              vehicle::VehicleInstance,
                              autopilot::AbstractAutopilot,
                              integrator::AbstractIntegrator,
                              scenario::AbstractScenario=ScriptedScenario(),
                              battery::AbstractBatteryModel=IdealBattery(),
                              log=SimLog())
    return SimulationInstance(cfg, env, vehicle, autopilot, integrator, scenario, battery, log, cfg.t0)
end

############################
# Helpers
############################

@inline function _ground_plane!(x::RigidBodyState)
    if x.pos_ned[3] > 0.0
        # Clamp to ground plane z=0 (NED).
        x = RigidBodyState(
            pos_ned = vec3(x.pos_ned[1], x.pos_ned[2], 0.0),
            vel_ned = vec3(x.vel_ned[1], x.vel_ned[2], min(x.vel_ned[3], 0.0)),
            q_bn = x.q_bn,
            ω_body = x.ω_body,
        )
    end
    return x
end

############################
# Main step
############################

"""Advance the simulation by one step."""
function step!(sim::SimulationInstance)
    dt = sim.cfg.dt
    t = sim.t

    # Scenario: high-level command and landed flag.
    cmd, landed = scenario_step(sim.scenario, t, sim.vehicle.state)

    # Battery model -> PX4 battery_status injection.
    batt = status(sim.battery)

    # Autopilot step (PX4 lockstep).
    x = sim.vehicle.state
    out = autopilot_step(sim.autopilot, t, x.pos_ned, x.vel_ned, x.q_bn, x.ω_body, cmd;
                         landed=landed, battery=batt)

    # Extract actuator commands from PX4 output.
    m_raw = out.actuator_motors
    s_raw = out.actuator_servos

    motors = SVector{12,Float64}(ntuple(i -> Float64(m_raw[i]), 12))
    servos = SVector{8,Float64}(ntuple(i -> Float64(s_raw[i]), 8))

    # Sanitize: PX4 often outputs NaNs when disarmed.
    motors = map(v -> (isfinite(v) ? clamp(v, 0.0, 1.0) : 0.0), motors)
    servos = map(v -> (isfinite(v) ? clamp(v, -1.0, 1.0) : 0.0), servos)

    # Apply actuator dynamics (if any) independently for motors and servos.
    motors = step_actuators!(sim.vehicle.motor_actuators, motors, dt)
    servos = step_actuators!(sim.vehicle.servo_actuators, servos, dt)

    u = ActuatorCommand(motors=motors, servos=servos)

    # Integrate rigid body.
    f = (τ, state, uu) -> dynamics(sim.vehicle.model, sim.env, τ, state, uu)
    new_state = step_integrator(sim.integrator, f, t, sim.vehicle.state, u, dt)

    if sim.cfg.enable_ground_plane
        new_state = _ground_plane!(new_state)
    end
    sim.vehicle.state = new_state

    # Update battery based on motor command.
    Powertrain.step!(sim.battery, motors, dt)

    # Log (keep it simple: store motor cmd + mission/nav info).
    pos_sp = (Float64(out.trajectory_setpoint_position[1]),
              Float64(out.trajectory_setpoint_position[2]),
              Float64(out.trajectory_setpoint_position[3]))
    vel_sp = (Float64(out.trajectory_setpoint_velocity[1]),
              Float64(out.trajectory_setpoint_velocity[2]),
              Float64(out.trajectory_setpoint_velocity[3]))
    acc_sp = (Float64(out.trajectory_setpoint_acceleration[1]),
              Float64(out.trajectory_setpoint_acceleration[2]),
              Float64(out.trajectory_setpoint_acceleration[3]))

    log!(sim.log, t, new_state,
         (motors[1], motors[2], motors[3], motors[4]);
         nav_state=out.nav_state,
         mission_seq=out.mission_seq,
         mission_count=out.mission_count,
         mission_finished=out.mission_finished,
         pos_sp=pos_sp,
         vel_sp=vel_sp,
         acc_sp=acc_sp,
         yaw_sp=Float64(out.trajectory_setpoint_yaw),
         yawspeed_sp=Float64(out.trajectory_setpoint_yawspeed))

    sim.t = t + dt
    return nothing
end

"""Run until `cfg.t_end` (or until mission finishes if you choose to stop on that externally)."""
function run!(sim::SimulationInstance; max_steps::Int=typemax(Int))
    steps = 0
    while sim.t < sim.cfg.t_end && steps < max_steps
        step!(sim)
        steps += 1
    end
    return sim
end

end # module Simulation
