"""PX4Lockstep.Sim.Simulation

Deterministic single-threaded simulation engine.

This simulator is meant to run PX4 in *lockstep* in the same process as the plant.

Core rules (enforced by code):
* The physics loop advances with a fixed `dt`.
* All multi-rate tasks (autopilot, estimator, logging, ... ) are scheduled using an
  **integer step counter** (no Float64 time comparisons) to avoid cadence jitter.
* PX4 time is injected as an *exact* microsecond counter derived from the step counter.
"""
module Simulation

using ..Types: Vec3, Quat, WorldOrigin, vec3, quat_rotate_inv
using ..RigidBody: RigidBodyState, RigidBodyDeriv
using ..Environment:
    EnvironmentModel, SampledWind, wind_velocity, air_density, step_wind!, sample_wind!
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, step_actuators!, dynamics, mass
using ..Propulsion: step_propulsion!
using ..Integrators: AbstractIntegrator, step_integrator
using ..Autopilots:
    AbstractAutopilot,
    AutopilotCommand,
    autopilot_step,
    autopilot_output_type,
    max_internal_rate_hz,
    PX4LockstepAutopilot
using ..Estimators: AbstractEstimator, TruthEstimator, estimate!
using ..Scenario: AbstractScenario, ScriptedScenario, scenario_step
import ..Powertrain
using ..Powertrain: AbstractBatteryModel, IdealBattery, BatteryStatus, status
using ..Logging: AbstractLogSink, SimLog, log!, close!, reserve!
using ..Scheduling: StepTrigger, due
using ..Contacts: AbstractContactModel, NoContact, contact_force_ned
using Random
using StaticArrays

export VehicleInstance, SimulationConfig, SimulationInstance, time_s, time_us, step!, run!

############################
# Simulation types
############################

"""Vehicle instance = model + propulsion state + rigid-body state."""
mutable struct VehicleInstance{M<:AbstractVehicleModel,AM,AS,P}
    model::M
    motor_actuators::AM
    servo_actuators::AS
    propulsion::P
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

    # Safety guardrails.
    #
    # If true (default), fail fast when `dt_autopilot` is slower than the fastest
    # configured internal autopilot task rate. Running those tasks slower than
    # configured is deterministic, but it violates the intended "PX4-in-lockstep"
    # timing contract and can invalidate stability/performance conclusions.
    strict_lockstep_rates::Bool = true
end

############################
# Internal helpers
############################

@inline function _sanitize_cmd(x, lo::Float64, hi::Float64)
    isfinite(x) ? clamp(x, lo, hi) : 0.0
end

"""Convert a step size to an *exact* integer microsecond count.

We require the user's `dt` to be representable as an integer number of microseconds.
This is the simplest robust contract for PX4 lockstep time injection.
"""
function _dt_to_us(dt::Float64; name::AbstractString = "dt")::Int
    dt > 0 || throw(ArgumentError("$name must be > 0"))
    dt_us = Int(round(dt * 1e6))
    dt_back = dt_us * 1e-6
    abs(dt - dt_back) <= 1e-12 || throw(
        ArgumentError(
            "$name must be an integer number of microseconds (got dt=$dt => $dt_us μs)",
        ),
    )
    dt_us > 0 || throw(ArgumentError("$name is too small (rounded to 0 μs)"))
    return dt_us
end

"""Convert a (non-negative) absolute time to an exact integer microsecond count."""
function _time_to_us(t::Float64; name::AbstractString = "t")::Int
    t >= 0 || throw(ArgumentError("$name must be >= 0"))
    t_us = Int(round(t * 1e6))
    t_back = t_us * 1e-6
    abs(t - t_back) <= 1e-12 || throw(
        ArgumentError(
            "$name must be an integer number of microseconds (got $t => $t_us μs)",
        ),
    )
    return t_us
end

"""Return integer steps for `dt_task`, enforcing that it is an integer multiple of `dt`."""
function _multiple_steps(dt_task::Float64, dt::Float64; name::AbstractString)::Int
    dt_task > 0 || throw(ArgumentError("$name must be > 0"))
    r = dt_task / dt
    k = Int(round(r))
    abs(r - k) <= 1e-12 || throw(
        ArgumentError("$name must be an integer multiple of dt (got $dt_task / $dt = $r)"),
    )
    k >= 1 || throw(ArgumentError("$name must be >= dt"))
    return k
end

"""Stable splitmix64 for deriving independent RNG seeds."""
@inline function _splitmix64(x::UInt64)::UInt64
    z = x + 0x9E3779B97F4A7C15
    z = (z ⊻ (z >> 30)) * 0xBF58476D1CE4E5B9
    z = (z ⊻ (z >> 27)) * 0x94D049BB133111EB
    return z ⊻ (z >> 31)
end

@inline function _seed_to_int(s::UInt64)::Int
    # Keep within Int range deterministically.
    return Int(s % UInt64(typemax(Int)))
end

"""Wrap the environment wind in a sample-and-hold adapter for RK4 determinism."""
function _wrap_sampled_wind(env::EnvironmentModel, pos_ned::Vec3, t::Float64)
    if env.wind isa SampledWind
        sample_wind!(env.wind, pos_ned, t)
        return env
    end

    sampled = SampledWind(env.wind)
    sample_wind!(sampled, pos_ned, t)
    return EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = sampled,
        gravity = env.gravity,
        origin = env.origin,
    )
end

@inline function _origin_matches(a::WorldOrigin, b::WorldOrigin; tol::Float64 = 1e-9)
    return isapprox(a.lat_deg, b.lat_deg; atol = tol) &&
           isapprox(a.lon_deg, b.lon_deg; atol = tol) &&
           isapprox(a.alt_msl_m, b.alt_msl_m; atol = tol)
end

"""Align PX4 home and environment origin when one side is default.

If both sides are custom and mismatch, emit a warning.
"""
function _sync_world_origin(env::EnvironmentModel, autopilot::AbstractAutopilot)
    autopilot isa PX4LockstepAutopilot || return env

    env_origin = env.origin
    ap_origin = autopilot.home
    _origin_matches(env_origin, ap_origin) && return env

    default_origin = WorldOrigin()
    env_default = _origin_matches(env_origin, default_origin)
    ap_default = _origin_matches(ap_origin, default_origin)

    if env_default && !ap_default
        return EnvironmentModel(
            atmosphere = env.atmosphere,
            wind = env.wind,
            gravity = env.gravity,
            origin = ap_origin,
        )
    elseif ap_default && !env_default
        autopilot.home = env_origin
        return env
    end

    @warn "World origin mismatch between environment and PX4 home; set them to match" env_origin=env_origin autopilot_home=ap_origin
    return env
end

function _make_rngs(seed::UInt64, rng_master::Union{Nothing,AbstractRNG})
    if rng_master === nothing
        s_wind = _splitmix64(seed ⊻ 0x57494E445F534545)  # "WIND_SEE"
        s_est = _splitmix64(seed ⊻ 0x4553545F53454544)   # "EST_SEED"
        s_misc = _splitmix64(seed ⊻ 0x4D4953435F534545)  # "MISC_SEE"
        return MersenneTwister(_seed_to_int(s_wind)),
        MersenneTwister(_seed_to_int(s_est)),
        MersenneTwister(_seed_to_int(s_misc))
    else
        # Deterministically derive seeds from the provided master RNG.
        s_wind = rand(rng_master, UInt64)
        s_est = rand(rng_master, UInt64)
        s_misc = rand(rng_master, UInt64)
        return MersenneTwister(_seed_to_int(s_wind)),
        MersenneTwister(_seed_to_int(s_est)),
        MersenneTwister(_seed_to_int(s_misc))
    end
end

############################
# Dynamics wrapper (avoids per-tick closure allocations)
############################

struct DynamicsWithContact{M,E,C}
    model::M
    env::E
    contact::C
end

@inline function (f::DynamicsWithContact)(t::Float64, x::RigidBodyState, u)
    d = dynamics(f.model, f.env, t, x, u)
    F_contact = contact_force_ned(f.contact, x, t)
    if (F_contact[1] != 0.0) || (F_contact[2] != 0.0) || (F_contact[3] != 0.0)
        d = RigidBodyDeriv(
            pos_dot = d.pos_dot,
            vel_dot = d.vel_dot + F_contact / mass(f.model),
            q_dot = d.q_dot,
            ω_dot = d.ω_dot,
        )
    end
    return d
end

############################
# Simulation instance
############################

"""A simulation instance."""
mutable struct SimulationInstance{E,V,A,EST,I,S,B,L,C,R,O,D}
    cfg::SimulationConfig
    env::E
    vehicle::V
    autopilot::A
    estimator::EST
    integrator::I
    scenario::S
    battery::B
    log::L
    contact::C

    # Deterministic RNG streams.
    rng_wind::R
    rng_est::R
    rng_misc::R

    # Timebase.
    step::Int
    t::Float64
    dt_us::Int
    t0_us::UInt64

    # Multi-rate schedules.
    ap_trig::StepTrigger
    log_trig::StepTrigger
    ap_dt::Float64

    # Autopilot output (sample-and-hold).
    last_out::Union{Nothing,O}

    # Whether the autopilot supports the `time_us::UInt64` stepping signature.
    ap_uses_time_us::Bool

    # Cached dynamics functor (avoids closure allocations).
    dynfun::D
end

"""Current simulation time in seconds."""
@inline time_s(sim::SimulationInstance) = sim.t

"""Current simulation time in microseconds (PX4 lockstep clock)."""
@inline function time_us(sim::SimulationInstance)::UInt64
    return sim.t0_us + UInt64(sim.step) * UInt64(sim.dt_us)
end

function SimulationInstance(;
    cfg::SimulationConfig = SimulationConfig(),
    env::EnvironmentModel = EnvironmentModel(),
    vehicle::VehicleInstance,
    autopilot::AbstractAutopilot,
    estimator::AbstractEstimator = TruthEstimator(),
    integrator::AbstractIntegrator,
    scenario::AbstractScenario = ScriptedScenario(),
    battery::AbstractBatteryModel = IdealBattery(),
    log::AbstractLogSink = SimLog(),
    contact::AbstractContactModel = NoContact(),
    rng::Union{Nothing,AbstractRNG} = nothing,
)
    cfg.dt > 0 || throw(ArgumentError("dt must be > 0"))
    cfg.t_end >= cfg.t0 || throw(ArgumentError("t_end must be >= t0"))

    # Enforce microsecond-quantized dt and t0.
    dt_us = _dt_to_us(cfg.dt; name = "dt")
    t0_us_i = _time_to_us(cfg.t0; name = "t0")
    t0_us = UInt64(t0_us_i)

    # Multi-rate steps.
    dt_ap = (cfg.dt_autopilot > 0) ? cfg.dt_autopilot : cfg.dt
    dt_log = (cfg.dt_log > 0) ? cfg.dt_log : cfg.dt
    dt_ap + 1e-12 >= cfg.dt || throw(ArgumentError("dt_autopilot must be >= dt"))
    dt_log + 1e-12 >= cfg.dt || throw(ArgumentError("dt_log must be >= dt"))
    ap_steps = _multiple_steps(dt_ap, cfg.dt; name = "dt_autopilot")
    log_steps = _multiple_steps(dt_log, cfg.dt; name = "dt_log")

    ap_trig = StepTrigger(ap_steps; offset_steps = 0)
    log_trig = StepTrigger(log_steps; offset_steps = 0)

    # Derived dt actually used for the estimator/autopilot cadence.
    ap_dt = ap_steps * cfg.dt

    # Guard against conceptually-inconsistent PX4 lockstep cadence.
    # If PX4 tasks are configured at (say) 250 Hz but we only step the lockstep lib at 100 Hz,
    # those loops will silently run too slowly.
    max_hz = max_internal_rate_hz(autopilot)
    if max_hz !== nothing
        dt_req = 1.0 / Float64(max_hz)
        if ap_dt > dt_req + 1e-12
            msg = "dt_autopilot is larger than the fastest configured autopilot task period; internal loops will run too slowly"
            if cfg.strict_lockstep_rates
                throw(
                    ArgumentError(
                        "$(msg). Set strict_lockstep_rates=false to override (dt_autopilot=$(ap_dt) s, required_dt=$(dt_req) s for max_task_rate_hz=$(max_hz)).",
                    ),
                )
            else
                @warn msg dt_autopilot=ap_dt required_dt=dt_req max_task_rate_hz=max_hz strict_lockstep_rates=cfg.strict_lockstep_rates
            end
        end
    end

    rng_wind, rng_est, rng_misc = _make_rngs(cfg.seed, rng)

    # Align PX4 home/origin before wrapping wind.
    env = _sync_world_origin(env, autopilot)

    # Hold wind constant over each physics step for deterministic integration.
    env = _wrap_sampled_wind(env, vehicle.state.pos_ned, cfg.t0)

    # Preallocate in-memory log capacity (nice perf win for long runs).
    if log isa SimLog
        n = Int(floor((cfg.t_end - cfg.t0) / (log_steps * cfg.dt))) + 2
        reserve!(log, n)
    end

    # Cache dynamics function.
    dynfun = DynamicsWithContact(vehicle.model, env, contact)

    # Autopilot output type for sample-and-hold.
    O = autopilot_output_type(autopilot)
    ap_uses_time_us = hasmethod(
        autopilot_step,
        Tuple{typeof(autopilot),UInt64,Vec3,Vec3,Quat,Vec3,AutopilotCommand},
    )

    E = typeof(env)
    V = typeof(vehicle)
    A = typeof(autopilot)
    EST = typeof(estimator)
    I = typeof(integrator)
    S = typeof(scenario)
    B = typeof(battery)
    L = typeof(log)
    C = typeof(contact)
    R = typeof(rng_wind)
    D = typeof(dynfun)

    return SimulationInstance{E,V,A,EST,I,S,B,L,C,R,O,D}(
        cfg,
        env,
        vehicle,
        autopilot,
        estimator,
        integrator,
        scenario,
        battery,
        log,
        contact,
        rng_wind,
        rng_est,
        rng_misc,
        0,
        cfg.t0,
        dt_us,
        t0_us,
        ap_trig,
        log_trig,
        ap_dt,
        nothing,
        ap_uses_time_us,
        dynfun,
    )
end

############################
# Main step
############################

"""Advance the simulation by one physics step (`cfg.dt`)."""
function step!(sim::SimulationInstance)
    dt = sim.cfg.dt
    step = sim.step
    t = sim.t

    # Snapshot the pre-step state for consistent logging.
    x0 = sim.vehicle.state

    # Advance environment disturbances (e.g. turbulence) deterministically.
    step_wind!(sim.env.wind, x0.pos_ned, t, dt, sim.rng_wind)

    # Scenario produces high-level autopilot commands and an approximate landed flag.
    cmd, landed = scenario_step(sim.scenario, t, x0, sim)

    # Sample wind once per tick so it remains constant across RK4 stages.
    wind_now = sample_wind!(sim.env.wind, x0.pos_ned, t)

    # Battery model -> PX4 battery_status injection (sampled at start of interval).
    batt = status(sim.battery)

    # Estimator + autopilot multi-rate step.
    if (sim.last_out === nothing) || due(sim.ap_trig, step)
        est = estimate!(sim.estimator, sim.rng_est, t, x0, sim.ap_dt)
        if sim.ap_uses_time_us
            sim.last_out = autopilot_step(
                sim.autopilot,
                time_us(sim),
                est.pos_ned,
                est.vel_ned,
                est.q_bn,
                est.ω_body,
                cmd;
                landed = landed,
                battery = batt,
            )
        else
            sim.last_out = autopilot_step(
                sim.autopilot,
                t,
                est.pos_ned,
                est.vel_ned,
                est.q_bn,
                est.ω_body,
                cmd;
                landed = landed,
                battery = batt,
            )
        end
    end

    out = sim.last_out
    out === nothing && error("autopilot produced no output")

    # Extract actuator commands from PX4 output.
    m_raw = getproperty(out, :actuator_motors)
    s_raw = getproperty(out, :actuator_servos)

    motors =
        SVector{12,Float64}(ntuple(i -> _sanitize_cmd(Float64(m_raw[i]), 0.0, 1.0), 12))
    servos = SVector{8,Float64}(ntuple(i -> _sanitize_cmd(Float64(s_raw[i]), -1.0, 1.0), 8))

    # Apply actuator dynamics (if any) independently for motors and servos.
    motors = step_actuators!(sim.vehicle.motor_actuators, motors, dt)
    servos = step_actuators!(sim.vehicle.servo_actuators, servos, dt)

    u_cmd = ActuatorCommand(motors = motors, servos = servos)

    # Compute air-relative velocity in body frame for inflow-aware propulsion.
    v_air_ned = wind_now - x0.vel_ned
    v_air_body = quat_rotate_inv(x0.q_bn, v_air_ned)

    # Propulsion: duty → ω → thrust/torque/current.
    p = sim.vehicle.propulsion
    p === nothing &&
        error("Simulation requires a propulsion model; attach to VehicleInstance.")
    N = length(p.units)
    duties = SVector{N,Float64}(ntuple(i -> motors[i], N))
    # Atmosphere expects MSL altitude. NED origin is typically at the home location.
    alt_msl_m = sim.env.origin.alt_msl_m - x0.pos_ned[3]
    ρ = air_density(sim.env.atmosphere, alt_msl_m)
    prop_out = step_propulsion!(p, duties, Float64(batt.voltage_v), ρ, v_air_body, dt)
    I_bus = prop_out.bus_current_a

    # Integrate rigid body (with optional contact forces).
    new_state = step_integrator(sim.integrator, sim.dynfun, t, x0, prop_out, dt)
    sim.vehicle.state = new_state

    # Update battery based on modeled bus current.
    Powertrain.step!(sim.battery, I_bus, dt)

    # Logging (multi-rate). Contract: log the *pre-step* state at time t.
    if due(sim.log_trig, step)
        # Environment sampled at the log time/state.
        wind = wind_now
        alt_msl_m_log = sim.env.origin.alt_msl_m - x0.pos_ned[3]
        rho = air_density(sim.env.atmosphere, alt_msl_m_log)
        v_air_body_log = v_air_body

        # PX4 setpoints (if present).
        pos_sp = (
            Float64(getproperty(out, :trajectory_setpoint_position)[1]),
            Float64(getproperty(out, :trajectory_setpoint_position)[2]),
            Float64(getproperty(out, :trajectory_setpoint_position)[3]),
        )
        vel_sp = (
            Float64(getproperty(out, :trajectory_setpoint_velocity)[1]),
            Float64(getproperty(out, :trajectory_setpoint_velocity)[2]),
            Float64(getproperty(out, :trajectory_setpoint_velocity)[3]),
        )
        acc_sp = (
            Float64(getproperty(out, :trajectory_setpoint_acceleration)[1]),
            Float64(getproperty(out, :trajectory_setpoint_acceleration)[2]),
            Float64(getproperty(out, :trajectory_setpoint_acceleration)[3]),
        )

        yaw_sp = Float64(getproperty(out, :trajectory_setpoint_yaw))
        yawspeed_sp = Float64(getproperty(out, :trajectory_setpoint_yawspeed))

        # Convenience rotor logging for quads.
        rotor_ω = (NaN, NaN, NaN, NaN)
        rotor_T = (NaN, NaN, NaN, NaN)
        if N >= 4
            rotor_ω = (
                prop_out.ω_rad_s[1],
                prop_out.ω_rad_s[2],
                prop_out.ω_rad_s[3],
                prop_out.ω_rad_s[4],
            )
            rotor_T = (
                prop_out.thrust_n[1],
                prop_out.thrust_n[2],
                prop_out.thrust_n[3],
                prop_out.thrust_n[4],
            )
        end

        log!(
            sim.log,
            t,
            x0,
            u_cmd;
            time_us = time_us(sim),
            wind_ned = wind,
            rho = rho,
            air_vel_body = (v_air_body_log[1], v_air_body_log[2], v_air_body_log[3]),
            battery = batt,
            nav_state = Int32(getproperty(out, :nav_state)),
            arming_state = Int32(getproperty(out, :arming_state)),
            mission_seq = Int32(getproperty(out, :mission_seq)),
            mission_count = Int32(getproperty(out, :mission_count)),
            mission_finished = Int32(getproperty(out, :mission_finished)),
            pos_sp = pos_sp,
            vel_sp = vel_sp,
            acc_sp = acc_sp,
            yaw_sp = yaw_sp,
            yawspeed_sp = yawspeed_sp,
            rotor_omega = rotor_ω,
            rotor_thrust = rotor_T,
        )
    end

    # Advance exact integer timebase.
    sim.step = step + 1
    sim.t = sim.cfg.t0 + sim.step * sim.cfg.dt
    return nothing
end

"""Run until `cfg.t_end`."""
function run!(
    sim::SimulationInstance;
    max_steps::Int = typemax(Int),
    close_log::Bool = true,
)
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
