_iris_spec_path() = joinpath(@__DIR__, "..", "Workflows", "assets", "aircraft", "iris_default.toml")

function iris_spec_for_tests(; strict::Bool = true)
    return Aircraft.load_spec(_iris_spec_path(); strict = strict)
end

iris_home_for_tests() = iris_spec_for_tests().home
iris_contact_for_tests() = iris_spec_for_tests().plant.contact

function iris_env_live_for_tests(; home = nothing)
    spec = iris_spec_for_tests()
    env = Aircraft._build_env_live(spec)
    if home === nothing
        return env
    end
    return Environment.EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = env.wind,
        gravity = env.gravity,
        origin = home,
    )
end

function iris_env_replay_for_tests(; home = nothing)
    spec = iris_spec_for_tests()
    env = Aircraft._build_env_replay(spec)
    if home === nothing
        return env
    end
    return Environment.EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = env.wind,
        gravity = env.gravity,
        origin = home,
    )
end

function iris_vehicle_for_tests(; x0_override = nothing, x0 = nothing)
    spec = iris_spec_for_tests()
    x0_use = x0 === nothing ? x0_override : x0
    return Aircraft._build_vehicle(spec; x0_override = x0_use)
end

function iris_batteries_for_tests()
    spec = iris_spec_for_tests()
    return Aircraft._build_batteries(spec)
end

iris_battery_for_tests() = iris_batteries_for_tests()[1]

function iris_dynfun_for_tests(env, vehicle::Vehicles.VehicleInstance, battery; contact = Contacts.NoContact())
    return PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )
end

function iris_scenario_for_tests(; arm_time_s::Float64 = 1.0, mission_time_s::Float64 = 2.0)
    s = Scenario.EventScenario()
    Scenario.arm_at!(s, arm_time_s)
    Scenario.mission_start_at!(s, mission_time_s)
    return s
end

function iris_timeline_for_tests(;
    t_end_s = nothing,
    dt_autopilot_s = nothing,
    dt_wind_s = nothing,
    dt_log_s = nothing,
    dt_phys_s = nothing,
    scenario_source = nothing,
)
    spec = iris_spec_for_tests()
    t_end_s = t_end_s === nothing ? spec.timeline.t_end_s : t_end_s
    dt_autopilot_s = dt_autopilot_s === nothing ? spec.timeline.dt_autopilot_s : dt_autopilot_s
    dt_wind_s = dt_wind_s === nothing ? spec.timeline.dt_wind_s : dt_wind_s
    dt_log_s = dt_log_s === nothing ? spec.timeline.dt_log_s : dt_log_s
    dt_phys_s = dt_phys_s === nothing ? spec.timeline.dt_phys_s : dt_phys_s

    t0_us = UInt64(0)
    t_end_us = Runtime.dt_to_us(t_end_s)
    dt_ap_us = Runtime.dt_to_us(dt_autopilot_s)
    dt_wind_us = Runtime.dt_to_us(dt_wind_s)
    dt_log_us = Runtime.dt_to_us(dt_log_s)
    dt_phys_us = dt_phys_s === nothing ? nothing : Runtime.dt_to_us(dt_phys_s)

    return Runtime.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scenario = scenario_source,
    )
end

function integrator_from_symbol(name::Symbol)
    if name === :Euler
        return Integrators.EulerIntegrator()
    elseif name === :RK4
        return Integrators.RK4Integrator()
    elseif name === :RK23
        return Integrators.RK23Integrator()
    elseif name === :RK45
        return Integrators.RK45Integrator()
    end
    error("Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)")
end

build_battery_spec(; kwargs...) = Aircraft.build_battery(Aircraft.BatterySpec(; kwargs...))
build_thevenin(; kwargs...) = build_battery_spec(model = :thevenin; kwargs...)
build_ideal(; kwargs...) = build_battery_spec(model = :ideal; kwargs...)

"""Test-only prop model that makes thrust/torque depend on Vax sign."""
struct SignProp <: Propulsion.AbstractPropParams
    kT::Float64
    kQ::Float64
end

Propulsion.prop_thrust(p::SignProp, _ρ::Float64, _ω::Float64, Vax::Float64) = p.kT * Vax
Propulsion.prop_torque(p::SignProp, _ρ::Float64, _ω::Float64, Vax::Float64) = p.kQ * Vax

"""Compute the L2 norm of a quaternion stored as a 4-vector."""
@inline function _qnorm(q::Types.Quat)::Float64
    return sqrt(sum(abs2, q))
end

@inline function quat_angle_error(q::Types.Quat, q_ref::Types.Quat)::Float64
    # Geodesic distance on SO(3), invariant to q ↦ -q.
    d = abs(sum(q .* q_ref))
    d = clamp(d, 0.0, 1.0)
    return 2.0 * acos(d)
end

"""A minimal deterministic dynamics: x-acceleration from cmd.motors[1]."""
struct CmdAccelX end

function (f::CmdAccelX)(t::Float64, x::RigidBody.RigidBodyState, u::Plant.PlantInput)
    a = u.cmd.motors[1]
    return RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Types.vec3(a, 0.0, 0.0),
        q_dot = RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Types.vec3(0.0, 0.0, 0.0),
    )
end

"""Deterministic dynamics driven by wind samples (for record/replay tests)."""
struct WindAccelX end

function (f::WindAccelX)(t::Float64, x::RigidBody.RigidBodyState, u::Plant.PlantInput)
    a = u.wind_ned[1]
    return RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Types.vec3(a, 0.0, 0.0),
        q_dot = RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Types.vec3(0.0, 0.0, 0.0),
    )
end

"""A trivial open-loop autopilot source used to record commands."""
mutable struct ConstantMotorAutopilotSource
    cmd::Vehicles.ActuatorCommand
end

function Runtime.update!(src::ConstantMotorAutopilotSource, bus::Runtime.SimBus, plant, t_us::UInt64)
    bus.cmd = src.cmd
    return nothing
end

"""Scenario source that steps wind disturbance at a specified boundary."""
struct WindDistScenario
    t_step_us::UInt64
    dist_ned::Types.Vec3
end

function Runtime.update!(src::WindDistScenario, bus::Runtime.SimBus, plant, t_us::UInt64)
    bus.ap_cmd = Autopilots.AutopilotCommand()
    bus.landed = false
    bus.faults = Faults.FaultState()
    bus.wind_dist_ned = t_us >= src.t_step_us ? src.dist_ned : Types.vec3(0.0, 0.0, 0.0)
    return nothing
end

struct ZeroRB end

function (d::ZeroRB)(t_s::Float64, x::RigidBody.RigidBodyState, u::Plant.PlantInput)
    return RigidBody.rb_deriv_zero()
end

mutable struct ProbeOrderState
    seen::Vector{Float64}
end

struct ProbeScenarioSource
    state::ProbeOrderState
end

struct ProbeWindSource
    state::ProbeOrderState
end

struct ProbeEstimatorSource
    state::ProbeOrderState
end

struct ProbeAutopilotSource
    state::ProbeOrderState
end

function Runtime.update!(src::ProbeScenarioSource, bus::Runtime.SimBus, plant, t_us::UInt64)
    bus.wind_ned = Types.vec3(1.0, 0.0, 0.0)
    return nothing
end

function Runtime.update!(src::ProbeWindSource, bus::Runtime.SimBus, plant, t_us::UInt64)
    push!(src.state.seen, bus.wind_ned[1])
    bus.wind_ned = Types.vec3(2.0, 0.0, 0.0)
    return nothing
end

function Runtime.update!(src::ProbeEstimatorSource, bus::Runtime.SimBus, plant, t_us::UInt64)
    push!(src.state.seen, bus.wind_ned[1])
    bus.wind_ned = Types.vec3(3.0, 0.0, 0.0)
    return nothing
end

function Runtime.update!(src::ProbeAutopilotSource, bus::Runtime.SimBus, plant, t_us::UInt64)
    push!(src.state.seen, bus.wind_ned[1])
    bus.wind_ned = Types.vec3(4.0, 0.0, 0.0)
    bus.cmd = Vehicles.ActuatorCommand()
    return nothing
end

"""Build a minimal Iris full-plant model (CoupledMultirotorModel) for tests."""
function _iris_fullplant(
    ;
    x0::RigidBody.RigidBodyState = RigidBody.RigidBodyState(),
    contact = Contacts.NoContact(),
    linear_drag::Union{Nothing,Float64} = nothing,
    angular_damping::Union{Nothing,Types.Vec3} = nothing,
)
    veh = iris_vehicle_for_tests(; x0 = x0)
    if linear_drag !== nothing || angular_damping !== nothing
        params = veh.model.params
        params_new = Vehicles.QuadrotorParams{4}(
            mass = params.mass,
            inertia_kgm2 = params.inertia_kgm2,
            inertia_inv_kgm2 = params.inertia_inv_kgm2,
            rotor_pos_body = params.rotor_pos_body,
            rotor_axis_body = params.rotor_axis_body,
            rotor_inertia_kgm2 = params.rotor_inertia_kgm2,
            rotor_dir = params.rotor_dir,
            linear_drag = linear_drag === nothing ? params.linear_drag : linear_drag,
            angular_damping =
                angular_damping === nothing ? params.angular_damping : angular_damping,
        )
        veh.model = Vehicles.GenericMultirotor{4}(params_new)
    end
    batt = iris_battery_for_tests()
    env = iris_env_replay_for_tests()

    model = PlantModels.CoupledMultirotorModel(
        veh.model,
        env,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )
    plant0 = Plant.init_plant_state(
        veh.state,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )

    return (veh = veh, batt = batt, env = env, model = model, plant0 = plant0)
end
