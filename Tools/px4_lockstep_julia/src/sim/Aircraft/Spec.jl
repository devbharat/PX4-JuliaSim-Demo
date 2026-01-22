"""Aircraft specs (Phase 1).

Goal
----
The spec layer exists to separate:

* **plant integration** (continuous dynamics + integrator + full plant state)
from
* **aircraft composition** (instances of components + their connections)

Phase 0 introduced the minimum scaffolding so the existing Iris workflow could
route through a stable `build_engine(spec; ...)` entrypoint.

Phase 1 makes the composition intent concrete by adding:
* explicit **component instances** (motors, batteries, sensors)
* explicit **connections** (power buses wiring motors to batteries, etc.)

Behavior is still Iris-equivalent.
"""

using StaticArrays

using ..Autopilots
using ..Runtime
using ..Integrators
using ..Contacts
using ..Types
using ..RigidBody

import ...LockstepConfig

# Parent module handle (PX4Lockstep.Sim). This is used to call helper builders
# (legacy Iris defaults) without hard dependencies on include order.
const _SIM = parentmodule(@__MODULE__)


# -----------------------
# Instance identifier tags
# -----------------------

# Phase 1 uses `Symbol` IDs for zero-friction authoring. Later phases may wrap
# these in small newtypes for better error messages and type safety.
const MotorId = Symbol
const ServoId = Symbol
const BatteryId = Symbol
const GpsId = Symbol
const RangefinderId = Symbol
const RadarId = Symbol


# ----------------------
# Actuator model specs
# ----------------------

abstract type AbstractActuatorModelSpec end

"""Direct (algebraic) actuator model spec."""
struct DirectActuatorSpec <: AbstractActuatorModelSpec end

"""First-order actuator model spec: ẏ = (u - y)/τ."""
Base.@kwdef struct FirstOrderActuatorSpec <: AbstractActuatorModelSpec
    τ::Float64 = 0.05
end

"""Second-order actuator model spec with optional rate limiting."""
Base.@kwdef struct SecondOrderActuatorSpec <: AbstractActuatorModelSpec
    ωn::Float64 = 20.0
    ζ::Float64 = 0.7
    rate_limit::Float64 = Inf
end


# ----------------------
# Component instance specs
# ----------------------

Base.@kwdef struct MotorSpec
    id::MotorId
    """Index into the PX4 lockstep ABI motor array (1..12)."""
    channel::Int
end

Base.@kwdef struct ServoSpec
    id::ServoId
    """Index into the PX4 lockstep ABI servo array (1..8)."""
    channel::Int
end

"""Propulsion model parameters (Iris-like default motor+prop set).

Phase 1 keeps this intentionally small: the builder uses
`Propulsion.default_iris_quadrotor_set(...)` with a hover-thrust derived from
airframe mass and motor count.
"""
Base.@kwdef struct PropulsionSpec
    kind::Symbol = :iris_like
    km_m::Float64 = 0.05
    V_nom::Float64 = 12.0
    rho_nom::Float64 = 1.225
    rotor_radius_m::Float64 = 0.127
    inflow_kT::Float64 = 8.0
    inflow_kQ::Float64 = 8.0
    """Optional override of yaw reaction torque sign pattern (+1/-1 per motor)."""
    rotor_dir::Union{Nothing,Vector{Float64}} = nothing
end

"""Airframe spec.

Phase 1 supports the Iris quadrotor rigid-body model.
"""
Base.@kwdef struct AirframeSpec
    kind::Symbol = :iris_quadrotor

    mass_kg::Float64 = 1.5
    inertia_diag_kgm2::Vec3 = vec3(0.029125, 0.029125, 0.055225)
    rotor_pos_body_m::Vector{Vec3} = Vec3[
        vec3(0.1515, 0.2450, 0.0),
        vec3(-0.1515, -0.1875, 0.0),
        vec3(0.1515, -0.2450, 0.0),
        vec3(-0.1515, 0.1875, 0.0),
    ]
    linear_drag::Float64 = 0.05
    angular_damping::Vec3 = vec3(0.02, 0.02, 0.01)

    """Initial rigid-body state for live/record runs."""
    x0::RigidBodyState = RigidBodyState()

    propulsion::PropulsionSpec = PropulsionSpec()
end

"""Actuation spec.

This describes *instances* (motors/servos) and how they correspond to the fixed-size
PX4 actuator arrays.
"""
Base.@kwdef struct ActuationSpec
    motors::Vector{MotorSpec} = MotorSpec[]
    servos::Vector{ServoSpec} = ServoSpec[]

    motor_actuators::AbstractActuatorModelSpec = DirectActuatorSpec()
    servo_actuators::AbstractActuatorModelSpec = DirectActuatorSpec()
end

"""Battery model spec (single-battery Phase 1)."""
Base.@kwdef struct BatterySpec
    id::BatteryId = :bat1
    model::Symbol = :thevenin

    capacity_ah::Float64 = 5.0
    soc0::Float64 = 1.0

    ocv_soc::Vector{Float64} = [0.0, 1.0]
    ocv_v::Vector{Float64} = [10.8, 12.6]

    r0::Float64 = 0.020
    r1::Float64 = 0.010
    c1::Float64 = 2000.0
    v1_0::Float64 = 0.0
    min_voltage_v::Float64 = 9.9

    low_thr::Float64 = 0.15
    crit_thr::Float64 = 0.10
    emerg_thr::Float64 = 0.05
end

"""Power bus wiring spec.

This is a functional abstraction (not a wiring harness simulator): it captures which
batteries power which loads.
"""
Base.@kwdef struct PowerBusSpec
    id::Symbol = :main
    battery_ids::Vector{BatteryId} = BatteryId[]
    motor_ids::Vector{MotorId} = MotorId[]
    servo_ids::Vector{ServoId} = ServoId[]
    avionics_load_w::Float64 = 0.0
end

"""Power system spec."""
Base.@kwdef struct PowerSpec
    batteries::Vector{BatterySpec} = BatterySpec[]
    buses::Vector{PowerBusSpec} = PowerBusSpec[]

    """Battery used for legacy single-battery injection paths."""
    primary_battery::BatteryId = :bat1
end


# ----------------------
# Sensor spec placeholders
# ----------------------

abstract type AbstractSensorSpec end

Base.@kwdef struct GpsSpec <: AbstractSensorSpec
    id::GpsId
    uorb_instance::Int = 0
end

Base.@kwdef struct RangefinderSpec <: AbstractSensorSpec
    id::RangefinderId
    uorb_instance::Int = 0
end

Base.@kwdef struct RadarSpec <: AbstractSensorSpec
    id::RadarId
    uorb_instance::Int = 0
end


# ----------------
# Core spec structs
# ----------------

Base.@kwdef struct PX4Spec
    """Path to the PX4 mission file."""
    mission_path::Union{Nothing,String} = nothing

    """Path to `libpx4_lockstep` (or `nothing` to use `ENV[PX4_LOCKSTEP_LIB]` / auto-find)."""
    libpath::Union{Nothing,String} = nothing

    """PX4 lockstep configuration (rates + feature toggles)."""
    lockstep_config::LockstepConfig = _SIM.iris_default_lockstep_config()

    """uORB pub/sub contract used by the PX4 bridge."""
    uorb_cfg::Autopilots.PX4UORBInterfaceConfig = Autopilots.iris_state_injection_interface()

    """Edge-triggered actuator command semantics (PX4 internal detail)."""
    edge_trigger::Bool = false
end

Base.@kwdef struct TimelineSpec
    """Simulation end time (seconds)."""
    t_end_s::Float64 = parse(Float64, get(ENV, "IRIS_T_END_S", "20.0"))

    """Autopilot cadence (seconds)."""
    dt_autopilot_s::Float64 = 0.004

    """Wind disturbance cadence (seconds)."""
    dt_wind_s::Float64 = 0.001

    """Logging cadence (seconds)."""
    dt_log_s::Float64 = 0.01

    """Optional physics cadence for debugging (seconds)."""
    dt_phys_s::Union{Nothing,Float64} = nothing
end

Base.@kwdef struct PlantSpec
    """Integrator used for plant integration."""
    integrator::Union{Symbol,Integrators.AbstractIntegrator} = :RK45

    """Contact model used by the plant dynamics."""
    contact::Contacts.AbstractContactModel = _SIM.iris_default_contact()
end

Base.@kwdef struct AircraftSpec
    """Spec name / preset tag (Phase 1 supports only :iris)."""
    name::Symbol = :iris

    px4::PX4Spec = PX4Spec()
    timeline::TimelineSpec = TimelineSpec()
    plant::PlantSpec = PlantSpec()

    airframe::AirframeSpec = AirframeSpec()
    actuation::ActuationSpec = ActuationSpec()
    power::PowerSpec = PowerSpec()
    sensors::Vector{AbstractSensorSpec} = AbstractSensorSpec[]

    """RNG seed for deterministic live sources (wind/estimator)."""
    seed::Int = 1

    """Home / local origin used for PX4 and NED conversion."""
    home::Autopilots.HomeLocation = _SIM.iris_default_home()

    """Optional telemetry sink (no-op by default)."""
    telemetry::Runtime.AbstractTelemetrySink = Runtime.NullTelemetry()

    """Optional log sinks (passed through to Runtime.Engine)."""
    log_sinks = nothing
end


# -----------------------
# Iris convenience preset
# -----------------------

"""Return an Iris aircraft spec with the same defaults as `simulate_iris_mission`.

This is a thin data wrapper: it does not run any simulation.
"""
function iris_spec(; 
    mission_path::Union{Nothing,AbstractString} = get(ENV, "PX4_LOCKSTEP_MISSION", nothing),
    libpath::Union{Nothing,AbstractString} = get(ENV, "PX4_LOCKSTEP_LIB", nothing),
    lockstep_config = _SIM.iris_default_lockstep_config(),
    uorb_cfg::Autopilots.PX4UORBInterfaceConfig = Autopilots.iris_state_injection_interface(),
    t_end_s::Float64 = parse(Float64, get(ENV, "IRIS_T_END_S", "20.0")),
    dt_autopilot_s::Float64 = 0.004,
    dt_wind_s::Float64 = 0.001,
    dt_log_s::Float64 = 0.01,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    seed::Integer = 1,
    integrator::Union{Symbol,Integrators.AbstractIntegrator} = :RK45,
    home = _SIM.iris_default_home(),
    contact = _SIM.iris_default_contact(),
    telemetry = Runtime.NullTelemetry(),
    log_sinks = nothing,
)
    px4 = PX4Spec(
        mission_path = mission_path === nothing ? nothing : String(mission_path),
        libpath = libpath === nothing ? nothing : String(libpath),
        lockstep_config = lockstep_config,
        uorb_cfg = uorb_cfg,
        edge_trigger = false,
    )

    timeline = TimelineSpec(
        t_end_s = t_end_s,
        dt_autopilot_s = dt_autopilot_s,
        dt_wind_s = dt_wind_s,
        dt_log_s = dt_log_s,
        dt_phys_s = dt_phys_s,
    )

    plant = PlantSpec(integrator = integrator, contact = contact)

    # --- Iris component instances ---
    motors = MotorSpec[
        MotorSpec(id = :motor1, channel = 1),
        MotorSpec(id = :motor2, channel = 2),
        MotorSpec(id = :motor3, channel = 3),
        MotorSpec(id = :motor4, channel = 4),
    ]

    actuation = ActuationSpec(
        motors = motors,
        servos = ServoSpec[],
        motor_actuators = DirectActuatorSpec(),
        servo_actuators = DirectActuatorSpec(),
    )

    batteries = BatterySpec[
        BatterySpec(
            id = :bat1,
            model = :thevenin,
            capacity_ah = 5.0,
            soc0 = 1.0,
            ocv_soc = [0.0, 1.0],
            ocv_v = [10.8, 12.6],
            r0 = 0.020,
            r1 = 0.010,
            c1 = 2000.0,
            v1_0 = 0.0,
            min_voltage_v = 9.9,
        ),
    ]

    buses = PowerBusSpec[
        PowerBusSpec(
            id = :main,
            battery_ids = BatteryId[:bat1],
            motor_ids = MotorId[:motor1, :motor2, :motor3, :motor4],
            servo_ids = ServoId[],
            avionics_load_w = 0.0,
        ),
    ]

    power = PowerSpec(batteries = batteries, buses = buses, primary_battery = :bat1)

    airframe = AirframeSpec(
        kind = :iris_quadrotor,
        mass_kg = 1.5,
        inertia_diag_kgm2 = vec3(0.029125, 0.029125, 0.055225),
        rotor_pos_body_m = Vec3[
            vec3(0.1515, 0.2450, 0.0),
            vec3(-0.1515, -0.1875, 0.0),
            vec3(0.1515, -0.2450, 0.0),
            vec3(-0.1515, 0.1875, 0.0),
        ],
        linear_drag = 0.05,
        angular_damping = vec3(0.02, 0.02, 0.01),
        x0 = RigidBodyState(),
        propulsion = PropulsionSpec(kind = :iris_like, km_m = 0.05),
    )

    return AircraftSpec(
        name = :iris,
        px4 = px4,
        timeline = timeline,
        plant = plant,
        airframe = airframe,
        actuation = actuation,
        power = power,
        sensors = AbstractSensorSpec[],
        seed = Int(seed),
        home = home,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )
end


# -----------------------
# Generic multirotor presets (Phase 2)
# -----------------------

"""Return a minimal octacopter aircraft spec (Phase 2).

This is primarily intended as a *composition / plant* regression target:
- `airframe.kind=:multirotor`
- 8 motors mapped to PX4 channels 1..8
- rotor positions on a circle (radius configurable)

Note: Running this against PX4 will require PX4-side configuration (allocator, mixer,
etc.). Phase 2 focuses on the Julia plant/framework side.
"""
function octa_spec(;
    mission_path::Union{Nothing,AbstractString} = nothing,
    libpath::Union{Nothing,AbstractString} = get(ENV, "PX4_LOCKSTEP_LIB", nothing),
    lockstep_config = _SIM.iris_default_lockstep_config(),
    uorb_cfg::Autopilots.PX4UORBInterfaceConfig = Autopilots.iris_state_injection_interface(),
    t_end_s::Float64 = 20.0,
    dt_autopilot_s::Float64 = 0.004,
    dt_wind_s::Float64 = 0.001,
    dt_log_s::Float64 = 0.01,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    seed::Integer = 1,
    integrator::Union{Symbol,Integrators.AbstractIntegrator} = :RK45,
    home = _SIM.iris_default_home(),
    contact = _SIM.iris_default_contact(),
    telemetry = Runtime.NullTelemetry(),
    log_sinks = nothing,
    # Geometry / mass properties
    radius_m::Float64 = 0.25,
    mass_kg::Float64 = 2.0,
    inertia_diag_kgm2::Vec3 = vec3(0.05, 0.05, 0.10),
)
    px4 = PX4Spec(
        mission_path = mission_path === nothing ? nothing : String(mission_path),
        libpath = libpath === nothing ? nothing : String(libpath),
        lockstep_config = lockstep_config,
        uorb_cfg = uorb_cfg,
        edge_trigger = false,
    )

    timeline = TimelineSpec(
        t_end_s = t_end_s,
        dt_autopilot_s = dt_autopilot_s,
        dt_wind_s = dt_wind_s,
        dt_log_s = dt_log_s,
        dt_phys_s = dt_phys_s,
    )

    plant = PlantSpec(integrator = integrator, contact = contact)

    motors = MotorSpec[
        MotorSpec(id = Symbol("motor$(i)"), channel = i) for i = 1:8
    ]

    actuation = ActuationSpec(
        motors = motors,
        servos = ServoSpec[],
        motor_actuators = DirectActuatorSpec(),
        servo_actuators = DirectActuatorSpec(),
    )

    batteries = BatterySpec[
        BatterySpec(
            id = :bat1,
            model = :thevenin,
            capacity_ah = 5.0,
            soc0 = 1.0,
            ocv_soc = [0.0, 1.0],
            ocv_v = [10.8, 12.6],
            r0 = 0.020,
            r1 = 0.010,
            c1 = 2000.0,
            v1_0 = 0.0,
            min_voltage_v = 9.9,
        ),
    ]

    buses = PowerBusSpec[
        PowerBusSpec(
            id = :main,
            battery_ids = BatteryId[:bat1],
            motor_ids = MotorId[m.id for m in motors],
            avionics_load_w = 15.0,
        ),
    ]

    power = PowerSpec(
        batteries = batteries,
        buses = buses,
        primary_battery = :bat1,
    )

    rotor_pos_body_m = Vec3[
        vec3(
            radius_m * cos(2.0 * pi * (Float64(i - 1) / 8.0)),
            radius_m * sin(2.0 * pi * (Float64(i - 1) / 8.0)),
            0.0,
        ) for i = 1:8
    ]

    airframe = AirframeSpec(
        kind = :multirotor,
        mass_kg = mass_kg,
        inertia_diag_kgm2 = inertia_diag_kgm2,
        rotor_pos_body_m = rotor_pos_body_m,
    )

    return AircraftSpec(
        name = :octa,
        px4 = px4,
        timeline = timeline,
        plant = plant,
        airframe = airframe,
        actuation = actuation,
        power = power,
        sensors = AbstractSensorSpec[],
        seed = Int(seed),
        home = home,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )
end
