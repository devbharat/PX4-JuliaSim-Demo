"""Aircraft specs.

Goal
----
The spec layer exists to separate:

* **plant integration** (continuous dynamics + integrator + full plant state)
from
* **aircraft composition** (instances of components + their connections)

The initial scaffolding introduced a stable `build_engine(spec; ...)` entrypoint.
The spec layer now makes the composition intent concrete by adding:
* explicit **component instances** (motors, batteries, sensors)
* explicit **connections** (power buses wiring motors to batteries, etc.)

The simulation core remains aircraft-agnostic; aircraft-specific defaults live in
`PX4Lockstep.Workflows`.
"""

using StaticArrays

using ..Autopilots
using ..Runtime
using ..Integrators
using ..Contacts
using ..Types
using ..RigidBody

import ...LockstepConfig

# -----------------------
# Instance identifier tags
# -----------------------

# Use `Symbol` IDs for low-friction authoring. These may be wrapped in small
# newtypes later for clearer errors and type safety.
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

"""Propulsion model parameters (simple default motor+prop set).

Kept intentionally small: the builder uses
`Propulsion.default_multirotor_set(...)` with a hover-thrust derived from
airframe mass and motor count.
"""
Base.@kwdef struct PropulsionSpec
    kind::Symbol = :multirotor_default
    km_m::Float64 = 0.05
    V_nom::Float64 = 12.0
    rho_nom::Float64 = 1.225
    rotor_radius_m::Float64 = 0.127
    inflow_kT::Float64 = 8.0
    inflow_kQ::Float64 = 8.0
    """Optional override of yaw reaction torque sign pattern (+1/-1 per motor)."""
    rotor_dir::Union{Nothing,Vector{Float64}} = nothing
end

"""Airframe spec (multirotor only)."""
Base.@kwdef struct AirframeSpec
    kind::Symbol = :multirotor

    mass_kg::Float64 = 1.0
    inertia_diag_kgm2::Vec3 = vec3(1.0, 1.0, 1.0)
    rotor_pos_body_m::Vector{Vec3} = Vec3[]

    """Per-rotor axis vectors in body frame (unit).

    Conventions
    -----------
    Body axes are **FRD**: X forward, Y right, Z down.

    The rigid-body dynamics interpret this as the *propulsor axis* `axis_b` such that the
    thrust force applied to the vehicle is `F_i = -T_i * axis_b[i]`.

    This must be provided explicitly (one axis per rotor). For a classic multirotor,
    use `(0,0,1)` for all rotors (thrust along **-body Z**).
    """
    rotor_axis_body_m::Vector{Vec3} = Vec3[]
    linear_drag::Float64 = 0.0
    angular_damping::Vec3 = vec3(0.0, 0.0, 0.0)

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

"""Battery model spec (one entry per battery)."""
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

Base.@kwdef struct PX4ParamSpec
    name::String
    value::Union{Int32,Float32}
end

PX4ParamSpec(name::Symbol, value::Integer) = PX4ParamSpec(String(name), Int32(value))
PX4ParamSpec(name::Symbol, value::AbstractFloat) =
    PX4ParamSpec(String(name), Float32(value))
PX4ParamSpec(name::AbstractString, value::Integer) =
    PX4ParamSpec(String(name), Int32(value))
PX4ParamSpec(name::AbstractString, value::AbstractFloat) =
    PX4ParamSpec(String(name), Float32(value))


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

    """Path to `libpx4_lockstep`."""
    libpath::Union{Nothing,String} = nothing

    """PX4 lockstep configuration (rates + feature toggles)."""
    lockstep_config::LockstepConfig = LockstepConfig()

    """uORB pub/sub contract used by the PX4 bridge (required for live/record)."""
    uorb_cfg::Union{Nothing,Autopilots.PX4UORBInterfaceConfig} = nothing

    """Optional PX4 parameter overrides applied at init-time."""
    params::Vector{PX4ParamSpec} = PX4ParamSpec[]

    """Derive and apply control allocator (CA_*) parameters from the aircraft spec."""
    derive_ca_params::Bool = true

    """Edge-triggered actuator command semantics (PX4 internal detail)."""
    edge_trigger::Bool = false
end

Base.@kwdef struct TimelineSpec
    """Simulation end time (seconds)."""
    t_end_s::Float64 = 20.0

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
    contact::Contacts.AbstractContactModel = Contacts.FlatGroundContact()
end

Base.@kwdef struct AircraftSpec
    """Spec name tag (freeform identifier)."""
    name::Symbol = :aircraft

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
    home::Autopilots.HomeLocation = Autopilots.HomeLocation()

    """Optional telemetry sink (no-op by default)."""
    telemetry::Runtime.AbstractTelemetrySink = Runtime.NullTelemetry()

    """Optional log sinks (passed through to Runtime.Engine)."""
    log_sinks = nothing
end


# -----------------------
# Generic multirotor presets (internal)
# -----------------------

"""The canonical generic multirotor defaults live in a TOML asset.

See `default_multirotor_spec()` for the internal TOML-backed defaults.
"""

"""Return a minimal octacopter aircraft spec.

This is primarily intended as a *composition / plant* regression target:
- `airframe.kind=:multirotor`
- 8 motors mapped to PX4 channels 1..8
- rotor positions on a circle (radius configurable)

Note: Running this against PX4 will require PX4-side configuration (allocator, mixer,
etc.) and an explicit `uorb_cfg`. This spec is **experimental / demo-only** and
untested against PX4. This remains a Julia-side plant/framework regression target.
"""
function octa_spec(;
    mission_path::Union{Nothing,AbstractString} = nothing,
    libpath::Union{Nothing,AbstractString} = nothing,
    lockstep_config = LockstepConfig(),
    uorb_cfg::Union{Nothing,Autopilots.PX4UORBInterfaceConfig} = nothing,
    t_end_s::Float64 = 20.0,
    dt_autopilot_s::Float64 = 0.004,
    dt_wind_s::Float64 = 0.001,
    dt_log_s::Float64 = 0.01,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    seed::Integer = 1,
    integrator::Union{Symbol,Integrators.AbstractIntegrator} = :RK45,
    home = Autopilots.HomeLocation(),
    contact = Contacts.FlatGroundContact(),
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

    motors = MotorSpec[MotorSpec(id = Symbol("motor$(i)"), channel = i) for i = 1:8]

    actuation = ActuationSpec(
        motors = motors,
        servos = ServoSpec[],
        motor_actuators = DirectActuatorSpec(),
        servo_actuators = DirectActuatorSpec(),
    )

    batteries = BatterySpec[BatterySpec(
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
    ),]

    buses = PowerBusSpec[PowerBusSpec(
        id = :main,
        battery_ids = BatteryId[:bat1],
        motor_ids = MotorId[m.id for m in motors],
        avionics_load_w = 15.0,
    ),]

    power = PowerSpec(batteries = batteries, buses = buses)

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
        rotor_axis_body_m = Vec3[vec3(0.0, 0.0, 1.0) for _ = 1:8],
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
