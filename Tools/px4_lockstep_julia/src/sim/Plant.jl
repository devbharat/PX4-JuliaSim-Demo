"""PX4Lockstep.Sim.Plant

Continuous-time plant state for variable-step integration.

Motivation
----------
The existing sim loop updates several subsystems *discretely* once per physics tick
(actuators, propulsion rotor speed, battery polarization), while integrating only the
rigid body with an ODE solver.

To support a **full-fledged variable-step integrator**, a single “plant state”
containing *all* continuous states is required so adaptive solvers can control error
across the whole plant, not only the rigid body.

This module provides:
  * A `PlantState` and `PlantDeriv` data model suitable for allocation-free integration.
  * A minimal `PlantInput` and `PlantOutputs` (algebraic coupling outputs).
  * State math helpers (`plant_add`, `plant_scale_add`, `plant_lincomb`) used by ODE
    integrators.

Design constraints
------------------
* Determinism: ODE RHS evaluation must be a **pure** function of (t, x, u) with no RNG.
* Inputs are piecewise constant between discrete event boundaries (autopilot, wind,
  failures). Adaptive substeps must never touch RNG or mutate shared state.
* Subsystem model objects may remain for convenience, but they are treated as
  **parameter-only**. `PlantState` is the single source of truth during integration.

This file focuses on shared plant data structures and helper math; coupled dynamics are
implemented in `PlantModels` (e.g., `PlantModels.CoupledMultirotorModel`).
"""
module Plant

using StaticArrays

using ..Types: Vec3, quat_normalize
using ..RigidBody: RigidBodyState, RigidBodyDeriv, rb_add, rb_scale_add
using ..Vehicles
using ..Propulsion
using ..Powertrain
using ..Faults: FaultState

export PlantState,
    PlantDeriv,
    PlantInput,
    PlantOutputs,
    init_plant_state,
    plant_add,
    plant_scale_add,
    plant_lincomb

############################
# Core plant state
############################

"""Continuous-time plant state.

Fields:
* `rb`           : rigid-body state
* `motors_y`     : motor actuator outputs (12 channels, normalized [0,1])
* `motors_ydot`  : motor actuator rates (used only for 2nd-order actuator models)
* `servos_y`     : servo actuator outputs (8 channels, normalized [-1,1])
* `servos_ydot`  : servo actuator rates
* `rotor_ω`      : per-rotor angular speeds (rad/s)
* `batt_soc`     : battery state of charge [0,1]
* `batt_v1`      : Thevenin polarization voltage state (V), 0 for ideal batteries

Notes:
* Motors/servos are fixed-size to match PX4 lockstep ABI sizes.
* Rotor count is a type parameter `N` so the integrator can stay allocation-free.
"""
struct PlantState{N}
    rb::RigidBodyState
    motors_y::SVector{12,Float64}
    motors_ydot::SVector{12,Float64}
    servos_y::SVector{8,Float64}
    servos_ydot::SVector{8,Float64}
    rotor_ω::SVector{N,Float64}
    batt_soc::Float64
    batt_v1::Float64
end

function PlantState{N}(;
    rb::RigidBodyState = RigidBodyState(),
    motors_y::SVector{12,Float64} = zero(SVector{12,Float64}),
    motors_ydot::SVector{12,Float64} = zero(SVector{12,Float64}),
    servos_y::SVector{8,Float64} = zero(SVector{8,Float64}),
    servos_ydot::SVector{8,Float64} = zero(SVector{8,Float64}),
    rotor_ω::SVector{N,Float64} = zero(SVector{N,Float64}),
    batt_soc::Float64 = 1.0,
    batt_v1::Float64 = 0.0,
) where {N}
    return PlantState{N}(
        rb,
        motors_y,
        motors_ydot,
        servos_y,
        servos_ydot,
        rotor_ω,
        batt_soc,
        batt_v1,
    )
end

"""Continuous-time plant derivative."""
struct PlantDeriv{N}
    rb::RigidBodyDeriv
    motors_y_dot::SVector{12,Float64}
    motors_ydot_dot::SVector{12,Float64}
    servos_y_dot::SVector{8,Float64}
    servos_ydot_dot::SVector{8,Float64}
    rotor_ω_dot::SVector{N,Float64}
    batt_soc_dot::Float64
    batt_v1_dot::Float64
end

function PlantDeriv{N}(;
    rb::RigidBodyDeriv = RigidBodyDeriv(),
    motors_y_dot::SVector{12,Float64} = zero(SVector{12,Float64}),
    motors_ydot_dot::SVector{12,Float64} = zero(SVector{12,Float64}),
    servos_y_dot::SVector{8,Float64} = zero(SVector{8,Float64}),
    servos_ydot_dot::SVector{8,Float64} = zero(SVector{8,Float64}),
    rotor_ω_dot::SVector{N,Float64} = zero(SVector{N,Float64}),
    batt_soc_dot::Float64 = 0.0,
    batt_v1_dot::Float64 = 0.0,
) where {N}
    return PlantDeriv{N}(
        rb,
        motors_y_dot,
        motors_ydot_dot,
        servos_y_dot,
        servos_ydot_dot,
        rotor_ω_dot,
        batt_soc_dot,
        batt_v1_dot,
    )
end

############################
# Inputs and algebraic outputs
############################

"""Plant input held constant between discrete event boundaries.

This type is intentionally minimal and should be expanded as coupled dynamics are implemented.

Determinism contract:
* Construct/update `PlantInput` only at *discrete* event times (autopilot tick, wind tick,
  failure injection).
* Never call RNG or mutate shared state inside the ODE RHS evaluation.
"""
Base.@kwdef struct PlantInput
    cmd::Vehicles.ActuatorCommand = Vehicles.ActuatorCommand()
    wind_ned::Vec3 = zero(Vec3)
    faults::FaultState = FaultState()
end

"""Algebraic outputs of the plant (coupling + logging).

Includes rotor outputs (thrust/torque/current), bus voltage/current, and battery-status
data. Fields may be `nothing` if not populated by the caller.
"""
Base.@kwdef struct PlantOutputs{N}
    rotors::Union{Nothing,Propulsion.RotorOutput{N}} = nothing

    # Electrical bus (positive current = discharge).
    bus_current_a::Float64 = 0.0
    bus_voltage_v::Float64 = NaN

    # Atmosphere + local relative flow at the vehicle.
    rho_kgm3::Float64 = NaN
    temp_k::Float64 = NaN
    air_vel_body::Vec3 = Vec3(NaN, NaN, NaN)

    battery_status::Union{Nothing,Powertrain.BatteryStatus} = nothing
end

############################
# State math (used by integrators)
############################

@inline function plant_add(x::PlantState{N}, k::PlantDeriv{N}, h::Float64) where {N}
    return PlantState{N}(
        rb = rb_add(x.rb, k.rb, h),
        motors_y = x.motors_y + k.motors_y_dot * h,
        motors_ydot = x.motors_ydot + k.motors_ydot_dot * h,
        servos_y = x.servos_y + k.servos_y_dot * h,
        servos_ydot = x.servos_ydot + k.servos_ydot_dot * h,
        rotor_ω = x.rotor_ω + k.rotor_ω_dot * h,
        batt_soc = x.batt_soc + k.batt_soc_dot * h,
        batt_v1 = x.batt_v1 + k.batt_v1_dot * h,
    )
end

@inline function plant_scale_add(
    x::PlantState{N},
    k1::PlantDeriv{N},
    k2::PlantDeriv{N},
    k3::PlantDeriv{N},
    k4::PlantDeriv{N},
    h::Float64,
) where {N}
    # Mirrors `RigidBody.rb_scale_add` (classic RK4 combination).
    w = h / 6.0
    return PlantState{N}(
        rb = rb_scale_add(x.rb, k1.rb, k2.rb, k3.rb, k4.rb, h),
        motors_y = x.motors_y +
                   (
            k1.motors_y_dot + 2k2.motors_y_dot + 2k3.motors_y_dot + k4.motors_y_dot
        ) * w,
        motors_ydot = x.motors_ydot +
                      (
            k1.motors_ydot_dot +
            2k2.motors_ydot_dot +
            2k3.motors_ydot_dot +
            k4.motors_ydot_dot
        ) * w,
        servos_y = x.servos_y +
                   (
            k1.servos_y_dot + 2k2.servos_y_dot + 2k3.servos_y_dot + k4.servos_y_dot
        ) * w,
        servos_ydot = x.servos_ydot +
                      (
            k1.servos_ydot_dot +
            2k2.servos_ydot_dot +
            2k3.servos_ydot_dot +
            k4.servos_ydot_dot
        ) * w,
        rotor_ω = x.rotor_ω +
                  (k1.rotor_ω_dot + 2k2.rotor_ω_dot + 2k3.rotor_ω_dot + k4.rotor_ω_dot) * w,
        batt_soc = x.batt_soc +
                   (
            k1.batt_soc_dot + 2k2.batt_soc_dot + 2k3.batt_soc_dot + k4.batt_soc_dot
        ) * w,
        batt_v1 = x.batt_v1 +
                  (k1.batt_v1_dot + 2k2.batt_v1_dot + 2k3.batt_v1_dot + k4.batt_v1_dot) * w,
    )
end

@inline function plant_lincomb(
    x::PlantState{N},
    h::Float64,
    ks::NTuple{K,PlantDeriv{N}},
    as::NTuple{K,Float64},
) where {N,K}
    # Generic linear combination used by adaptive RK methods.
    # Note: keep quaternion normalization to a single normalize at the end (matches `_rb_lincomb`).

    # Rigid-body state fields.
    pos = x.rb.pos_ned
    vel = x.rb.vel_ned
    q = x.rb.q_bn
    ω = x.rb.ω_body

    # Other continuous states.
    motors_y = x.motors_y
    motors_ydot = x.motors_ydot
    servos_y = x.servos_y
    servos_ydot = x.servos_ydot
    rotor_ω = x.rotor_ω
    batt_soc = x.batt_soc
    batt_v1 = x.batt_v1

    @inbounds for i = 1:K
        w = as[i] * h
        pos = pos + ks[i].rb.pos_dot * w
        vel = vel + ks[i].rb.vel_dot * w
        q = q + ks[i].rb.q_dot * w
        ω = ω + ks[i].rb.ω_dot * w

        motors_y = motors_y + ks[i].motors_y_dot * w
        motors_ydot = motors_ydot + ks[i].motors_ydot_dot * w
        servos_y = servos_y + ks[i].servos_y_dot * w
        servos_ydot = servos_ydot + ks[i].servos_ydot_dot * w
        rotor_ω = rotor_ω + ks[i].rotor_ω_dot * w
        batt_soc = batt_soc + ks[i].batt_soc_dot * w
        batt_v1 = batt_v1 + ks[i].batt_v1_dot * w
    end

    rb = RigidBodyState(pos_ned = pos, vel_ned = vel, q_bn = quat_normalize(q), ω_body = ω)

    return PlantState{N}(
        rb = rb,
        motors_y = motors_y,
        motors_ydot = motors_ydot,
        servos_y = servos_y,
        servos_ydot = servos_ydot,
        rotor_ω = rotor_ω,
        batt_soc = batt_soc,
        batt_v1 = batt_v1,
    )
end

############################
# Initialization helpers
############################

# Actuator state extraction for initialization.
@inline _act_state(::Vehicles.DirectActuators, ::Val{N}) where {N} =
    (zero(SVector{N,Float64}), zero(SVector{N,Float64}))
@inline _act_state(a::Vehicles.FirstOrderActuators{N}, ::Val{N}) where {N} =
    (a.y, zero(SVector{N,Float64}))
@inline _act_state(a::Vehicles.SecondOrderActuators{N}, ::Val{N}) where {N} = (a.y, a.ydot)

# Battery initial state extraction.
#
# Battery model objects are parameter-only; state lives in PlantState.
@inline function _battery_state(b::Powertrain.IdealBattery)
    return (b.soc0, 0.0)
end
@inline function _battery_state(b::Powertrain.TheveninBattery)
    return (b.soc0, b.v1_0)
end

"""Initialize a PlantState from initial actuator + propulsion + battery parameters.

Canonical semantics
-------------------
PlantState is the only source of truth during integration. Subsystem model objects
(actuators/propulsion/battery) are treated as parameter-only inputs to the plant RHS
and are not synchronized from the integrated state.

Notes
-----
- Actuator y/y_dot initial conditions are taken from the actuator objects (if present).
- Rotor angular rates start at 0 rad/s.
"""
function init_plant_state(
    rb::RigidBodyState,
    motor_actuators,
    servo_actuators,
    propulsion::Propulsion.QuadRotorSet{N},
    battery::Powertrain.AbstractBatteryModel,
) where {N}
    my, mydot = _act_state(motor_actuators, Val(12))
    sy, sydot = _act_state(servo_actuators, Val(8))
    soc, v1 = _battery_state(battery)

    # Rotor angular rates are owned by the integrated plant state.
    ω = zero(SVector{N,Float64})

    return PlantState{N}(
        rb = rb,
        motors_y = SVector{12,Float64}(my),
        motors_ydot = SVector{12,Float64}(mydot),
        servos_y = SVector{8,Float64}(sy),
        servos_ydot = SVector{8,Float64}(sydot),
        rotor_ω = ω,
        batt_soc = soc,
        batt_v1 = v1,
    )
end

end # module Plant
