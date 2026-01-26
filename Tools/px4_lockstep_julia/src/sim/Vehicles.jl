"""PX4Lockstep.Sim.Vehicles

Aircraft/vehicle models.

The long-term scope includes:

* 6DOF rigid body
* multiple aero model options (identified coefficients vs CFD tables)
* prop/motor/ESC/battery models
* actuator dynamics for surfaces (2nd order, rate-limited)

This module starts with a high-signal baseline:

* a rigid-body multirotor model (suitable for closed-loop testing)
* clean interfaces so it can be replaced with higher-fidelity models later.
"""
module Vehicles

using ..Types: Vec3, Quat, Mat3, vec3, quat_to_dcm
using ..RigidBody: RigidBodyState, RigidBodyDeriv, quat_deriv
using ..Environment: EnvironmentModel, gravity_accel
using ..Propulsion: RotorOutput
using LinearAlgebra
using StaticArrays

export AbstractVehicleModel,
    MotorMap,
    ServoMap,
    PropulsorLayout,
    AbstractActuatorModel,
    DirectActuators,
    FirstOrderActuators,
    SecondOrderActuators,
    ActuatorCommand,
    VehicleInstance,
    QuadrotorParams,
    GenericMultirotor,
    map_motors,
    map_servos,
    step_actuators!,
    mass,
    inertia,
    inertia_diag,
    dynamics

"""Propulsor layout geometry.

This is the geometric definition of how propulsors are mounted on the vehicle.

Fields
------
* `pos_b[i]`  : propulsor position in body frame (meters)
* `axis_b[i]` : unit propulsor axis in body frame
* `spin_dir[i]`: reaction torque sign (+1/-1); included for completeness

Conventions
-----------
Body axes are **FRD**: X forward, Y right, Z down.

`axis_b[i]` is defined such that the thrust force applied to the vehicle is:

```
F_i = -T_i * axis_b[i]
```

For a classic multirotor with thrust along **-body Z**, `axis_b = (0,0,1)`.
"""
struct PropulsorLayout{N}
    pos_b::SVector{N,Vec3}
    axis_b::SVector{N,Vec3}
    spin_dir::SVector{N,Float64}
end

"""Actuator command packet.

Fixed-size arrays match the PX4 lockstep ABI (`actuator_motors` and `actuator_servos`) to
support multiple airframes without changing the sim engine.

* `motors` are typically normalized ESC duty commands.
* `servos` are typically normalized [-1,1] deflections.
"""
Base.@kwdef struct ActuatorCommand
    motors::SVector{12,Float64} = zero(SVector{12,Float64})
    servos::SVector{8,Float64} = zero(SVector{8,Float64})
end

############################
# Actuator channel mapping
############################

"""Mapping from PX4 motor output channels (1..12) to physical propulsors.

The plant state stores actuator outputs in fixed-size ABI arrays (`motors_y::SVector{12}`),
but the physical vehicle may have:

- fewer propulsors than channels (e.g., 4 motors)
- non-trivial wiring (e.g., motor #1 driven by channel 5)

`MotorMap{N}` defines how to read the duty signal for each of the `N` physical propulsors.

Important
---------
`FaultState.motor_disable_mask` is defined over **physical propulsors** (1..N), not over
PX4 output channels. The mapping only affects which actuator channel drives each propulsor.
"""
struct MotorMap{N}
    motor_channel::SVector{N,Int}
end

"""Mapping from PX4 servo output channels (1..8) to physical control surfaces."""
struct ServoMap{M}
    servo_channel::SVector{M,Int}
end

"""Map ABI motor array -> physical motor duties (size N)."""
@inline function map_motors(mm::MotorMap{N}, motors::SVector{12,Float64}) where {N}
    return SVector{N,Float64}(ntuple(i -> motors[mm.motor_channel[i]], N))
end

"""Map ABI servo array -> physical servo commands (size M)."""
@inline function map_servos(sm::ServoMap{M}, servos::SVector{8,Float64}) where {M}
    return SVector{M,Float64}(ntuple(i -> servos[sm.servo_channel[i]], M))
end

@inline function _finite_or(x::Float64, fallback::Float64)
    return isfinite(x) ? x : fallback
end

"""Validate that an `ActuatorCommand` is finite and within expected ABI ranges.

Ranges
------
- `motors[i]` must be in `[0,1]`
- `servos[i]` must be in `[-1,1]`

Notes
-----
- NaNs are commonly used by PX4 to indicate unused actuator channels. Set
  `allow_nan=true` to accept those values.

This is intended for *engine boundary* validation.
"""
function validate(cmd::ActuatorCommand; atol::Float64 = 1e-6, allow_nan::Bool = true)
    m = cmd.motors
    s = cmd.servos
    @inbounds for i = 1:12
        v = m[i]
        if isnan(v)
            allow_nan || error("ActuatorCommand.motors[$i] is NaN")
            continue
        end
        isfinite(v) || error("ActuatorCommand.motors[$i] is not finite: $v")
        (-atol <= v <= 1.0 + atol) ||
            error("ActuatorCommand.motors[$i] out of range [0,1]: $v")
    end
    @inbounds for i = 1:8
        v = s[i]
        if isnan(v)
            allow_nan || error("ActuatorCommand.servos[$i] is NaN")
            continue
        end
        isfinite(v) || error("ActuatorCommand.servos[$i] is not finite: $v")
        (-1.0 - atol <= v <= 1.0 + atol) ||
            error("ActuatorCommand.servos[$i] out of range [-1,1]: $v")
    end
    return nothing
end

"""Deterministically sanitize an `ActuatorCommand`.

Behavior
--------
- Non-finite values are replaced with 0.0.
- Values are clamped to ABI ranges: motors `[0,1]`, servos `[-1,1]`.
- If `strict=true`, this will `error(...)` on any non-finite value (except NaNs
  when `allow_nan=true`) or value outside the allowed range (within a small
  tolerance) *before* clamping.
"""
function sanitize(
    cmd::ActuatorCommand;
    strict::Bool = false,
    atol::Float64 = 1e-6,
    allow_nan::Bool = true,
)
    strict && validate(cmd; atol = atol, allow_nan = allow_nan)

    motors = SVector{12,Float64}(
        ntuple(i -> clamp(_finite_or(cmd.motors[i], 0.0), 0.0, 1.0), 12),
    )
    servos =
        SVector{8,Float64}(ntuple(i -> clamp(_finite_or(cmd.servos[i], 0.0), -1.0, 1.0), 8))
    return ActuatorCommand(motors = motors, servos = servos)
end

############################
# Vehicle model interface
############################

abstract type AbstractVehicleModel end

"""Vehicle mass (kg).

Expose this so the simulation engine can apply external forces (e.g. contact) without
hard-coding vehicle types.
"""
function mass end

"""Vehicle inertia diagonal (kg*m^2) in body frame."""
function inertia_diag end

"""Vehicle inertia tensor (kg*m^2) in body frame."""
function inertia end

"""Compute the rigid-body dynamics.

`dynamics(model, env, t, x, u, wind_ned)` must return `RigidBodyDeriv`.

Design contract
---------------
The canonical simulation engine treats **wind as a first-class input** sampled into the
runtime bus (sample-and-hold). Vehicle dynamics must therefore use `wind_ned` provided by
the plant input rather than reading `env.wind` (which may be disabled during replay).

`u` is vehicle-specific (e.g. rotor thrust/torque for multirotors).
"""
function dynamics end

"""Vehicle instance = model + propulsion/actuator instances + rigid-body state.

This type is engine-agnostic and exists to avoid coupling "vehicle assembly" to
any particular run loop implementation.

Historically this lived in the legacy fixed-step `Sim.Simulation` module.
As part of engine unification, it is promoted here so examples and workflows
can construct a vehicle without depending on a deprecated engine.
"""
mutable struct VehicleInstance{M<:AbstractVehicleModel,AM,AS,P}
    model::M
    motor_actuators::AM
    servo_actuators::AS
    propulsion::P
    state::RigidBodyState
end

############################
# Actuators
############################

abstract type AbstractActuatorModel end

"""Direct (no dynamics) actuator model.

`cmd_out = cmd_in`.
"""
struct DirectActuators <: AbstractActuatorModel end

@inline function step_actuators!(::DirectActuators, cmd, dt::Float64)
    return cmd
end

"""First-order actuator dynamics: `ẏ = (u - y)/τ`.

Provides a simple motor/servo lag model when some dynamics are required without a large
modeling effort.
"""
mutable struct FirstOrderActuators{N} <: AbstractActuatorModel
    τ::Float64
    y::SVector{N,Float64}
    last_dt::Float64
    last_tau::Float64
    alpha::Float64
end

function FirstOrderActuators{N}(;
    τ::Float64 = 0.05,
    y0 = zero(SVector{N,Float64}),
) where {N}
    return FirstOrderActuators{N}(τ, SVector{N,Float64}(y0), NaN, NaN, 1.0)
end

@inline function _ensure_coeffs!(a::FirstOrderActuators, dt::Float64)
    if dt != a.last_dt || a.τ != a.last_tau
        if !(isfinite(a.τ) && a.τ > 0.0) || dt <= 0.0
            a.alpha = 1.0
        else
            a.alpha = 1.0 - exp(-dt / a.τ)
        end
        a.last_dt = dt
        a.last_tau = a.τ
    end
end

@inline function step_actuators!(
    a::FirstOrderActuators{N},
    cmd::SVector{N,Float64},
    dt::Float64,
) where {N}
    # Exact discretization of ẏ = (u - y)/τ assuming ZOH on u across dt:
    #   y[k+1] = exp(-dt/τ) y[k] + (1-exp(-dt/τ)) u[k]
    # This keeps the effective time constant correct even if dt changes.
    _ensure_coeffs!(a, dt)
    a.y = (1.0 - a.alpha) * a.y + a.alpha * cmd
    return a.y
end

"""Second-order actuator dynamics with optional rate limiting.

Suitable for control surfaces (elevons, rudders, etc.) and any actuator that benefits
from more realism than a first-order lag.

Continuous-time model (per axis):
  ÿ + 2ζωₙ ẏ + ωₙ² y = ωₙ² u

Discrete update uses a simple semi-implicit Euler step which is stable enough for the
small dt typically used in flight dynamics sims.
"""
mutable struct SecondOrderActuators{N} <: AbstractActuatorModel
    ωn::Float64
    ζ::Float64
    rate_limit::Float64
    y::SVector{N,Float64}
    ydot::SVector{N,Float64}
end

function SecondOrderActuators{N}(;
    ωn::Float64 = 30.0,
    ζ::Float64 = 0.7,
    rate_limit::Float64 = Inf,
    y0 = zero(SVector{N,Float64}),
    ydot0 = zero(SVector{N,Float64}),
) where {N}
    return SecondOrderActuators{N}(
        ωn,
        ζ,
        rate_limit,
        SVector{N,Float64}(y0),
        SVector{N,Float64}(ydot0),
    )
end

@inline function step_actuators!(
    a::SecondOrderActuators{N},
    cmd::SVector{N,Float64},
    dt::Float64,
) where {N}
    ωn = a.ωn
    ζ = a.ζ
    # Acceleration
    ydd = (ωn*ωn) .* (cmd .- a.y) .- (2.0*ζ*ωn) .* a.ydot
    # Semi-implicit integration
    a.ydot = a.ydot .+ ydd .* dt
    if isfinite(a.rate_limit)
        rl = a.rate_limit
        a.ydot = map(v -> clamp(v, -rl, rl), a.ydot)
    end
    a.y = a.y .+ a.ydot .* dt
    return a.y
end

############################
# Multirotor baseline model
############################

"""Quadrotor physical parameters.

Conventions:
* Rotor `i` produces thrust magnitude `T_i >= 0`.
* The thrust force applied to the vehicle is `F_i = -T_i * rotor_axis_body[i]`.
  - Classic multirotor: `rotor_axis_body[i] = (0,0,1)` so thrust is along **-body Z**.
* Rotor positions are in body coordinates, meters.

Notes:
* Yaw reaction torque sign is owned by the *propulsion* model (e.g. `Propulsion.QuadRotorSet`).
  The rigid-body model consumes signed per-rotor shaft torque from propulsion.
"""
Base.@kwdef struct QuadrotorParams{N}
    mass::Float64 = 1.5
    inertia_kgm2::Mat3 = @SMatrix [
        0.029125 0.0 0.0
        0.0 0.029125 0.0
        0.0 0.0 0.055225
    ]
    inertia_inv_kgm2::Mat3 = @SMatrix [
        1.0 / 0.029125 0.0 0.0
        0.0 1.0 / 0.029125 0.0
        0.0 0.0 1.0 / 0.055225
    ]
    rotor_pos_body::SVector{N,Vec3}
    rotor_axis_body::SVector{N,Vec3}
    rotor_inertia_kgm2::SVector{N,Float64}
    rotor_dir::SVector{N,Float64}
    linear_drag::Float64 = 0.05             # N/(m/s) per axis (simple model)
    angular_damping::Vec3 = vec3(0.02, 0.02, 0.01)
end

"""A minimal N-propulsor multirotor rigid-body model.

This is intentionally low-fidelity (rigid-body + simple drag + thrust moments) but it is:
- stable and high-signal for closed-loop PX4 testing
- allocation-free in the hot path

The baseline multirotor model supports arbitrary propulsor counts `N`.

Notes
-----
* Thrust direction is configurable per propulsor via `rotor_axis_body`.
"""
struct GenericMultirotor{N} <: AbstractVehicleModel
    params::QuadrotorParams{N}
end

"""Vehicle mass (kg)."""
mass(m::GenericMultirotor) = m.params.mass

"""Vehicle inertia diagonal (kg*m^2) in body axes."""
inertia_diag(m::GenericMultirotor) = begin
    I = m.params.inertia_kgm2
    vec3(I[1, 1], I[2, 2], I[3, 3])
end

"""Vehicle inertia tensor (kg*m^2) in body axes."""
inertia(m::GenericMultirotor) = m.params.inertia_kgm2

@inline function _multirotor_dynamics(
    p::QuadrotorParams{N},
    env::EnvironmentModel,
    t::Float64,
    x::RigidBodyState,
    u::RotorOutput{N},
    wind_ned::Vec3,
) where {N}
    m = p.mass
    I = p.inertia_kgm2
    I_inv = p.inertia_inv_kgm2

    thrusts = u.thrust_n
    shaft_torques = u.shaft_torque_nm

    # Total force in body (sum of per-propulsor thrust vectors).
    # Convention: F_i = -T_i * axis_b[i].
    F_body = vec3(0.0, 0.0, 0.0)
    @inbounds for i = 1:N
        F_body += -thrusts[i] .* p.rotor_axis_body[i]
    end
    R_bn = quat_to_dcm(x.q_bn)
    F_ned = R_bn * F_body

    # Simple linear drag based on air-relative velocity.
    # Important: wind must come from the **bus** (sample-and-hold) so record/replay uses
    # the exact same forcing. Do not read `env.wind` here.
    v_rel = x.vel_ned - wind_ned
    F_drag = -p.linear_drag .* v_rel

    # Gravity in NED.
    g_ned = gravity_accel(env.gravity, x.pos_ned, t)

    # Translational dynamics.
    vel_dot = (F_ned + F_drag) / m + g_ned

    # Moments from propulsors (lever arm cross thrust + reaction torque about propulsor axis).
    τ = vec3(0.0, 0.0, 0.0)
    @inbounds for i = 1:N
        r = p.rotor_pos_body[i]
        Ti = thrusts[i]
        axis_b = p.rotor_axis_body[i]
        # Force vector for each rotor in body.
        Fi = -Ti .* axis_b
        τ += cross(r, Fi)
        # Signed reaction torque comes from propulsion output.
        # Direction is along the propulsor axis.
        τ += axis_b .* shaft_torques[i]
    end

    # Angular dynamics.
    ω = x.ω_body
    Iω = I * ω
    ω_cross_Iω = cross(ω, Iω)

    # Rotor gyroscopic coupling.
    #
    # The propulsion layer owns the per-rotor axial inertia `J` and provides rotor speeds
    # and accelerations. The rigid body accounts for the additional angular momentum
    # stored in the spinning rotors:
    #
    #   L_total = I_body*ω + Σ (J_i * Ω_i)
    #
    # which yields extra terms in body-frame dynamics:
    #
    #   I_body*ω̇ = τ - ω×(I_body*ω) - ω×H_rotor - Ḣ_rotor
    #
    # where H_rotor = Σ(J_i*Ω_i).
    #
    # Note: `p.rotor_dir` is the *reaction torque sign* (torque on body). Rotor spin
    # direction is opposite that sign.
    H = vec3(0.0, 0.0, 0.0)
    Hdot = vec3(0.0, 0.0, 0.0)
    @inbounds for i = 1:N
        J = p.rotor_inertia_kgm2[i]
        # Rotor spin sign: opposite the reaction torque sign.
        s_spin = -p.rotor_dir[i]
        axis_b = p.rotor_axis_body[i]
        H += (J * s_spin * u.ω_rad_s[i]) .* axis_b
        Hdot += (J * s_spin * u.ω_dot_rad_s2[i]) .* axis_b
    end

    τ_damped = τ - p.angular_damping .* ω
    rhs = τ_damped - ω_cross_Iω - cross(ω, H) - Hdot
    ω_dot = I_inv * rhs

    return RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = vel_dot,
        q_dot = quat_deriv(x.q_bn, ω),
        ω_dot = ω_dot,
    )
end

"""Rigid-body dynamics for the generic multirotor."""
function dynamics(
    model::GenericMultirotor{N},
    env::EnvironmentModel,
    t::Float64,
    x::RigidBodyState,
    u::RotorOutput{N},
    wind_ned::Vec3,
) where {N}
    return _multirotor_dynamics(model.params, env, t, x, u, wind_ned)
end


end # module Vehicles
