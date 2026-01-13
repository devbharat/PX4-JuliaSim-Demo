"""PX4Lockstep.Sim.Vehicles

Aircraft/vehicle models.

The long-term plan (per your Gradient doc) includes:

* 6DOF rigid body
* multiple aero model options (identified coefficients vs CFD tables)
* prop/motor/ESC/battery models
* actuator dynamics for surfaces (2nd order, rate-limited)

This file starts with a high-signal baseline:

* a rigid-body quadrotor model for the PX4 Iris (good enough to close the loop)
* clean interfaces so it can be replaced with higher-fidelity models later.
"""
module Vehicles

using ..Types: Vec3, Quat, Mat3, vec3, quat_to_dcm
using ..RigidBody: RigidBodyState, RigidBodyDeriv, quat_deriv
using ..Environment: EnvironmentModel, wind_velocity, gravity_accel
using LinearAlgebra
using StaticArrays

export AbstractVehicleModel,
       AbstractActuatorModel,
       DirectActuators, FirstOrderActuators,
       ActuatorCommand,
       QuadrotorParams, IrisQuadrotor,
       step_actuators!,
       dynamics

"""Actuator command packet.

We keep fixed-size arrays matching the PX4 lockstep ABI (`actuator_motors` and
`actuator_servos`) to make it easy to support multiple airframes without changing the sim
engine.

* `motors` are typically normalized thrust commands.
* `servos` are typically normalized [-1,1] deflections.
"""
Base.@kwdef struct ActuatorCommand
    motors::SVector{12,Float64} = zero(SVector{12,Float64})
    servos::SVector{8,Float64} = zero(SVector{8,Float64})
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

This is a good default for motors/servos when you want *some* dynamics without a big
modeling effort.
"""
mutable struct FirstOrderActuators{N} <: AbstractActuatorModel
    τ::Float64
    y::SVector{N, Float64}
end

function FirstOrderActuators{N}(; τ::Float64=0.05, y0=zero(SVector{N,Float64})) where {N}
    return FirstOrderActuators{N}(τ, SVector{N,Float64}(y0))
end

@inline function step_actuators!(a::FirstOrderActuators{N}, cmd::SVector{N,Float64}, dt::Float64) where {N}
    α = clamp(dt / a.τ, 0.0, 1.0)
    a.y = (1.0 - α) * a.y + α * cmd
    return a.y
end

"""Second-order actuator dynamics with optional rate limiting.

This is a good default for control surfaces (elevons, rudders, etc.) and any actuator
where you want a bit more realism than a first-order lag.

Continuous-time model (per axis):
  ÿ + 2ζωₙ ẏ + ωₙ² y = ωₙ² u

Discrete update uses a simple semi-implicit Euler step which is stable enough for the
small dt typically used in flight dynamics sims.
"""
mutable struct SecondOrderActuators{N} <: AbstractActuatorModel
    ωn::Float64
    ζ::Float64
    rate_limit::Float64
    y::SVector{N, Float64}
    ydot::SVector{N, Float64}
end

function SecondOrderActuators{N}(; ωn::Float64=30.0, ζ::Float64=0.7, rate_limit::Float64=Inf,
                                  y0=zero(SVector{N,Float64}), ydot0=zero(SVector{N,Float64})) where {N}
    return SecondOrderActuators{N}(ωn, ζ, rate_limit, SVector{N,Float64}(y0), SVector{N,Float64}(ydot0))
end

@inline function step_actuators!(a::SecondOrderActuators{N}, cmd::SVector{N,Float64}, dt::Float64) where {N}
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
# Vehicle model interface
############################

abstract type AbstractVehicleModel end

"""Compute the rigid-body dynamics.

`dynamics(t, state, u)` must return `RigidBodyDeriv`.

`u` is typically an actuator-level command vector (e.g. motor normalized thrust commands).
"""
function dynamics end

############################
# Quadrotor baseline model
############################

"""Quadrotor physical parameters.

Conventions:
* Rotor thrust acts along **-body Z** (upwards in NED).
* Rotor positions are in body coordinates, meters.
* `km` maps thrust to yaw moment (Nm per N). Sign determined by rotor direction.
"""
Base.@kwdef struct QuadrotorParams{N}
    mass::Float64 = 1.5
    inertia_diag::Vec3 = vec3(0.029125, 0.029125, 0.055225)
    rotor_pos_body::SVector{N, Vec3}
    rotor_dir::SVector{N, Float64}          # +1 or -1
    km::Float64 = 0.05                      # yaw moment coefficient
    hover_throttle::Float64 = 0.5           # PX4 iris default-ish
    thrust_exponent::Float64 = 2.0            # thrust ∝ cmd^exp
    linear_drag::Float64 = 0.05             # N/(m/s) per axis (simple model)
    angular_damping::Vec3 = vec3(0.02, 0.02, 0.01)
    max_total_thrust_n::Float64 = (mass * 9.80665) / (hover_throttle^thrust_exponent)
end

@inline function max_thrust_per_rotor(p::QuadrotorParams{N}) where {N}
    return p.max_total_thrust_n / N
end

"""A minimal Iris quadrotor dynamics model.

This is intentionally low-fidelity (rigid-body + simple drag + rotor thrust moments) but
it is stable and high-signal for closing the loop with PX4. It is meant to be replaced
by higher-fidelity aero/powertrain models without changing the sim engine.
"""
struct IrisQuadrotor <: AbstractVehicleModel
    params::QuadrotorParams{4}
end

function IrisQuadrotor(; params::Union{Nothing,QuadrotorParams{4}}=nothing)
    if params === nothing
        rotor_pos = SVector(
            vec3( 0.1515,  0.2450, 0.0),
            vec3(-0.1515, -0.1875, 0.0),
            vec3( 0.1515, -0.2450, 0.0),
            vec3(-0.1515,  0.1875, 0.0),
        )
        rotor_dir = SVector(1.0, 1.0, -1.0, -1.0)
        params = QuadrotorParams{4}(rotor_pos_body=rotor_pos, rotor_dir=rotor_dir)
    end
    return IrisQuadrotor(params)
end

"""Rigid-body dynamics for the quadrotor.

`u` is a vector of normalized motor commands. For non-reversible rotors we clamp to [0,1].
"""
function dynamics(model::IrisQuadrotor, env::EnvironmentModel,
                  t::Float64, x::RigidBodyState, u)
    p = model.params
    m = p.mass
    I = p.inertia_diag

    # Convert/shape input to 4 motor commands.
    cmd = if u isa ActuatorCommand
        SVector{4,Float64}(u.motors[1], u.motors[2], u.motors[3], u.motors[4])
    elseif u isa SVector{4,Float64}
        u
    elseif u isa NTuple{4,Float64}
        SVector{4,Float64}(u)
    elseif u isa AbstractVector
        SVector{4,Float64}(Float64.(u[1:4]))
    else
        error("Unsupported motor command type: $(typeof(u))")
    end

    # Motor -> thrust
    Tmax = max_thrust_per_rotor(p)
    thrusts = map(c -> (clamp(c, 0.0, 1.0)^p.thrust_exponent) * Tmax, cmd)

    # Total force in body (rotor thrust along -Z).
    F_body = vec3(0.0, 0.0, -sum(thrusts))
    R_bn = quat_to_dcm(x.q_bn)
    F_ned = R_bn * F_body

    # Simple linear drag based on air-relative velocity.
    w = wind_velocity(env.wind, x.pos_ned, t)
    v_rel = x.vel_ned - w
    F_drag = -p.linear_drag .* v_rel

    # Gravity in NED.
    g_ned = gravity_accel(env.gravity, x.pos_ned, t)

    # Translational dynamics.
    vel_dot = (F_ned + F_drag) / m + g_ned

    # Moments from rotors (lever arm cross thrust + yaw torque).
    τ = vec3(0.0, 0.0, 0.0)
    for i in 1:4
        r = p.rotor_pos_body[i]
        Ti = thrusts[i]
        # Force vector for each rotor in body.
        Fi = vec3(0.0, 0.0, -Ti)
        τ += cross(r, Fi)
        τ += vec3(0.0, 0.0, p.rotor_dir[i] * p.km * Ti)
    end

    # Angular dynamics with simple damping.
    ω = x.ω_body
    Iω = vec3(I[1]*ω[1], I[2]*ω[2], I[3]*ω[3])
    ω_cross_Iω = cross(ω, Iω)
    τ_damped = τ - p.angular_damping .* ω
    ω_dot = vec3(
        (τ_damped[1] - ω_cross_Iω[1]) / I[1],
        (τ_damped[2] - ω_cross_Iω[2]) / I[2],
        (τ_damped[3] - ω_cross_Iω[3]) / I[3],
    )

    return RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = vel_dot,
        q_dot   = quat_deriv(x.q_bn, ω),
        ω_dot   = ω_dot,
    )
end

end # module Vehicles
