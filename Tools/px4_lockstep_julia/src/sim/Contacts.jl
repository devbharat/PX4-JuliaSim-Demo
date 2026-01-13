"""PX4Lockstep.Sim.Contacts

Pluggable contact/terrain models.

The long-term roadmap includes terrain and touchdown modeling. This module provides a
"minimum viable" contact layer that is:

* deterministic
* stable with fixed-step RK integration
* optional (NoContact by default)

We keep the interface intentionally small: contact returns an external force in the
world/NED frame that the simulation adds to the rigid-body translational dynamics.

Future extensions:
* contact points (landing gear), producing both force and moment
* terrain height fields
* rolling friction and stick-slip
"""
module Contacts

using ..Types: Vec3, vec3
using ..RigidBody: RigidBodyState

export AbstractContactModel, NoContact, FlatGroundContact, contact_force_ned

abstract type AbstractContactModel end

"""No contact model."""
struct NoContact <: AbstractContactModel end

contact_force_ned(::NoContact, ::RigidBodyState, ::Float64) = vec3(0.0, 0.0, 0.0)

"""Flat ground plane contact at z=0 (NED), applied at the vehicle COM.

This is a compliant penalty model (spring-damper + friction):

* normal force:  N = k * penetration + c * v_z (only when penetrating)
* friction:      F_t = -μ N * v_t / (|v_t| + v_eps)

Because the force is applied at the COM, it does not generate moments.
"""
Base.@kwdef mutable struct FlatGroundContact <: AbstractContactModel
    k_n_per_m::Float64 = 5_000.0
    c_n_per_mps::Float64 = 600.0
    μ::Float64 = 0.8
    v_eps::Float64 = 0.05
    enable_friction::Bool = true
end

function contact_force_ned(c::FlatGroundContact, x::RigidBodyState, t::Float64)
    z = x.pos_ned[3] # down is +
    if z <= 0.0
        return vec3(0.0, 0.0, 0.0)
    end

    penetration = z
    vz = x.vel_ned[3]

    # Only resist compression/penetration (vz>0 is moving down into ground).
    N = c.k_n_per_m * penetration + (vz > 0.0 ? c.c_n_per_mps * vz : 0.0)
    N = max(0.0, N)

    Fx = 0.0
    Fy = 0.0
    if c.enable_friction
        vx = x.vel_ned[1]
        vy = x.vel_ned[2]
        vxy = sqrt(vx*vx + vy*vy)
        # Smooth Coulomb friction with small epsilon.
        scale = -c.μ * N / (vxy + c.v_eps)
        Fx = scale * vx
        Fy = scale * vy
    end

    # Upward normal force is negative in NED.
    Fz = -N
    return vec3(Fx, Fy, Fz)
end

end # module Contacts
