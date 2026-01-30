"""PX4Lockstep.Sim.Contacts

Pluggable contact/terrain models.

The long-term roadmap includes terrain and touchdown modeling. This module provides a
"minimum viable" contact layer that is:

* deterministic
* stable with fixed-step RK integration
* optional (NoContact by default)

The interface remains intentionally minimal: `contact_force_ned(model, x, t)` returns an
external force in the world/NED frame that is added to the rigid-body translational
dynamics.

As ground-contact handling matures, this module also defines a small set of *contact
observables* (`ContactInfo`) that can be surfaced through `PlantOutputs` for logging and
for downstream systems (e.g., landed detection, touchdown cues).
"""
module Contacts

using ..Types: Vec3, vec3
using ..RigidBody: RigidBodyState

export AbstractContactModel,
    NoContact,
    FlatGroundContact,
    FlatGroundConstraintContact,
    ContactMode,
    ContactInfo,
    ground_gap_m,
    ground_normal_ned,
    contact_force_ned,
    contact_info

abstract type AbstractContactModel end

# Flat ground primitives (z=0 plane in NED, down is +).
@inline ground_gap_m(x::RigidBodyState)::Float64 = -x.pos_ned[3] # + = separated (above ground)
@inline ground_normal_ned()::Vec3 = vec3(0.0, 0.0, -1.0)         # outward normal (up) in NED

@enum ContactMode begin
    CONTACT_AIRBORNE
    CONTACT_IMPACTING
    CONTACT_GROUNDED
end

"""Lightweight contact observables for logging and downstream logic.

Conventions:
* World frame is NED, ground plane is z=0 (down-positive).
* `gap_m` is positive when separated (vehicle above the plane), 0 at contact, negative in penetration.
* `penetration_m` is `max(0, -gap_m)`.

Notes:
* `ContactInfo` is an instantaneous snapshot derived from the current state (and
  optionally the unconstrained acceleration). It does **not** currently record
  interval-localized impact events (TOI/impulse), which occur *inside* an integration
  interval.
* Impact telemetry is instead surfaced via interval metadata and latched on the
  runtime `SimBus` (`impact_dv_ned`, `impact_time_us`, `impact_count`, ...).
"""
Base.@kwdef struct ContactInfo
    active::Bool = false
    mode::ContactMode = CONTACT_AIRBORNE

    gap_m::Float64 = NaN
    penetration_m::Float64 = 0.0
    normal_ned::Vec3 = ground_normal_ned()

    # Instantaneous contact forces (penalty model) in NED.
    force_ned::Vec3 = vec3(0.0, 0.0, 0.0)
    normal_force_n::Float64 = 0.0
    friction_force_ned::Vec3 = vec3(0.0, 0.0, 0.0)

    # Hybrid-impact bookkeeping (Phase 3+).
    v_n_pre_mps::Float64 = NaN
    v_n_post_mps::Float64 = NaN
    normal_impulse_ns::Float64 = 0.0
    tangential_impulse_ned_ns::Vec3 = vec3(0.0, 0.0, 0.0)
    impact_dv_ned::Vec3 = vec3(0.0, 0.0, 0.0)
    t_last_impact_us::UInt64 = UInt64(0)
end

"""No contact model."""
struct NoContact <: AbstractContactModel end

contact_force_ned(::NoContact, ::RigidBodyState, ::Float64) = vec3(0.0, 0.0, 0.0)

"""Fallback contact observables for unknown contact models."""
function contact_info(::AbstractContactModel, x::RigidBodyState, ::Float64)
    gap = ground_gap_m(x)
    return ContactInfo(
        active = false,
        mode = CONTACT_AIRBORNE,
        gap_m = gap,
        penetration_m = max(0.0, -gap),
        normal_ned = ground_normal_ned(),
    )
end

function contact_info(::NoContact, x::RigidBodyState, t::Float64)
    gap = ground_gap_m(x)
    return ContactInfo(
        active = false,
        mode = CONTACT_AIRBORNE,
        gap_m = gap,
        penetration_m = max(0.0, -gap),
        normal_ned = ground_normal_ned(),
    )
end

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

"""Flat ground plane *constraint* contact at z=0 (NED), applied at the vehicle COM.

This is a **non-stiff** alternative to the penalty spring-damper contact.

Normal reaction
---------------
Rather than introducing a stiff spring, the normal force is chosen to cancel the
unconstrained *downward* acceleration when the vehicle is at/near the ground.

Let `a_free_ned` be the unconstrained translational acceleration in NED (including
gravity, thrust, etc.) and `a_free_z = a_free_ned[3]` (down-positive). When the
vehicle is at/near the ground and not moving upward, we apply:

* `N = m * max(a_free_z, 0)`
* `Fz = -N` (upward in NED)

So the resulting vertical acceleration becomes ~0, keeping the vehicle resting on
the ground without forcing tiny integration steps.

Friction
--------
Friction uses the same smooth Coulomb approximation as `FlatGroundContact`, but
with the normal force `N` computed from the constraint reaction.

Notes
-----
* This model is intended to make **resting contact** stable for adaptive explicit
  integrators.
* Touchdown timing/impact is still handled by projection or hybrid events (Phase 3+).
"""
Base.@kwdef mutable struct FlatGroundConstraintContact <: AbstractContactModel
    μ::Float64 = 0.8
    v_eps::Float64 = 0.05
    enable_friction::Bool = true

    """Coefficient of restitution for the *impact map* (Phase 3+).

`e = 0` is perfectly inelastic (no bounce), `e = 1` is perfectly elastic.

This does not affect the continuous grounded-mode constraint reaction; it is only
used when an impact event is localized and an instantaneous velocity reset is applied.
"""
    restitution::Float64 = 0.0

    """Capture threshold for "resting" impacts (m/s).

If the pre-impact closing speed along the normal is below this threshold, we force
an inelastic capture (`v_n_post = 0`) to avoid Zeno/chatter loops.
"""
    v_rest_threshold_mps::Float64 = 0.05

    """If true, apply a tangential Coulomb impulse at impact (Phase 3+).

This is a single-point COM contact approximation. Continuous friction in grounded
mode still applies independently.
"""
    enable_impact_friction::Bool = true

    """Grounded-mode contact substep (microseconds).

Phase 4 (Plan1) moves resting/sliding contact to a **time-stepping impulse solve**.
While the vehicle is grounded, the plant integrator runs a fixed-step contact loop
with step size `h_contact_us` (rather than letting an adaptive RK solver shrink to
resolve contact stiffness).

Smaller values improve contact fidelity and static-friction behavior (at higher cost).
Typical values: 500–2000 µs.
"""
    h_contact_us::UInt64 = UInt64(1000)

    """Vertical slop band above the plane treated as 'at ground' (m).

This prevents chatter when numerical drift leaves `z` at tiny negative values.
"""
    z_slop_m::Float64 = 1e-6
end

@inline function _flat_ground_constraint_eval(
    c::FlatGroundConstraintContact,
    x::RigidBodyState,
    m_kg::Float64,
    a_free_ned::Vec3,
)
    z = x.pos_ned[3] # down is +
    vz = x.vel_ned[3]
    a_free_z = a_free_ned[3]

    # Candidate if at/near the plane and not moving upward.
    candidate = (z >= -c.z_slop_m) && (vz >= 0.0)

    # Apply a unilateral normal reaction only if the unconstrained dynamics would
    # accelerate further into the ground.
    N = candidate && (a_free_z > 0.0) ? (m_kg * a_free_z) : 0.0

    Fx = 0.0
    Fy = 0.0
    if c.enable_friction && (N > 0.0)
        vx = x.vel_ned[1]
        vy = x.vel_ned[2]
        vxy = sqrt(vx * vx + vy * vy)
        scale = -c.μ * N / (vxy + c.v_eps)
        Fx = scale * vx
        Fy = scale * vy
    end

    # Upward normal force is negative in NED.
    F = vec3(Fx, Fy, -N)
    F_fric = vec3(Fx, Fy, 0.0)

    gap = ground_gap_m(x)
    penetration = max(0.0, -gap)

    return F, N, F_fric, gap, penetration, candidate
end

"""Constraint contact force given *unconstrained* acceleration.

This overload is intentionally separate from the 3-argument `contact_force_ned` used by
penalty models.
"""
function contact_force_ned(
    c::FlatGroundConstraintContact,
    x::RigidBodyState,
    t::Float64,
    m_kg::Float64,
    a_free_ned::Vec3,
)
    F, _N, _F_fric, _gap, _penetration, _candidate =
        _flat_ground_constraint_eval(c, x, m_kg, a_free_ned)
    return F
end

"""Constraint contact observables from *state only*.

This method does **not** know the unconstrained acceleration, so force fields are left
at zero/NaN. Engines should prefer the 5-argument overload when available.
"""
function contact_info(c::FlatGroundConstraintContact, x::RigidBodyState, t::Float64)
    z = x.pos_ned[3]
    vz = x.vel_ned[3]
    gap = ground_gap_m(x)
    penetration = max(0.0, -gap)

    active = (z >= -c.z_slop_m) && (vz >= 0.0)
    mode = if !active
        CONTACT_AIRBORNE
    elseif vz > 0.0
        CONTACT_IMPACTING
    else
        CONTACT_GROUNDED
    end

    return ContactInfo(
        active = active,
        mode = mode,
        gap_m = gap,
        penetration_m = penetration,
        normal_ned = ground_normal_ned(),
        force_ned = vec3(0.0, 0.0, 0.0),
        normal_force_n = NaN,
        friction_force_ned = vec3(0.0, 0.0, 0.0),
    )
end

"""Constraint contact observables given *unconstrained* acceleration."""
function contact_info(
    c::FlatGroundConstraintContact,
    x::RigidBodyState,
    t::Float64,
    m_kg::Float64,
    a_free_ned::Vec3,
)
    F, N, F_fric, gap, penetration, candidate =
        _flat_ground_constraint_eval(c, x, m_kg, a_free_ned)

    mode = if !candidate
        CONTACT_AIRBORNE
    elseif x.vel_ned[3] > 0.0
        CONTACT_IMPACTING
    else
        CONTACT_GROUNDED
    end

    return ContactInfo(
        active = candidate,
        mode = mode,
        gap_m = gap,
        penetration_m = penetration,
        normal_ned = ground_normal_ned(),
        force_ned = F,
        normal_force_n = N,
        friction_force_ned = F_fric,
    )
end

@inline function _flat_ground_eval(c::FlatGroundContact, x::RigidBodyState)
    z = x.pos_ned[3] # down is +
    if z <= 0.0
        return vec3(0.0, 0.0, 0.0), 0.0, vec3(0.0, 0.0, 0.0), 0.0
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
    F = vec3(Fx, Fy, -N)
    F_fric = vec3(Fx, Fy, 0.0)
    return F, N, F_fric, penetration
end

function contact_force_ned(c::FlatGroundContact, x::RigidBodyState, t::Float64)
    F, _N, _F_fric, _penetration = _flat_ground_eval(c, x)
    return F
end

function contact_info(c::FlatGroundContact, x::RigidBodyState, t::Float64)
    gap = ground_gap_m(x)
    F, N, F_fric, penetration = _flat_ground_eval(c, x)
    active = penetration > 0.0

    mode = if !active
        CONTACT_AIRBORNE
    elseif x.vel_ned[3] > 0.0
        CONTACT_IMPACTING
    else
        CONTACT_GROUNDED
    end

    return ContactInfo(
        active = active,
        mode = mode,
        gap_m = gap,
        penetration_m = penetration,
        normal_ned = ground_normal_ned(),
        force_ned = F,
        normal_force_n = N,
        friction_force_ned = F_fric,
    )
end

end # module Contacts
