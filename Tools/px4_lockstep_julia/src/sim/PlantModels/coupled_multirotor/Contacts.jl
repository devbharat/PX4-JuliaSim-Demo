############################
# Contact + grounded helpers
############################

# Internal helper: update only the rigid-body substate.
@inline function _with_rb(x::PlantState{N,B}, rb::RigidBodyState) where {N,B}
    return PlantState{N,B}(
        rb = rb,
        motors_y = x.motors_y,
        motors_ydot = x.motors_ydot,
        servos_y = x.servos_y,
        servos_ydot = x.servos_ydot,
        rotor_ω = x.rotor_ω,
        power = x.power,
    )
end

# Internal helper: apply an instantaneous impact map at z=0 (COM contact).
@inline function _apply_ground_impact(
    c::FlatGroundConstraintContact,
    rb::RigidBodyState,
)::RigidBodyState
    pos = rb.pos_ned
    vel = rb.vel_ned

    # Project position to the plane (z=0).
    pos2 = SVector{3,Float64}(pos[1], pos[2], 0.0)

    vx = vel[1]
    vy = vel[2]
    vz = vel[3] # down-positive

    # Only apply an impact impulse if closing into the plane (vz > 0).
    if !(vz > 0.0)
        return RigidBodyState(
            pos_ned = pos2,
            vel_ned = vel,
            q_bn = rb.q_bn,
            ω_body = rb.ω_body,
        )
    end

    # Zeno capture rule: treat small impacts as resting contact.
    e = abs(vz) < c.v_rest_threshold_mps ? 0.0 : c.restitution
    vz_post = -e * vz

    vx_post = vx
    vy_post = vy
    if c.enable_friction && c.enable_impact_friction && (c.μ > 0.0)
        vxy = sqrt(vx * vx + vy * vy)
        if vxy > 0.0
            # Tangential impulse bound: |Δv_t| <= μ (1+e) v_n_pre.
            dv_t_max = c.μ * (1.0 + e) * vz
            if vxy <= dv_t_max
                vx_post = 0.0
                vy_post = 0.0
            else
                # Reduce tangential speed by dv_t_max, preserving direction.
                s = (vxy - dv_t_max) / vxy
                vx_post = vx * s
                vy_post = vy * s
            end
        end
    end

    vel2 = SVector{3,Float64}(vx_post, vy_post, vz_post)
    return RigidBodyState(
        pos_ned = pos2,
        vel_ned = vel2,
        q_bn = rb.q_bn,
        ω_body = rb.ω_body,
    )
end


# Internal helper: grounded candidate predicate (time-stepping mode).
@inline function _ground_candidate(c::FlatGroundConstraintContact, rb::RigidBodyState)::Bool
    # Treat tiny upward velocities as numerical noise to reduce mode chattering.
    return (rb.pos_ned[3] >= -c.z_slop_m) && (rb.vel_ned[3] >= -c.vz_slop_mps)
end


# Internal helper: safe step-size limiter near the ground guard.
#
# When airborne and descending toward the plane, cap the size of the next integration
# chunk to O(time-to-impact) based on the current distance-to-ground and closing speed.
#
# This reduces:
# - large-step tunneling risk (missing a guard crossing when z(t) is non-monotonic), and
# - adaptive-solver thrash from repeatedly proposing big steps near the guard.
#
# Note: we intentionally use a safety factor > 1 so the lookahead step can still cross
# the guard and bracket the TOI for bisection.
@inline function _airborne_guard_step_us(
    c::FlatGroundConstraintContact,
    rb::RigidBodyState,
    remaining_us::UInt64,
)::UInt64
    dt_us = remaining_us

    z = rb.pos_ned[3]  # down-positive
    vz = rb.vel_ned[3]

    # Only limit when above ground and closing toward the plane.
    if (z < 0.0) && (vz > 0.0)
        phi_m = -z

        # Tunables (kept local for now to avoid API churn).
        SAFETY = 1.2
        V_EPS = 1e-3

        t_est_s = phi_m / max(vz, V_EPS)
        h_max_s = SAFETY * t_est_s

        if isfinite(h_max_s) && (h_max_s > 0.0)
            h_max_us = UInt64(floor(h_max_s * 1e6))
            h_max_us = max(UInt64(1), h_max_us)
            if h_max_us < dt_us
                dt_us = h_max_us
            end
        else
            dt_us = UInt64(1)
        end
    end

    return dt_us
end


"""Advance one grounded contact substep using a 1-contact BLCP impulse solve.

This is the grounded-mode integrator:

* Integrate the *smooth* plant dynamics (no contact) over `dt_s` using a single
  explicit evaluation (Euler).
* Apply an impulse-based contact solve at the end of the substep to enforce
  non-penetration (unilateral normal impulse) and Coulomb friction
  (stick/slip in the tangential plane).
* Update position using the post-solve velocity (symplectic Euler for translation).
* Apply a **position-only** correction to eliminate tiny numerical penetration.

Notes
-----
* This assumes a single contact at the COM (no torques from contact).
* The impulse solve is closed-form for a single contact with isotropic friction.
"""
function _grounded_blcp_substep(
    f_free::CoupledMultirotorModel,
    c::FlatGroundConstraintContact,
    t_us::UInt64,
    x::PlantState{N,B},
    u::PlantInput,
    dt_us::UInt64,
) where {N,B}
    dt_us == 0 && return x

    t_s = Float64(t_us) * 1e-6
    dt_s = Float64(dt_us) * 1e-6

    # Smooth plant dynamics without contact (single evaluation).
    d = f_free(t_s, x, u)

    # Integrate non-rigid-body plant states (Euler; dt is small in grounded mode).
    motors_y = x.motors_y + d.motors_y_dot * dt_s
    motors_ydot = x.motors_ydot + d.motors_ydot_dot * dt_s
    servos_y = x.servos_y + d.servos_y_dot * dt_s
    servos_ydot = x.servos_ydot + d.servos_ydot_dot * dt_s
    rotor_ω = x.rotor_ω + d.rotor_ω_dot * dt_s
    power = PowerState{B}(
        soc = x.power.soc + d.power.soc_dot * dt_s,
        v1 = x.power.v1 + d.power.v1_dot * dt_s,
        temp_c = x.power.temp_c + d.power.temp_dot * dt_s,
    )

    # Rigid-body angular dynamics (contact-at-COM => no contact torque).
    q_new = quat_normalize(x.rb.q_bn + d.rb.q_dot * dt_s)
    ω_new = x.rb.ω_body + d.rb.ω_dot * dt_s

    # Translational dynamics: symplectic Euler + end-of-step impulse solve.
    v_pred = x.rb.vel_ned + d.rb.vel_dot * dt_s
    vx = v_pred[1]
    vy = v_pred[2]
    vz = v_pred[3]  # down-positive

    m = mass(f_free.model)

    # Normal impulse to prevent closing velocity at the plane.
    λ_n = 0.0
    vx_post = vx
    vy_post = vy
    vz_post = vz

    # Only solve contact if we are at/near the plane (grounded-mode assumption).
    if x.rb.pos_ned[3] >= -c.z_slop_m
        if vz > 0.0
            λ_n = m * vz
            vz_post = 0.0
        end

        # Tangential (x,y) friction impulse: stick if within Coulomb bound, else slip.
        if c.enable_friction && (λ_n > 0.0) && (c.μ > 0.0)
            vxy = sqrt(vx * vx + vy * vy)
            if vxy > 0.0
                Jt_req = m * vxy
                Jt_max = c.μ * λ_n
                if Jt_req <= Jt_max
                    # Static friction (stiction): cancel tangential velocity.
                    vx_post = 0.0
                    vy_post = 0.0
                else
                    # Dynamic friction: reduce tangential speed by Jt_max / m.
                    dv = Jt_max / m
                    s = (vxy - dv) / vxy
                    s = max(0.0, s)
                    vx_post = vx * s
                    vy_post = vy * s
                end
            end
        end
    end

    v_post = SVector{3,Float64}(vx_post, vy_post, vz_post)

    # Symplectic position update.
    pos1 = x.rb.pos_ned + v_post * dt_s

    # Position-only post-stabilization.
    z = pos1[3]
    if z > 0.0
        z = 0.0
    elseif (abs(z) <= c.z_slop_m) && (v_post[3] >= 0.0)
        # Snap tiny drift to the plane while in contact.
        z = 0.0
    end
    pos1 = SVector{3,Float64}(pos1[1], pos1[2], z)

    rb1 = RigidBodyState(pos_ned = pos1, vel_ned = v_post, q_bn = q_new, ω_body = ω_new)

    x1 = PlantState{N,B}(
        rb = rb1,
        motors_y = motors_y,
        motors_ydot = motors_ydot,
        servos_y = servos_y,
        servos_ydot = servos_ydot,
        rotor_ω = rotor_ω,
        power = power,
    )

    # Guardrail: clamp physical bounds *per grounded substep* so invalid
    # actuator/rotor states cannot feed into subsequent substeps.
    return plant_project(f_free, x1)
end

# Internal helper: deterministic integer-microsecond bisection for first touchdown.
function _locate_touchdown_us(
    f::CoupledMultirotorModel,
    probe_integrator,
    t0_s::Float64,
    x0::PlantState{N,B},
    u::PlantInput,
    dt_us::UInt64,
) where {N,B}
    lo = UInt64(0)
    hi = dt_us

    # Guard: dt_us > 0 and (z0 < 0 && zend >= 0) should have been established by the caller.
    while (hi - lo) > UInt64(1)
        mid = lo + (hi - lo) ÷ UInt64(2)
        reset!(probe_integrator)
        x_mid = step_integrator(probe_integrator, f, t0_s, x0, u, Float64(mid) * 1e-6)

        if x_mid.rb.pos_ned[3] < 0.0
            lo = mid
        else
            hi = mid
        end
    end

    return hi
end
