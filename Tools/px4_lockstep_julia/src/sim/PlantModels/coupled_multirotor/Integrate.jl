############################
# RHS functor
############################

function (f::CoupledMultirotorModel)(
    t::Float64,
    x::PlantState{N,B},
    u::PlantInput,
) where {N,B}
    # Actuator dynamics (pure; no mutation of actuator model objects).
    my_dot, mydot_dot =
        _actuator_derivs(f.motor_actuators, x.motors_y, x.motors_ydot, u.cmd.motors)
    sy_dot, sydot_dot =
        _actuator_derivs(f.servo_actuators, x.servos_y, x.servos_ydot, u.cmd.servos)

    # Propulsion + bus solve.
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} ||
        error("CoupledMultirotorModel currently supports QuadRotorSet{$N} propulsion")

    rot_out, ω_dot, _I_motors_total, _V_bus, _I_bus_total, I_batt, _ρ, _v_air_body =
        _eval_propulsion_and_power_network(
            p,
            f.batteries,
            f.power_net,
            f.env,
            t,
            x,
            u,
            f.motor_map,
            f.model.params.rotor_axis_body,
        )

    # Rigid-body dynamics.
    d_rb = Vehicles.dynamics(f.model, f.env, t, x.rb, rot_out, u.wind_ned)

    # Contact forces (NED) applied at COM.
    #
    # Two contact styles exist:
    # - Penalty/contact-force models: `contact_force_ned(model, x, t)`
    # - Constraint-based ground contact: `contact_force_ned(model, x, t, m, a_free_ned)`
    #
    # The constraint model needs the *unconstrained* acceleration; use the rigid-body
    # derivative computed above (contact not applied yet).
    F_contact = if f.contact isa NoContact
        SVector{3,Float64}(0.0, 0.0, 0.0)
    elseif f.contact isa FlatGroundConstraintContact
        contact_force_ned(f.contact, x.rb, t, mass(f.model), d_rb.vel_dot)
    else
        contact_force_ned(f.contact, x.rb, t)
    end
    if (F_contact[1] != 0.0) || (F_contact[2] != 0.0) || (F_contact[3] != 0.0)
        d_rb = RigidBodyDeriv(
            pos_dot = d_rb.pos_dot,
            vel_dot = d_rb.vel_dot + F_contact / mass(f.model),
            q_dot = d_rb.q_dot,
            ω_dot = d_rb.ω_dot,
        )
    end

    # Battery dynamics from per-battery current draw.
    soc_dot = MVector{B,Float64}(undef)
    v1_dot = MVector{B,Float64}(undef)
    temp_dot = MVector{B,Float64}(undef)
    @inbounds for i = 1:B
        cap_c = _battery_capacity_c(f.batteries[i])
        I = max(0.0, I_batt[i])
        sdot = -I / cap_c
        if x.power.soc[i] <= 0.0 && sdot < 0.0
            sdot = 0.0
        end
        soc_dot[i] = sdot
        v1_dot[i] = _battery_v1_dot(f.batteries[i], x.power.v1[i], I)

        # Thermal (optional): temperature is a plant state, held constant if disabled.
        temp_dot[i] = _battery_temp_dot(
            f.batteries[i],
            x.power.soc[i],
            x.power.temp_c[i],
            x.power.v1[i],
            I,
        )
    end

    power_dot = PowerDeriv{B}(
        soc_dot = SVector{B,Float64}(soc_dot),
        v1_dot = SVector{B,Float64}(v1_dot),
        temp_dot = SVector{B,Float64}(temp_dot),
    )

    return PlantDeriv{N,B}(
        rb = d_rb,
        motors_y_dot = my_dot,
        motors_ydot_dot = mydot_dot,
        servos_y_dot = sy_dot,
        servos_ydot_dot = sydot_dot,
        rotor_ω_dot = ω_dot,
        power = power_dot,
    )
end


############################
# Protocol: post-step projection
############################

"""Project plant state into physical bounds.

This is intentionally conservative: it is a last-line-of-defense clamp to prevent
NaN cascades from occasional overshoot in adaptive solvers.

Important: this is not meant to hide modeling bugs. If projection is frequently
active, tighten tolerances or fix the stiffness/discontinuity.
"""
function plant_project(f::CoupledMultirotorModel, x::PlantState{N,B}) where {N,B}
    # Optional ground-contact projection for constraint contact.
    #
    # This is a deterministic last-line-of-defense clamp to prevent slow numerical
    # drift below/above the z=0 plane when the vehicle is intended to be resting
    # on the ground.
    rb_proj = x.rb
    if f.contact isa FlatGroundConstraintContact
        # Position-only post-stabilization.
        #
        # We correct small penetration/drift in `z` without injecting momentum.
        # Velocities are determined by the contact impulse solve inside
        # `plant_integrate_interval` (grounded time-stepping) or by the smooth
        # dynamics in flight.
        z = rb_proj.pos_ned[3]
        z_slop = f.contact.z_slop_m

        # Snap tiny drift to the plane and prevent penetration.
        z2 = ((z > 0.0) || (abs(z) <= z_slop)) ? 0.0 : z

        if z2 != z
            pos = rb_proj.pos_ned
            rb_proj = RigidBodyState(
                pos_ned = SVector{3,Float64}(pos[1], pos[2], z2),
                vel_ned = rb_proj.vel_ned,
                q_bn = rb_proj.q_bn,
                ω_body = rb_proj.ω_body,
            )
        end
    end

    # Rotor speeds nonnegative.
    ω_clamped = map(w -> max(0.0, w), x.rotor_ω)

    # SOC in [0,1] (vectorized for B batteries).
    soc_clamped = map(s -> clamp(s, 0.0, 1.0), x.power.soc)

    # Battery temperature in a broad physically-plausible range (°C).
    # This is a conservative guardrail for adaptive solvers; the resistance surface
    # itself also clamps its temperature axis to data bounds.
    temp_clamped = map(T -> clamp(T, -50.0, 120.0), x.power.temp_c)

    # Actuator outputs in ABI-consistent ranges.
    motors_y_clamped = map(u -> clamp(u, 0.0, 1.0), x.motors_y)
    servos_y_clamped = map(u -> clamp(u, -1.0, 1.0), x.servos_y)

    # Rate limits for 2nd-order actuators (projection semantics).
    motors_ydot_proj = x.motors_ydot
    if f.motor_actuators isa Vehicles.SecondOrderActuators
        rl = f.motor_actuators.rate_limit
        if isfinite(rl)
            motors_ydot_proj = map(v -> clamp(v, -rl, rl), motors_ydot_proj)
        end
    else
        motors_ydot_proj = zero(SVector{12,Float64})
    end

    servos_ydot_proj = x.servos_ydot
    if f.servo_actuators isa Vehicles.SecondOrderActuators
        rl = f.servo_actuators.rate_limit
        if isfinite(rl)
            servos_ydot_proj = map(v -> clamp(v, -rl, rl), servos_ydot_proj)
        end
    else
        servos_ydot_proj = zero(SVector{8,Float64})
    end

    if (rb_proj !== x.rb) ||
       (ω_clamped != x.rotor_ω) ||
       (soc_clamped != x.power.soc) ||
       (temp_clamped != x.power.temp_c) ||
       (motors_y_clamped != x.motors_y) ||
       (servos_y_clamped != x.servos_y) ||
       (motors_ydot_proj != x.motors_ydot) ||
       (servos_ydot_proj != x.servos_ydot)
        power = PowerState{B}(soc = soc_clamped, v1 = x.power.v1, temp_c = temp_clamped)
        return PlantState{N,B}(
            rb = rb_proj,
            motors_y = motors_y_clamped,
            motors_ydot = motors_ydot_proj,
            servos_y = servos_y_clamped,
            servos_ydot = servos_ydot_proj,
            rotor_ω = ω_clamped,
            power = power,
        )
    end

    return x
end


############################
# Protocol: interval integration (hybrid ground contact)
############################

"""Protocol override: integrate with hybrid ground-contact handling.

For `FlatGroundConstraintContact` this implements a hybrid contact path:

* detect descending crossings of the `z=0` plane within the interval,
  localize the time of impact (TOI) to integer microseconds via bisection, and apply
  an instantaneous velocity reset (impact map).
* while grounded, integrate using a fixed-step time-stepping loop and
  a 1-contact BLCP impulse solve (normal + Coulomb friction).
* use position-only post-stabilization (no velocity clamping) to prevent
  long-horizon drift into the plane.
"""
function plant_integrate_interval(
    f::CoupledMultirotorModel,
    integrator,
    t0_us::UInt64,
    x0::PlantState{N,B},
    u::PlantInput,
    dt_us::UInt64,
) where {N,B}
    dt_us == 0 && return x0

    # Only special-case the constraint ground-contact model.
    if !(f.contact isa FlatGroundConstraintContact)
        t0_s = Float64(t0_us) * 1e-6
        dt_s = Float64(dt_us) * 1e-6
        return step_integrator(integrator, f, t0_s, x0, u, dt_s)
    end

    c = f.contact

    # Pre-build a "free" model (no contact) for the grounded time-stepping loop.
    # This avoids double-counting when we apply contact impulses.
    f_free = CoupledMultirotorModel(
        f.model,
        f.env,
        NoContact(),
        f.motor_actuators,
        f.servo_actuators,
        f.propulsion,
        f.batteries,
        f.power_net,
        f.motor_map,
        f.servo_map,
    )

    # Deterministic guardrails (avoid pathological loops).
    MAX_IMPACTS_PER_INTERVAL = 4
    n_impacts = 0

    # Interval contact telemetry: max impact Δv and its timestamp.
    #
    # This is latched and returned to the engine so we can log spikes without
    # requiring high-rate per-substep logging.
    max_impact_dv_ned = SVector{3,Float64}(0.0, 0.0, 0.0)
    max_impact_dv_norm2 = 0.0
    t_max_impact_us = UInt64(0)

    # Work vars for the current remaining segment.
    t_seg_us = t0_us
    x = x0
    remaining_us = dt_us

    # Use the caller-provided integrator as a probe for lookahead + bisection.
    # The canonical Engine resets the integrator at each interval boundary, so
    # there is no cross-interval adaptive history to preserve.
    probe = integrator

    while remaining_us > 0
        # Grounded time-stepping loop (fixed substeps + impulse solve).
        if _ground_candidate(c, x.rb)
            h_us = max(UInt64(1), c.h_contact_us)
            dt_step_us = remaining_us < h_us ? remaining_us : h_us

            x = _grounded_blcp_substep(f_free, c, t_seg_us, x, u, dt_step_us)
            t_seg_us += dt_step_us
            remaining_us -= dt_step_us

            # If we lifted off during this substep, the predicate will flip on the
            # next loop and we fall back to airborne integration.
            continue
        end

        # Airborne segment with TOI localization and impact map.
        t_seg_s = Float64(t_seg_us) * 1e-6

        # Safe step-size limiter near the guard.
        #
        # We integrate flight in chunks capped to O(time-to-impact) based on the
        # current distance to the plane and closing speed. This reduces the chance
        # of missing a crossing when z(t) is non-monotonic over a long interval.
        dt_step_us = _airborne_guard_step_us(c, x.rb, remaining_us)
        dt_step_s = Float64(dt_step_us) * 1e-6

        # Lookahead end state to see whether touchdown is bracketed.
        #
        # IMPORTANT: use the *free* dynamics (NoContact) for lookahead + TOI
        # localization. Contact reaction forces must not affect the guard crossing.
        reset!(probe)
        x_end = step_integrator(probe, f_free, t_seg_s, x, u, dt_step_s)

        z0 = x.rb.pos_ned[3]
        z1 = x_end.rb.pos_ned[3]

        # Touchdown is a descending crossing from above-ground (z<0) to contact/penetration (z>=0).
        if (z0 < 0.0) && (z1 >= 0.0) && (n_impacts < MAX_IMPACTS_PER_INTERVAL)
            tau_us = _locate_touchdown_us(f_free, probe, t_seg_s, x, u, dt_step_us)

            # Integrate up to the TOI using the *free* dynamics.
            reset!(integrator)
            x_imp =
                step_integrator(integrator, f_free, t_seg_s, x, u, Float64(tau_us) * 1e-6)

            # Apply the instantaneous impact map on the rigid-body state.
            rb_pre = x_imp.rb
            rb_post = _apply_ground_impact(c, rb_pre)
            x = _with_rb(x_imp, rb_post)

            # Impact telemetry (Δv) for logging.
            dv = rb_post.vel_ned - rb_pre.vel_ned
            dv_norm2 = dv[1] * dv[1] + dv[2] * dv[2] + dv[3] * dv[3]
            if dv_norm2 > max_impact_dv_norm2
                max_impact_dv_norm2 = dv_norm2
                max_impact_dv_ned = dv
                t_max_impact_us = t_seg_us + tau_us
            end

            # Advance segment bookkeeping.
            t_seg_us += tau_us
            remaining_us -= tau_us
            n_impacts += 1

            # Continue with the remainder of the interval (could include bounce + re-impact).
            continue
        end

        # No touchdown in this chunk: integrate as flight using the free dynamics.
        reset!(integrator)
        x = step_integrator(integrator, f_free, t_seg_s, x, u, dt_step_s)
        t_seg_us += dt_step_us
        remaining_us -= dt_step_us
        continue
    end

    info = (
        impact_count = n_impacts,
        impact_dv_ned = max_impact_dv_ned,
        impact_time_us = t_max_impact_us,
    )
    return x, info
end


"""Apply boundary-time updates at an autopilot tick.

This is used to keep hybrid semantics correct for `DirectActuators`:
`motors_y/servos_y` are algebraic outputs and must be snapped to the newly
published command at the tick boundary.

For non-direct actuators, this is a no-op.
"""
function plant_on_autopilot_tick(
    f::CoupledMultirotorModel,
    x::PlantState{N,B},
    cmd::ActuatorCommand,
) where {N,B}
    x2 = x

    if f.motor_actuators isa Vehicles.DirectActuators
        x2 = PlantState{N,B}(
            rb = x2.rb,
            motors_y = cmd.motors,
            motors_ydot = zero(SVector{12,Float64}),
            servos_y = x2.servos_y,
            servos_ydot = x2.servos_ydot,
            rotor_ω = x2.rotor_ω,
            power = x2.power,
        )
    end

    if f.servo_actuators isa Vehicles.DirectActuators
        x2 = PlantState{N,B}(
            rb = x2.rb,
            motors_y = x2.motors_y,
            motors_ydot = x2.motors_ydot,
            servos_y = cmd.servos,
            servos_ydot = zero(SVector{8,Float64}),
            rotor_ω = x2.rotor_ω,
            power = x2.power,
        )
    end

    return x2
end
