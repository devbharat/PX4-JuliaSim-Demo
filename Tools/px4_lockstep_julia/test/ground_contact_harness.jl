using Test
using PX4Lockstep

const Sim = PX4Lockstep.Sim
const T = Sim.Types
const RB = Sim.RigidBody
const INTEG = Sim.Integrators
const CONTACT = Sim.Contacts

"""Simple vertical point-mass + ground-contact RHS for contact regression tests.

This deliberately isolates the integrator + contact interaction, without involving
propulsion, actuator dynamics, or PX4-in-the-loop scheduling.
"""
struct PointMassGroundContact
    contact::CONTACT.FlatGroundContact
    mass_kg::Float64
    g_ned::Float64
end
struct ThrustInput
    thrust_n::Float64
end

function (f::PointMassGroundContact)(t::Float64, x::RB.RigidBodyState, u::ThrustInput)
    F_contact = CONTACT.contact_force_ned(f.contact, x, t)
    F_thrust = T.vec3(0.0, 0.0, -u.thrust_n) # thrust up is -z in NED
    a = (F_contact + F_thrust) / f.mass_kg + T.vec3(0.0, 0.0, f.g_ned)

    return RB.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = a,
        q_dot = RB.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = T.vec3(0.0, 0.0, 0.0),
    )
end

function _simulate_contact_case(integ, f, x0, dt_s::Float64, t_end_s::Float64, thrust_fn)
    # Avoid cross-test coupling for adaptive integrators.
    INTEG.reset!(integ)
    n = Int(round(t_end_s / dt_s))
    x = x0
    t = 0.0

    # Metrics.
    max_z = x.pos_ned[3]
    min_z = x.pos_ned[3]
    max_abs_vz = abs(x.vel_ned[3])
    prev_z = x.pos_ned[3]
    n_touch = 0

    for _ = 1:n
        u = ThrustInput(thrust_fn(t))
        x = INTEG.step_integrator(integ, f, t, x, u, dt_s)
        t += dt_s

        z = x.pos_ned[3]
        max_z = max(max_z, z)
        min_z = min(min_z, z)
        max_abs_vz = max(max_abs_vz, abs(x.vel_ned[3]))

        # "Touchdown" event detector for Phase 0 harness only:
        # counts descending crossings from above-ground (z<0) to contact/penetration (z>=0).
        if prev_z < 0.0 && z >= 0.0
            n_touch += 1
        end
        prev_z = z
    end

    return (; x, max_z, min_z, max_abs_vz, n_touch)
end

"""Point-mass model using constraint-reaction ground contact + projection.

This mirrors the intended Phase 2 behavior in the full plant model:
* non-stiff normal reaction during resting contact, and
* a simple post-step projection to enforce non-penetration.
"""
struct PointMassConstraintGroundContact
    contact::CONTACT.FlatGroundConstraintContact
    mass_kg::Float64
    g_ned::Float64
end

function (f::PointMassConstraintGroundContact)(t::Float64, x::RB.RigidBodyState, u::ThrustInput)
    # Free acceleration (no contact) from thrust + gravity.
    F_thrust = T.vec3(0.0, 0.0, -u.thrust_n)
    a_free = F_thrust / f.mass_kg + T.vec3(0.0, 0.0, f.g_ned)

    F_contact = CONTACT.contact_force_ned(f.contact, x, t, f.mass_kg, a_free)
    a = a_free + F_contact / f.mass_kg

    return RB.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = a,
        q_dot = RB.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = T.vec3(0.0, 0.0, 0.0),
    )
end

@inline function _project_ground(c::CONTACT.FlatGroundConstraintContact, x::RB.RigidBodyState)
    z = x.pos_ned[3]
    z_slop = c.z_slop_m

    # Phase 5 (Plan1): position-only post-stabilization. Do not clamp velocities.
    z2 = ((z > 0.0) || (abs(z) <= z_slop)) ? 0.0 : z

    if z2 != z
        pos = x.pos_ned
        return RB.RigidBodyState(
            pos_ned = T.vec3(pos[1], pos[2], z2),
            vel_ned = x.vel_ned,
            q_bn = x.q_bn,
            ω_body = x.ω_body,
        )
    end
    return x
end

function _simulate_contact_case_projected(
    integ,
    f::PointMassConstraintGroundContact,
    x0,
    dt_s::Float64,
    t_end_s::Float64,
    thrust_fn,
)
    INTEG.reset!(integ)

    n = Int(round(t_end_s / dt_s))
    x = x0
    t = 0.0

    max_z = x.pos_ned[3]
    min_z = x.pos_ned[3]
    max_abs_vz = abs(x.vel_ned[3])
    prev_z = x.pos_ned[3]
    n_touch = 0

    for _ = 1:n
        u = ThrustInput(thrust_fn(t))
        x = INTEG.step_integrator(integ, f, t, x, u, dt_s)
        x = _project_ground(f.contact, x)
        t += dt_s

        z = x.pos_ned[3]
        max_z = max(max_z, z)
        min_z = min(min_z, z)
        max_abs_vz = max(max_abs_vz, abs(x.vel_ned[3]))

        if prev_z < 0.0 && z >= 0.0
            n_touch += 1
        end
        prev_z = z
    end

    return (; x, max_z, min_z, max_abs_vz, n_touch)
end

@testset "Ground contact: ContactInfo primitives" begin
    rb_air = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, -1.0), vel_ned = T.vec3(0.0, 0.0, 0.0))
    rb_pen = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, 0.1), vel_ned = T.vec3(0.0, 0.0, 0.0))

    @test CONTACT.ground_gap_m(rb_air) ≈ 1.0
    @test CONTACT.ground_gap_m(rb_pen) ≈ -0.1
    @test CONTACT.ground_normal_ned() == T.vec3(0.0, 0.0, -1.0)

    c = CONTACT.FlatGroundContact(enable_friction = false)
    ci_air = CONTACT.contact_info(c, rb_air, 0.0)
    ci_pen = CONTACT.contact_info(c, rb_pen, 0.0)

    @test ci_air.active == false
    @test ci_pen.active == true
    @test ci_pen.penetration_m > 0.0
    @test ci_pen.normal_force_n >= 0.0
end

@testset "Ground contact: Phase 0 regression harness (expected to fail until hybrid contact is implemented)" begin
    g = 9.80665
    m = 1.5
    mg = m * g

    # Use a relatively stiff contact to expose the adaptive-solver pain points.
    contact = CONTACT.FlatGroundContact(
        k_n_per_m = 20_000.0,
        c_n_per_mps = 800.0,
        enable_friction = false,
    )
    f = PointMassGroundContact(contact, m, g)

    q0 = T.Quat(1.0, 0.0, 0.0, 0.0)
    ω0 = T.vec3(0.0, 0.0, 0.0)

    # Keep the harness cheap (these will be strengthened once contact is fixed).
    dt = 0.01

    integrators = [
        :RK4 => INTEG.RK4Integrator(),
        :RK23 => INTEG.RK23Integrator(
            h_min = 1e-8,
            h_max = 0.1,
            max_substeps = 50_000,
            rtol_pos = 1e-4,
            atol_pos = 1e-4,
            rtol_vel = 1e-4,
            atol_vel = 1e-4,
            rtol_ω = 1e-4,
            atol_ω = 1e-4,
            atol_att_rad = 1e-4,
        ),
        :RK45 => INTEG.RK45Integrator(
            h_min = 1e-8,
            h_max = 0.1,
            max_substeps = 50_000,
            rtol_pos = 1e-4,
            atol_pos = 1e-4,
            rtol_vel = 1e-4,
            atol_vel = 1e-4,
            rtol_ω = 1e-4,
            atol_ω = 1e-4,
            atol_att_rad = 1e-4,
        ),
    ]

    z_tol = 1e-4
    z_tol_rest = 1e-3
    v_tol = 0.2

    # 1) Rest-on-ground: start at z=0, motors off.
    # Desired after fix: no penetration, stable rest, no solver failures.
    for (name, integ) in integrators
        x0 = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, 0.0), vel_ned = T.vec3(0.0, 0.0, 0.0), q_bn = q0, ω_body = ω0)
        ok = true
        res = nothing
        try
            res = _simulate_contact_case(integ, f, x0, dt, 3.0, t -> 0.0)
        catch
            ok = false
        end

        @test ok && (res.max_z <= z_tol_rest) && (res.max_abs_vz <= v_tol)
    end

    # 2) Drop test: start 1m above ground, motors off.
    for (name, integ) in integrators
        x0 = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, -1.0), vel_ned = T.vec3(0.0, 0.0, 0.0), q_bn = q0, ω_body = ω0)
        ok = true
        res = nothing
        try
            res = _simulate_contact_case(integ, f, x0, dt, 3.0, t -> 0.0)
        catch
            ok = false
        end

        # Desired after fix: exactly one touchdown event, settle, no penetration.
        @test_broken ok && (res.n_touch == 1) && (res.max_z <= z_tol) && (res.max_abs_vz <= 5.0)
    end

    # 3) Takeoff test: start landed, ramp thrust from 0 -> 1.3*mg.
    for (name, integ) in integrators
        x0 = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, 0.0), vel_ned = T.vec3(0.0, 0.0, 0.0), q_bn = q0, ω_body = ω0)
        ok = true
        res = nothing
        try
            thrust_fn = t -> begin
                if t < 0.5
                    0.0
                elseif t < 1.5
                    α = (t - 0.5) / 1.0
                    (1.3 * mg) * α
                else
                    1.3 * mg
                end
            end
            res = _simulate_contact_case(integ, f, x0, dt, 2.5, thrust_fn)
        catch
            ok = false
        end

        # Desired after fix: clean liftoff (z becomes negative), no "sticking" or chatter.
        @test ok
        if ok
            @test res.min_z < -0.05
        end
    end

    # 4) Landing test: start hovering at 2m, then cut thrust.
    for (name, integ) in integrators
        x0 = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, -2.0), vel_ned = T.vec3(0.0, 0.0, 0.0), q_bn = q0, ω_body = ω0)
        ok = true
        res = nothing
        try
            thrust_fn = t -> (t < 1.0 ? mg : 0.0)
            res = _simulate_contact_case(integ, f, x0, dt, 3.0, thrust_fn)
        catch
            ok = false
        end

        # Desired after fix: exactly one touchdown event, no penetration after settle.
        @test_broken ok && (res.n_touch == 1) && (res.max_z <= z_tol)
    end

    # 5) High-speed "tunneling" test: start 10m above ground with high down velocity.
    for (name, integ) in integrators
        x0 = RB.RigidBodyState(pos_ned = T.vec3(0.0, 0.0, -10.0), vel_ned = T.vec3(0.0, 0.0, 200.0), q_bn = q0, ω_body = ω0)
        ok = true
        res = nothing
        try
            res = _simulate_contact_case(integ, f, x0, 0.02, 0.2, t -> 0.0)
        catch
            ok = false
        end

        # Desired after fix: event localization prevents deep penetration even with a coarse step.
        @test_broken ok && (res.n_touch >= 1) && (res.max_z <= 0.05)
    end
end

@testset "Ground contact: Phase 2 constraint contact (resting stability + clean takeoff)" begin
    g = 9.80665
    m = 1.5
    mg = m * g

    contact = CONTACT.FlatGroundConstraintContact(enable_friction = false)
    f = PointMassConstraintGroundContact(contact, m, g)

    q0 = T.Quat(1.0, 0.0, 0.0, 0.0)
    ω0 = T.vec3(0.0, 0.0, 0.0)

    dt = 0.01

    integrators = [
        :RK4 => INTEG.RK4Integrator(),
        :RK23 => INTEG.RK23Integrator(h_min = 1e-8, h_max = 0.1, max_substeps = 50_000),
        :RK45 => INTEG.RK45Integrator(h_min = 1e-8, h_max = 0.1, max_substeps = 50_000),
    ]

    # These are intentionally conservative. The goal in Phase 2 is:
    # - adaptive solvers must *not* run out of steps when resting on the ground, and
    # - the simple projection keeps the state on/above the plane.
    z_tol = 1e-6
    v_tol = 1e-2

    # 1) Rest-on-ground: start at z=0, motors off.
    for (_name, integ) in integrators
        x0 = RB.RigidBodyState(
            pos_ned = T.vec3(0.0, 0.0, 0.0),
            vel_ned = T.vec3(0.0, 0.0, 0.0),
            q_bn = q0,
            ω_body = ω0,
        )

        ok = true
        res = nothing
        try
            res = _simulate_contact_case_projected(integ, f, x0, dt, 3.0, t -> 0.0)
        catch
            ok = false
        end

        @test ok && (res.max_z <= z_tol) && (res.max_abs_vz <= v_tol)
    end

    # 2) Takeoff: start landed, ramp thrust from 0 -> 1.3*mg.
    for (_name, integ) in integrators
        x0 = RB.RigidBodyState(
            pos_ned = T.vec3(0.0, 0.0, 0.0),
            vel_ned = T.vec3(0.0, 0.0, 0.0),
            q_bn = q0,
            ω_body = ω0,
        )

        ok = true
        res = nothing
        try
            thrust_fn = t -> begin
                if t < 0.5
                    0.0
                elseif t < 1.5
                    α = (t - 0.5) / 1.0
                    (1.3 * mg) * α
                else
                    1.3 * mg
                end
            end
            res = _simulate_contact_case_projected(integ, f, x0, dt, 2.5, thrust_fn)
        catch
            ok = false
        end

        if _name == :RK23
            # RK23 still trips max-substeps in this projected takeoff case.
            @test_broken ok
            continue
        end
        @test ok
        if ok
            @test res.min_z < -0.05
        end
    end
end


@testset "Ground contact: Phase 3 hybrid impact map (TOI + restitution bounce)" begin
    # This is an engine-level regression for the Phase 3 protocol `plant_integrate_interval`.
    # We purposely set a nonzero restitution so a bounce must occur; without the hybrid
    # impact map the plant would settle on the plane (no bounce).

    # Build Iris plant components (TOML-first). Keep the run short and disable derived outputs
    # to make this test lightweight.
    env = iris_env_replay_for_tests()
    vehicle = iris_vehicle_for_tests()
    battery = iris_battery_for_tests()

    contact = CONTACT.FlatGroundConstraintContact(
        enable_friction = false,
        restitution = 0.5,
        v_rest_threshold_mps = 0.05,
        enable_impact_friction = false,
    )
    dynfun = iris_dynfun_for_tests(env, vehicle, battery; contact = contact)

    rb0 = RB.RigidBodyState(
        pos_ned = T.vec3(0.0, 0.0, -1.0),
        vel_ned = T.vec3(0.0, 0.0, 0.0),
        q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = T.vec3(0.0, 0.0, 0.0),
    )
    plant0 = Sim.Plant.init_plant_state(
        rb0,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = RT.dt_to_us(0.75) # 0.75s: after first impact (~0.45s) but before second (~0.9s)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = RT.dt_to_us(0.01),
        dt_wind_us = RT.dt_to_us(0.05),
        dt_log_us = RT.dt_to_us(0.01),
    )

    # Zero command + zero wind.
    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)

    wind_data = [T.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    integ = INTEG.RK4Integrator()

    cfg = RT.EngineConfig(mode = RT.MODE_REPLAY, enable_derived_outputs = false)
    sim = RT.Engine(
        cfg;
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
        scenario = nothing,
        estimator = nothing,
    )

    RT.run!(sim)

    # With restitution=0.5 we must be above ground after the first bounce.
    @test sim.plant.rb.pos_ned[3] < -0.05
end

@testset "Ground contact: hybrid tunneling regression (TOI in coarse step)" begin
    # High-speed touchdown should still be localized even with a large integration interval.

    env = iris_env_replay_for_tests()
    vehicle = iris_vehicle_for_tests()
    battery = iris_battery_for_tests()

    contact = CONTACT.FlatGroundConstraintContact(
        enable_friction = false,
        restitution = 0.0,           # inelastic impact: no bounce
        v_rest_threshold_mps = 0.05,
        enable_impact_friction = false,
        h_contact_us = UInt64(1000),
    )
    dynfun = iris_dynfun_for_tests(env, vehicle, battery; contact = contact)

    # Start 10 m above ground, high downward velocity.
    rb0 = RB.RigidBodyState(
        pos_ned = T.vec3(0.0, 0.0, -10.0),
        vel_ned = T.vec3(0.0, 0.0, 200.0),
        q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = T.vec3(0.0, 0.0, 0.0),
    )
    plant0 = Sim.Plant.init_plant_state(
        rb0,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Single coarse interval (0.2s) should still detect touchdown.
    integ = INTEG.RK4Integrator()
    t0_us = UInt64(0)
    dt_us = UInt64(200_000)
    u = Sim.Plant.PlantInput(cmd = Sim.Vehicles.ActuatorCommand(), wind_ned = T.vec3(0.0, 0.0, 0.0))

    res = Sim.plant_integrate_interval(dynfun, integ, t0_us, plant0, u, dt_us)
    @test (res isa Tuple) && (length(res) == 2)
    x1, info = res

    # Should not tunnel deeply below the plane.
    @test x1.rb.pos_ned[3] <= 1e-3

    # Impact metadata should record at least one impact.
    @test getproperty(info, :impact_count) >= 1
    @test getproperty(info, :impact_dv_ned)[3] < 0.0
end
