const V = Sim.Verification

@testset "Verification: simple harmonic oscillator (analytic)" begin
    case = V.SHOCase(ω0 = 2π, x0 = 1.0, v0 = 0.0)
    f = V.sho_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.x0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.v0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    T = 1.0
    dt = 0.01

    x_euler, _ = V.integrate_fixed(Sim.Integrators.EulerIntegrator(), f, x0, dt, T)
    x_rk4, _ = V.integrate_fixed(Sim.Integrators.RK4Integrator(), f, x0, dt, T)

    x_ref, v_ref = V.sho_analytic(case, T)
    err_euler = abs(x_euler.pos_ned[1] - x_ref)
    err_rk4 = abs(x_rk4.pos_ned[1] - x_ref)

    # RK4 should be substantially more accurate than Euler at the same dt.
    @test err_rk4 < 1e-6
    @test err_euler > err_rk4

    # Energy drift over 10 periods should stay bounded for RK4 at dt=0.01.
    T10 = 10.0
    n = Int(round(T10 / dt))
    x = x0
    t = 0.0
    E0 = V.sho_energy(case, case.x0, case.v0)
    Emax = E0
    Emin = E0
    rk4 = Sim.Integrators.RK4Integrator()
    for _ = 1:n
        x = Sim.Integrators.step_integrator(rk4, f, t, x, nothing, dt)
        t += dt
        E = V.sho_energy(case, x.pos_ned[1], x.vel_ned[1])
        Emax = max(Emax, E)
        Emin = min(Emin, E)
    end
    drift = (Emax - Emin) / E0
    @test drift < 1e-4
end


@testset "Verification: pendulum (small-angle analytic + energy invariant)" begin
    case = V.PendulumCase(g = 9.80665, L = 1.0, θ0 = 0.1, θdot0 = 0.0)
    f = V.pendulum_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.θ0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.θdot0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    ω = sqrt(case.g / case.L)
    T = 2π / ω
    dt = 0.01
    n = Int(round(T / dt))
    t_end = n * dt

    x_rk4, t_end = V.integrate_fixed(Sim.Integrators.RK4Integrator(), f, x0, dt, t_end)

    θ_ref, θdot_ref, _ = V.pendulum_small_angle_analytic(case, t_end)
    @test abs(x_rk4.pos_ned[1] - θ_ref) < 1e-5

    E0 = V.pendulum_energy(case, case.θ0, case.θdot0)
    E1 = V.pendulum_energy(case, x_rk4.pos_ned[1], x_rk4.vel_ned[1])
    @test abs(E1 - E0) / E0 < 1e-4
end


@testset "Verification: circular Kepler orbit (analytic + invariants)" begin
    case = V.KeplerCircularCase(
        μ = 1.0,
        r0 = Sim.Types.vec3(1.0, 0.0, 0.0),
        v0 = Sim.Types.vec3(0.0, 1.0, 0.0),
    )
    f = V.kepler_rhs(case.μ)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = case.r0,
        vel_ned = case.v0,
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    T = 2π
    dt = 0.01
    n = Int(round(T / dt))
    T_end = n * dt

    rk4 = Sim.Integrators.RK4Integrator()
    x = x0
    t = 0.0

    E0 = V.kepler_energy(case.μ, x0.pos_ned, x0.vel_ned)
    h0 = V.kepler_angmom(x0.pos_ned, x0.vel_ned)

    for _ = 1:n
        x = Sim.Integrators.step_integrator(rk4, f, t, x, nothing, dt)
        t += dt
    end

    r_ref, v_ref, _ω = V.kepler_circular_analytic(case, T_end)
    @test norm(x.pos_ned - r_ref) < 1e-5
    @test norm(x.vel_ned - v_ref) < 1e-5

    E1 = V.kepler_energy(case.μ, x.pos_ned, x.vel_ned)
    h1 = V.kepler_angmom(x.pos_ned, x.vel_ned)

    @test abs(E1 - E0) / abs(E0) < 1e-6
    @test norm(h1 - h0) / norm(h0) < 1e-6
end


@testset "Verification: torque-free rigid body (invariants)" begin
    case = V.TorqueFreeRigidBodyCase(
        I_body = Sim.Types.vec3(2.0, 1.0, 0.5),
        ω0 = Sim.Types.vec3(1.0, 2.0, 3.0),
        q0 = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
    )
    f = V.torque_free_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = case.q0,
        ω_body = case.ω0,
    )

    T = 2.0
    dt = 0.001
    n = Int(round(T / dt))

    rk4 = Sim.Integrators.RK4Integrator()
    x = x0
    t = 0.0

    E0 = V.rigidbody_rot_energy(case.I_body, x0.ω_body)
    L0 = V.rigidbody_angmom_ned(x0.q_bn, case.I_body, x0.ω_body)

    for _ = 1:n
        x = Sim.Integrators.step_integrator(rk4, f, t, x, nothing, dt)
        t += dt
    end

    E1 = V.rigidbody_rot_energy(case.I_body, x.ω_body)
    L1 = V.rigidbody_angmom_ned(x.q_bn, case.I_body, x.ω_body)

    @test abs(E1 - E0) / E0 < 1e-6
    @test norm(L1 - L0) < 1e-4
    @test abs(norm(x.q_bn) - 1.0) < 1e-10
end


@testset "Verification: RK45 reference compare utility" begin
    # Large-angle pendulum (nonlinear). Use RK45 on a fine grid as numerical truth
    # and compare RK4 on a coarse grid.
    case = V.PendulumCase(g = 9.80665, L = 1.0, θ0 = 1.0, θdot0 = 0.0)
    f = V.pendulum_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.θ0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.θdot0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    dt_sol = 0.02
    dt_ref = dt_sol / 10
    t_end = 0.4

    ref_fine = V.rk45_reference(f, x0, dt_ref, t_end)
    ref = V.resample_trajectory(ref_fine, dt_sol)
    sol = V.simulate_trajectory(Sim.Integrators.RK4Integrator(), f, x0, dt_sol, t_end)

    invfun = (s::Sim.RigidBody.RigidBodyState,) -> (E = V.pendulum_energy(case, s.pos_ned[1], s.vel_ned[1]),)
    cmp = V.compare_to_reference(ref, sol; invfun = invfun)

    # Basic sanity: arrays are the right size and errors are finite and nonzero.
    @test length(cmp.err.t) == length(ref.x)
    @test isfinite(cmp.err.max.pos)
    @test isfinite(cmp.err.max.vel)
    @test cmp.err.max.pos > 0.0

    # Reference should drift less in energy than the coarse RK4 solution.
    drift_ref = maximum(abs.(cmp.invariants.ref.drift.E))
    drift_sol = maximum(abs.(cmp.invariants.sol.drift.E))
    @test drift_ref < drift_sol
end



@testset "Verification: RK45 quantize_us does not drift timebase (SHO)" begin
    # Regression test for microsecond quantization: adaptive solvers must not "creep"
    # past the requested interval due to tiny floating-point residual `remaining`.
    case = V.SHOCase(ω0 = 2π, x0 = 1.0, v0 = 0.0)
    f = V.sho_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.x0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.v0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    dt = 0.0004
    t_end = 0.5

    tr = V.rk45_reference(f, x0, dt, t_end; quantize_us = true, h_max = dt)

    x_ref, v_ref = V.sho_analytic(case, t_end)
    @test abs(tr.x[end].pos_ned[1] - x_ref) < 1e-6
    @test abs(tr.x[end].vel_ned[1] - v_ref) < 1e-6
end

@testset "Verification: RK45 reference compare utility (PlantState)" begin
    # Full-plant reference compare: embed an SHO into the rigid-body subset and add
    # rotor ω + battery SOC/V1 continuous states. Use RK45 on a fine grid as numerical
    # truth and compare RK4 on a coarse grid.
    sho = V.SHOCase(ω0 = 2 * pi, x0 = 1.0, v0 = 0.0)
    f_rb = V.sho_rhs(sho)

    omega_target = SVector{4,Float64}(400.0, 420.0, 410.0, 430.0)
    alpha = 200.0
    soc_dot = -0.01
    tau_v1 = 0.2

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, _u)
        rb_dot = f_rb(t, x.rb, nothing)
        rotor_dot = -alpha .* (x.rotor_ω - omega_target)
        power_dot = Sim.Plant.PowerDeriv{1}(
            soc_dot = SVector{1,Float64}(soc_dot),
            v1_dot = SVector{1,Float64}(-x.power.v1[1] / tau_v1),
        )
        return Sim.Plant.PlantDeriv{4,1}(
            rb = rb_dot,
            rotor_ω_dot = rotor_dot,
            power = power_dot,
        )
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(sho.x0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(sho.v0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        rotor_ω = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(1.0),
        ),
    )

    dt_sol = 0.02
    dt_ref = dt_sol / 10
    t_end = 0.4

    ref_fine = V.rk45_reference(
        f,
        x0,
        dt_ref,
        t_end;
        plant_error_control = true,
        # Include non-RB states in the *reference* adaptivity so rotor/battery accuracy
        # is not accidentally ignored.
        atol_rotor = 1e-10,
        atol_soc = 1e-12,
        atol_v1 = 1e-12,
    )
    ref = V.resample_trajectory(ref_fine, dt_sol)
    sol = V.simulate_trajectory(Sim.Integrators.RK4Integrator(), f, x0, dt_sol, t_end)

    invfun = (s::Sim.Plant.PlantState{4,1},) -> (
        E_sho = V.sho_energy(sho, s.rb.pos_ned[1], s.rb.vel_ned[1]),
    )
    cmp = V.compare_to_reference(ref, sol; invfun = invfun)

    @test length(cmp.err.t) == length(ref.x)
    @test isfinite(cmp.err.max.rotor)
    @test isfinite(cmp.err.max.soc)
    @test isfinite(cmp.err.max.v1)
    @test cmp.err.max.rotor > 0.0

    drift_ref = maximum(abs.(cmp.invariants.ref.drift.E_sho))
    drift_sol = maximum(abs.(cmp.invariants.sol.drift.E_sho))
    @test drift_ref < drift_sol
end
