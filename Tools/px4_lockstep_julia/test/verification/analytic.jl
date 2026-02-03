@testset "Analytic: free-fall under gravity matches closed form" begin
    g = 9.80665
    dt = 0.01
    T = 1.0
    n = Int(round(T / dt))

    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.Types.Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Closed form in NED (z is down):
    #   v_z(t) = g t
    #   z(t)   = 0.5 g t^2
    z_exp = 0.5 * g * T^2
    vz_exp = g * T

    @test isapprox(x.pos_ned[3], z_exp; atol = 1e-9)
    @test isapprox(x.vel_ned[3], vz_exp; atol = 1e-9)
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)
end

@testset "Analytic: constant body-rate quaternion integration" begin
    dt = 0.001
    T = 1.0
    n = Int(round(T / dt))

    ω = π  # rad/s about body Z (down)
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, ω),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            vel_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Exact solution: q(t) = [cos(ωt/2), 0, 0, sin(ωt/2)]
    q_ref = Sim.Types.quat_from_axis_angle(Sim.Types.vec3(0.0, 0.0, 1.0), ω * T)
    θ_err = PX4Lockstep.Tests.Fixtures.quat_angle_error(x.q_bn, q_ref)

    @test θ_err < 1e-6
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)

    yaw = Sim.Types.yaw_from_quat(x.q_bn)
    @test isapprox(Sim.Types.wrap_pi(yaw), Sim.Types.wrap_pi(ω * T); atol = 1e-6)
end

@testset "Analytic: DelayedEstimator ring-buffer exactness" begin
    rng = MersenneTwister(0)
    dt_est = 0.01
    delay_s = 0.03
    delay_steps = 3

    inner = Sim.Estimators.TruthEstimator()
    est = Sim.Estimators.DelayedEstimator(inner; delay_s = delay_s, dt_est = dt_est)
    Sim.Estimators.reset!(est)

    outs = Float64[]
    for k = 0:10
        t = k * dt_est
        x = Sim.RigidBody.RigidBodyState(
            pos_ned = Sim.Types.vec3(Float64(k), 0.0, 0.0),
            vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        y = Sim.Estimators.estimate!(est, rng, t, x, dt_est)
        push!(outs, y.pos_ned[1])
    end

    # During fill, the estimator returns current.
    @test outs[1] == 0.0
    @test outs[2] == 1.0
    @test outs[3] == 2.0

    # After warm-up, output is delayed by exactly delay_steps samples.
    for k = delay_steps + 1:11
        # outs is 1-indexed; k corresponds to truth sample k-1.
        truth_k = k - 1
        @test outs[k] == Float64(truth_k - delay_steps)
    end

    # Reset should restore the warm-up behavior.
    Sim.Estimators.reset!(est)
    x = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(42.0, 0.0, 0.0))
    y = Sim.Estimators.estimate!(est, rng, 0.0, x, dt_est)
    @test y.pos_ned[1] == 42.0

    # Delay must be an exact multiple of dt_est (no silent rounding).
    @test_throws ErrorException Sim.Estimators.DelayedEstimator(
        inner;
        delay_s = 0.015,
        dt_est = dt_est,
    )

    # The runtime stepping cadence must match the configured dt_est.
    x = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0))
    @test_throws ErrorException Sim.Estimators.estimate!(est, rng, 0.0, x, 2 * dt_est)
end
