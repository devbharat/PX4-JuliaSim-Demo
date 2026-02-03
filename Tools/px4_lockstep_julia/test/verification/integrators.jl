@testset "Integrators: adaptive RK45 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    # Analytic solution in NED (positive z = down).
    @test isapprox(x1.vel_ned[3], g; atol = 1e-4)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 1e-4)

    st = Sim.Integrators.last_stats(integ1)
    @test st.nfev > 0
    @test st.naccept > 0

    # Determinism: a fresh integrator produces identical results.
    integ2 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK23 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.vel_ned[3], g; atol = 5e-4)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 5e-4)

    st = Sim.Integrators.last_stats(integ1)
    @test st.nfev > 0
    @test st.naccept > 0

    integ2 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: RK4 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK4Integrator()
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.vel_ned[3], g; atol = 1e-6)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 1e-6)

    integ2 = Sim.Integrators.RK4Integrator()
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK45 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 1e-4)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 1e-4)

    # Other state groups remain unchanged for this RHS.
    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    # Determinism check.
    integ2 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK23 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 5e-4)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 5e-4)

    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    integ2 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: RK4 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK4Integrator()
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 1e-6)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 1e-6)

    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    integ2 = Sim.Integrators.RK4Integrator()
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end


@testset "Integrators: plant-aware error norm is opt-in" begin
    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    ω_hi = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
    ω_lo = SVector{4,Float64}(90.0, 90.0, 90.0, 90.0)

    power0 = Sim.Plant.PowerState{1}(
        soc = SVector{1,Float64}(1.0),
        v1 = SVector{1,Float64}(0.0),
    )
    x_ref = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_hi, power = power0)
    x_hi = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_hi, power = power0)
    x_lo = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_lo, power = power0)

    integ = Sim.Integrators.RK45Integrator(
        plant_error_control = false,  # default behavior
        atol_rotor = 1.0,
        rtol_rotor = 0.0,
        # Keep RB tolerances finite but RB deltas are zero here.
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
    )

    err_off = Sim.Integrators._err_norm(integ, x_hi, x_lo, x_ref)
    @test err_off == 0.0

    integ.plant_error_control = true
    err_on = Sim.Integrators._err_norm(integ, x_hi, x_lo, x_ref)
    @test err_on > 1.0
end
