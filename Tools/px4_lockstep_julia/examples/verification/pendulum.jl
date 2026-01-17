using PX4Lockstep
using Printf

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

"""Run nonlinear pendulum verification.

For small angles, compare against the small-angle analytic solution.
Also report energy drift (nonlinear energy invariant).
"""
function main()
    case = V.PendulumCase(g = 9.80665, L = 1.0, θ0 = 0.1, θdot0 = 0.0)
    f = V.pendulum_rhs(case)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.θ0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.θdot0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    # One small-angle period.
    ω = sqrt(case.g / case.L)
    T = 2π / ω
    dt = 0.01
    n = Int(round(T / dt))
    t_final = n * dt

    println("\nPendulum (small-angle analytic check)")
    println("====================================")
    @printf("%-6s  %-8s  %-12s  %-12s\n", "Solver", "dt", "|θ-θ*|", "ΔE/E")

    for (name, integ) in [
        ("Euler", Sim.Integrators.EulerIntegrator()),
        ("RK4", Sim.Integrators.RK4Integrator()),
    ]
        x_end, t_end = V.integrate_fixed(integ, f, x0, dt, t_final)
        θ_ref, θdot_ref, _ = V.pendulum_small_angle_analytic(case, t_end)
        θ_err = abs(x_end.pos_ned[1] - θ_ref)

        E0 = V.pendulum_energy(case, case.θ0, case.θdot0)
        E1 = V.pendulum_energy(case, x_end.pos_ned[1], x_end.vel_ned[1])
        dE = abs(E1 - E0) / E0

        @printf("%-6s  %-8.4g  %-12.3e  %-12.3e\n", name, dt, θ_err, dE)
    end

    adaptive = [
        (
            "RK23",
            Sim.Integrators.RK23Integrator(
                rtol_pos = 1e-10,
                atol_pos = 1e-10,
                rtol_vel = 1e-10,
                atol_vel = 1e-10,
                rtol_ω = 1e-12,
                atol_ω = 1e-12,
                atol_att_rad = 1e-12,
                h_min = 1e-6,
                h_max = 0.05,
            ),
        ),
        (
            "RK45",
            Sim.Integrators.RK45Integrator(
                rtol_pos = 1e-10,
                atol_pos = 1e-10,
                rtol_vel = 1e-10,
                atol_vel = 1e-10,
                rtol_ω = 1e-12,
                atol_ω = 1e-12,
                atol_att_rad = 1e-12,
                h_min = 1e-6,
                h_max = 0.05,
            ),
        ),
    ]

    stats = Tuple{String,Sim.Integrators.IntegratorStats}[]
    for (name, integ) in adaptive
        x_end = Sim.Integrators.step_integrator(integ, f, 0.0, x0, nothing, t_final)
        θ_ref, θdot_ref, _ = V.pendulum_small_angle_analytic(case, t_final)
        θ_err = abs(x_end.pos_ned[1] - θ_ref)

        E0 = V.pendulum_energy(case, case.θ0, case.θdot0)
        E1 = V.pendulum_energy(case, x_end.pos_ned[1], x_end.vel_ned[1])
        dE = abs(E1 - E0) / E0

        @printf("%-6s  %-8s  %-12.3e  %-12.3e\n", name, "adapt", θ_err, dE)
        push!(stats, (name, Sim.Integrators.last_stats(integ)))
    end

    for (name, st) in stats
        println("\n$(name) stats: nfev=$(st.nfev), accept=$(st.naccept), reject=$(st.nreject), h_last=$(st.h_last)")
    end
end

main()
