using PX4Lockstep
using Printf

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

"""Run the simple harmonic oscillator reference problem.

This measures final-state error against the analytic solution and reports energy drift.
"""
function main()
    case = V.SHOCase(ω0 = 2π, x0 = 1.0, v0 = 0.0)
    f = V.sho_rhs(case)

    # Embed SHO into a RigidBodyState.
    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(case.x0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(case.v0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    T = 1.0  # One period for ω0=2π.
    dt = 0.01

    configs = [
        ("Euler", Sim.Integrators.EulerIntegrator(), dt),
        ("RK4", Sim.Integrators.RK4Integrator(), dt),
    ]

    println("\nSimple harmonic oscillator (ω0 = 2π rad/s, T = 1 s)")
    println("=====================================================")
    @printf("%-6s  %-8s  %-12s  %-12s\n", "Solver", "dt", "|x-x*|", "ΔE/E")

    for (name, integ, h) in configs
        x_end, t_end = V.integrate_fixed(integ, f, x0, h, T)
        x_ref, v_ref = V.sho_analytic(case, t_end)
        x_err = abs(x_end.pos_ned[1] - x_ref)

        E0 = V.sho_energy(case, case.x0, case.v0)
        E1 = V.sho_energy(case, x_end.pos_ned[1], x_end.vel_ned[1])
        dE = abs(E1 - E0) / E0

        @printf("%-6s  %-8.4g  %-12.3e  %-12.3e\n", name, h, x_err, dE)
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
        x_end = Sim.Integrators.step_integrator(integ, f, 0.0, x0, nothing, T)
        x_ref, v_ref = V.sho_analytic(case, T)
        x_err = abs(x_end.pos_ned[1] - x_ref)
        E0 = V.sho_energy(case, case.x0, case.v0)
        E1 = V.sho_energy(case, x_end.pos_ned[1], x_end.vel_ned[1])
        dE = abs(E1 - E0) / E0
        @printf("%-6s  %-8s  %-12.3e  %-12.3e\n", name, "adapt", x_err, dE)
        push!(stats, (name, Sim.Integrators.last_stats(integ)))
    end

    for (name, st) in stats
        println("\n$(name) stats: nfev=$(st.nfev), accept=$(st.naccept), reject=$(st.nreject), h_last=$(st.h_last)")
    end
end

main()
