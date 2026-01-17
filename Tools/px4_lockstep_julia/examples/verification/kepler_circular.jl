using PX4Lockstep
using Printf
using LinearAlgebra

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

"""Run the circular Kepler orbit reference problem.

Invariants (energy, angular momentum) and final-state error are checked against the
closed-form circular orbit solution.
"""
function main()
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

    T = 2π  # orbital period for μ=1, R=1
    dt = 0.01
    n = Int(round(T / dt))
    T_end = n * dt

    println("\nCircular Kepler orbit (μ = 1, R = 1, T = 2π)")
    println("================================================")
    @printf("%-6s  %-8s  %-12s  %-12s  %-12s\n", "Solver", "dt", "|r-r*|", "ΔE/E", "Δh/h")

    for (name, integ) in [("Euler", Sim.Integrators.EulerIntegrator()), ("RK4", Sim.Integrators.RK4Integrator())]
        x = x0
        t = 0.0
    E0 = V.kepler_energy(case.μ, x0.pos_ned, x0.vel_ned)
    h0 = V.kepler_angmom(x0.pos_ned, x0.vel_ned)

        for _ = 1:n
            x = Sim.Integrators.step_integrator(integ, f, t, x, nothing, dt)
            t += dt
        end

        r_ref, v_ref, ω = V.kepler_circular_analytic(case, T)
        r_err = norm(x.pos_ned - r_ref)

        E1 = V.kepler_energy(case.μ, x.pos_ned, x.vel_ned)
        dE = abs(E1 - E0) / abs(E0)

        h1 = V.kepler_angmom(x.pos_ned, x.vel_ned)
        dh = norm(h1 - h0) / norm(h0)

        @printf("%-6s  %-8.4g  %-12.3e  %-12.3e  %-12.3e\n", name, dt, r_err, dE, dh)
    end

    adaptive = [
        (
            "RK23",
            Sim.Integrators.RK23Integrator(
                rtol_pos = 1e-10,
                atol_pos = 1e-12,
                rtol_vel = 1e-10,
                atol_vel = 1e-12,
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
                atol_pos = 1e-12,
                rtol_vel = 1e-10,
                atol_vel = 1e-12,
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
        x_end = Sim.Integrators.step_integrator(integ, f, 0.0, x0, nothing, T_end)
        r_ref, v_ref, ω = V.kepler_circular_analytic(case, T_end)
        r_err = norm(x_end.pos_ned - r_ref)
        E0 = V.kepler_energy(case.μ, x0.pos_ned, x0.vel_ned)
        E1 = V.kepler_energy(case.μ, x_end.pos_ned, x_end.vel_ned)
        dE = abs(E1 - E0) / abs(E0)
        h0 = V.kepler_angmom(x0.pos_ned, x0.vel_ned)
        h1 = V.kepler_angmom(x_end.pos_ned, x_end.vel_ned)
        dh = norm(h1 - h0) / norm(h0)
        @printf("%-6s  %-8s  %-12.3e  %-12.3e  %-12.3e\n", name, "adapt", r_err, dE, dh)
        push!(stats, (name, Sim.Integrators.last_stats(integ)))
    end

    for (name, st) in stats
        println("\n$(name) stats: nfev=$(st.nfev), accept=$(st.naccept), reject=$(st.nreject), h_last=$(st.h_last)")
    end
end

main()
