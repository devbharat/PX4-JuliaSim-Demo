using PX4Lockstep
using Printf
using LinearAlgebra

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

"""Run torque-free rigid-body rotation verification.

This checks conserved quantities:
* rotational kinetic energy
* inertial angular momentum vector
and reports quaternion norm drift.
"""
function main()
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

    println("\nTorque-free rigid body (diagonal inertia)")
    println("=======================================")
    @printf("%-6s  %-8s  %-12s  %-12s  %-12s\n", "Solver", "dt", "ΔE/E", "|ΔL|", "|q|-1")

    for (name, integ) in [("Euler", Sim.Integrators.EulerIntegrator()), ("RK4", Sim.Integrators.RK4Integrator())]
        x = x0
        t = 0.0
        E0 = V.rigidbody_rot_energy(case.I_body, x0.ω_body)
        L0 = V.rigidbody_angmom_ned(x0.q_bn, case.I_body, x0.ω_body)

        for _ = 1:n
            x = Sim.Integrators.step_integrator(integ, f, t, x, nothing, dt)
            t += dt
        end

        E1 = V.rigidbody_rot_energy(case.I_body, x.ω_body)
        dE = abs(E1 - E0) / E0

        L1 = V.rigidbody_angmom_ned(x.q_bn, case.I_body, x.ω_body)
        dL = norm(L1 - L0)

        qn = abs(norm(x.q_bn) - 1.0)

        @printf("%-6s  %-8.4g  %-12.3e  %-12.3e  %-12.3e\n", name, dt, dE, dL, qn)
    end

    adaptive = [
        (
            "RK23",
            Sim.Integrators.RK23Integrator(
                rtol_pos = 1e-12,
                atol_pos = 1e-12,
                rtol_vel = 1e-12,
                atol_vel = 1e-12,
                rtol_ω = 1e-10,
                atol_ω = 1e-12,
                atol_att_rad = 1e-12,
                h_min = 1e-6,
                h_max = 0.02,
            ),
        ),
        (
            "RK45",
            Sim.Integrators.RK45Integrator(
                rtol_pos = 1e-12,
                atol_pos = 1e-12,
                rtol_vel = 1e-12,
                atol_vel = 1e-12,
                rtol_ω = 1e-10,
                atol_ω = 1e-12,
                atol_att_rad = 1e-12,
                h_min = 1e-6,
                h_max = 0.02,
            ),
        ),
    ]

    stats = Tuple{String,Sim.Integrators.IntegratorStats}[]
    for (name, integ) in adaptive
        x_end = Sim.Integrators.step_integrator(integ, f, 0.0, x0, nothing, T)
        E0 = V.rigidbody_rot_energy(case.I_body, x0.ω_body)
        E1 = V.rigidbody_rot_energy(case.I_body, x_end.ω_body)
        dE = abs(E1 - E0) / E0
        L0 = V.rigidbody_angmom_ned(x0.q_bn, case.I_body, x0.ω_body)
        L1 = V.rigidbody_angmom_ned(x_end.q_bn, case.I_body, x_end.ω_body)
        dL = norm(L1 - L0)
        qn = abs(norm(x_end.q_bn) - 1.0)
        @printf("%-6s  %-8s  %-12.3e  %-12.3e  %-12.3e\n", name, "adapt", dE, dL, qn)
        push!(stats, (name, Sim.Integrators.last_stats(integ)))
    end

    for (name, st) in stats
        println("\n$(name) stats: nfev=$(st.nfev), accept=$(st.naccept), reject=$(st.nreject), h_last=$(st.h_last)")
    end
end

main()
