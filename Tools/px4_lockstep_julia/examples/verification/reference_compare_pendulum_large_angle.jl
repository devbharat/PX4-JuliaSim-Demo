#!/usr/bin/env julia

"""Reference comparison demo: nonlinear pendulum (large angle).

This example demonstrates the verification workflow for cases without convenient
analytic solutions:

1. Generate a high-accuracy RK45 *reference* trajectory on a fine grid.
2. Downsample the reference to the solver's output grid.
3. Run a candidate solver (e.g. RK4) on the coarse grid.
4. Compare error vs time and invariant drift vs time.

Run:

    julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_pendulum_large_angle.jl

This script writes a CSV file next to the script for quick plotting.
"""

using PX4Lockstep
using Printf

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

# Large-angle pendulum (nonlinear); no closed-form time history unless using elliptic integrals.
case = V.PendulumCase(g = 9.80665, L = 1.0, θ0 = 1.0, θdot0 = 0.0)
f = V.pendulum_rhs(case)

x0 = Sim.RigidBody.RigidBodyState(
    pos_ned = Sim.Types.vec3(case.θ0, 0.0, 0.0),
    vel_ned = Sim.Types.vec3(case.θdot0, 0.0, 0.0),
    q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
    ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
)

# Output grid for the *tested* solver.
dt_sol = 0.02
t_end = 5.0

# Reference output grid must divide dt_sol exactly for deterministic downsampling.
dt_ref = dt_sol / 10

ref_fine = V.rk45_reference(f, x0, dt_ref, t_end)
ref = V.resample_trajectory(ref_fine, dt_sol)

invfun = (s::Sim.RigidBody.RigidBodyState,) -> (
    E = V.pendulum_energy(case, s.pos_ned[1], s.vel_ned[1]),
)
solvers = [
    ("RK4", Sim.Integrators.RK4Integrator()),
    ("RK23", Sim.Integrators.RK23Integrator()),
    ("RK45", Sim.Integrators.RK45Integrator()),
]

println("Nonlinear pendulum reference compare (large angle)")
println("  dt_ref = $(dt_ref) s (RK45 reference)")
println("  dt_sol = $(dt_sol) s (solver output grid)")
println("  t_end  = $(t_end) s")
println()
@printf("%-6s  %-14s  %-16s  %-14s\n", "Solver", "|θ-θ_ref| max", "|θdot-θdot_ref|", "max |ΔE|")

for (label, integ) in solvers
    sol = V.simulate_trajectory(integ, f, x0, dt_sol, t_end)
    cmp = V.compare_to_reference(ref, sol; invfun = invfun)

    drift_ref = cmp.invariants === nothing ? fill(NaN, length(cmp.err.t)) : cmp.invariants.ref.drift.E
    drift_sol = cmp.invariants === nothing ? fill(NaN, length(cmp.err.t)) : cmp.invariants.sol.drift.E
    max_drift = maximum(abs.(drift_sol))
    @printf("%-6s  %-14.6e  %-16.6e  %-14.6e\n", label, cmp.err.max.pos, cmp.err.max.vel, max_drift)

    csv_path = joinpath(@__DIR__, "pendulum_large_angle_reference_compare_" * lowercase(label) * ".csv")
    open(csv_path, "w") do io
        println(io, "t,theta_err,theta_dot_err,energy_drift_ref,energy_drift_sol")
        t = cmp.err.t
        θ_err = cmp.err.pos_err
        θdot_err = cmp.err.vel_err
        drift_ref = cmp.invariants === nothing ? fill(NaN, length(t)) : cmp.invariants.ref.drift.E
        drift_sol = cmp.invariants === nothing ? fill(NaN, length(t)) : cmp.invariants.sol.drift.E
        for i in eachindex(t)
            println(io, "$(t[i]),$(θ_err[i]),$(θdot_err[i]),$(drift_ref[i]),$(drift_sol[i])")
        end
    end
    println("  Wrote CSV: $(csv_path)")
end
