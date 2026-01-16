#!/usr/bin/env julia

"""Reference comparison demo: `PlantState` (SHO + rotor omega + battery states).

This example exercises the *full-plant* variable-step integration surface area without
pulling in PX4 or the full vehicle/propulsion models.

It embeds a simple harmonic oscillator (SHO) into the rigid-body subset (`rb.pos_ned[1]`,
`rb.vel_ned[1]`) and adds additional continuous states:

* `rotor_ω` follows a fast first-order stable ODE
* battery SOC and Thevenin V1 follow simple linear dynamics

Procedure:
1. Generate a high-accuracy RK45 *reference* trajectory on a fine grid.
2. Downsample the reference to the solver's output grid.
3. Run a candidate solver (e.g. RK4) on the coarse grid.
4. Compare error vs time (including rotor omega + SOC/V1) and invariant drift vs time.

Run:

    julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_plant_sho_rotor_battery.jl

This script writes a CSV file next to the script for quick plotting.
"""

using PX4Lockstep
using StaticArrays

const Sim = PX4Lockstep.Sim
const V = Sim.Verification

# Rigid-body subset: SHO
sho = V.SHOCase(ω0 = 2 * pi, x0 = 1.0, v0 = 0.0)
f_rb = V.sho_rhs(sho)

# Additional plant states: rotor omega and a toy battery
omega_target = SVector{4,Float64}(400.0, 420.0, 410.0, 430.0)
alpha = 200.0          # (1/s) rotor convergence rate (fast)
soc_dot = -0.01    # per second
tau_v1 = 0.2         # seconds

function f(t::Float64, x::Sim.Plant.PlantState{4}, _u)
    rb_dot = f_rb(t, x.rb, nothing)
    rotor_dot = -alpha .* (x.rotor_ω - omega_target)
    return Sim.Plant.PlantDeriv{4}(
        rb = rb_dot,
        rotor_ω_dot = rotor_dot,
        batt_soc_dot = soc_dot,
        batt_v1_dot = -x.batt_v1 / tau_v1,
    )
end

rb0 = Sim.RigidBody.RigidBodyState(
    pos_ned = Sim.Types.vec3(sho.x0, 0.0, 0.0),
    vel_ned = Sim.Types.vec3(sho.v0, 0.0, 0.0),
    q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
    ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
)

x0 = Sim.Plant.PlantState{4}(
    rb = rb0,
    rotor_ω = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
    batt_soc = 1.0,
    batt_v1 = 1.0
)

# Output grid for the *tested* solver.
# Use 250 Hz to mirror PX4 lockstep cadence.
dt_sol = 0.004
t_end = 2.0

# Reference output grid must divide dt_sol exactly for deterministic downsampling.
dt_ref = dt_sol / 10

# Plant-aware error control should be enabled for a high-quality *plant* reference,
# otherwise RK45 adaptivity may ignore rotor/battery accuracy.
ref_fine = V.rk45_reference(
    f,
    x0,
    dt_ref,
    t_end;
    plant_error_control = true,
    atol_rotor = 1e-10,
    atol_soc = 1e-12,
    atol_v1 = 1e-12,
)
ref = V.resample_trajectory(ref_fine, dt_sol)

rk4 = Sim.Integrators.RK4Integrator()
sol = V.simulate_trajectory(rk4, f, x0, dt_sol, t_end)

invfun = (s::Sim.Plant.PlantState{4},) -> (
    E_sho = V.sho_energy(sho, s.rb.pos_ned[1], s.rb.vel_ned[1]),
)

cmp = V.compare_to_reference(ref, sol; invfun = invfun)

println("PlantState reference compare (SHO + rotor omega + battery)")
println("  dt_ref = $(dt_ref) s (RK45 reference, plant-aware)")
println("  dt_sol = $(dt_sol) s (RK4 test)")
println("  t_end  = $(t_end) s")
println()

println("Max error vs reference:")
println("  pos max      = $(cmp.err.max.pos) m")
println("  vel max      = $(cmp.err.max.vel) m/s")
println("  rotor omega max  = $(cmp.err.max.rotor) rad/s")
println("  SOC max      = $(cmp.err.max.soc)")
println("  V1 max       = $(cmp.err.max.v1) V")

if cmp.invariants !== nothing
    drift_ref = cmp.invariants.ref.drift.E_sho
    drift_sol = cmp.invariants.sol.drift.E_sho
    println()
    println("SHO energy drift:")
    println("  reference: max |ΔE| = $(maximum(abs.(drift_ref)))")
    println("  RK4:       max |ΔE| = $(maximum(abs.(drift_sol)))")
end

# Write CSV for plotting.
csv_path = joinpath(@__DIR__, "plant_sho_rotor_battery_reference_compare.csv")
open(csv_path, "w") do io
    println(io, "t,pos_err,vel_err,att_err_rad,omega_err,rotor_err,soc_err,v1_err,sho_energy_drift_ref,sho_energy_drift_sol")
    t = cmp.err.t
    drift_ref = cmp.invariants === nothing ? fill(NaN, length(t)) : cmp.invariants.ref.drift.E_sho
    drift_sol = cmp.invariants === nothing ? fill(NaN, length(t)) : cmp.invariants.sol.drift.E_sho
    for i in eachindex(t)
        println(io, "$(t[i]),$(cmp.err.pos_err[i]),$(cmp.err.vel_err[i]),$(cmp.err.att_err_rad[i]),$(cmp.err.ω_err[i]),$(cmp.err.rotor_err[i]),$(cmp.err.soc_err[i]),$(cmp.err.v1_err[i]),$(drift_ref[i]),$(drift_sol[i])")
    end
end

println()
println("Wrote CSV: $(csv_path)")
