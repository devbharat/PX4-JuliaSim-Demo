# Verification examples

These scripts are **deterministic reference problems** intended to sanity-check the
numerical behavior of the integrators and the underlying math used by
`PX4Lockstep.Sim`.

They deliberately avoid PX4 and the full vehicle model; the point is to measure
solver **accuracy and stability** against analytic solutions and/or conserved
quantities (invariants).

Run any script from the repo root, for example:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/sho.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/pendulum.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/kepler_circular.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/torque_free_rigid_body.jl
```

## Reference-trajectory comparison (no analytic solution required)

For problems without a convenient analytic time history, you can generate a
high-accuracy RK45 **reference** trajectory and compare other solvers against it:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_pendulum_large_angle.jl
```

This uses:

* `Sim.Verification.rk45_reference(...)` to generate the reference
* `Sim.Verification.resample_trajectory(...)` for deterministic downsampling
* `Sim.Verification.compare_to_reference(...)` to compute error vs time and
  invariant drift vs time

The script also writes a small CSV file for quick plotting.


## PlantState reference-trajectory comparison (full-plant continuous states)

If you want to exercise adaptive stepping on **non-rigid-body** continuous states
(e.g., rotor omega and battery SOC/V1) without pulling PX4 into the loop, run:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_plant_sho_rotor_battery.jl
```

This uses the same RK45 reference machinery but compares `PlantState` trajectories
(RB + rotor omega + SOC/V1).



## Iris integrator comparison (PX4 lockstep + record→replay)

Closed-loop "run it three times" comparisons are often misleading once trajectories
diverge and PX4 issues different commands.

The recommended workflow is to **record** one baseline PX4 run (commands, wind samples,
scenario outputs including faults) and then **replay** that recording open-loop while
sweeping plant integrators.

```bash
PX4_LOCKSTEP_LIB=... PX4_LOCKSTEP_MISSION=... \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
```

See the script header for environment variables such as:

* `IRIS_T_END_S`
* `IRIS_SWEEP_SOLVERS` (default: "RK4,RK23,RK45")

The summary CSV is written under `examples/replay/out/`.


Notes:

* These scripts are written to run quickly and print a small metrics table.
* The associated unit tests live in `test/verification_cases.jl`.
