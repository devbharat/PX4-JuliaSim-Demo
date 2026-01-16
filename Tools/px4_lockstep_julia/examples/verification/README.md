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



## Iris mission closed-loop integrator comparison (PX4 lockstep + PlantSimulation)

If you have the PX4 lockstep library available, you can do a **system-level**
integrator sensitivity comparison on the Iris mission use case:

```bash
PX4_LOCKSTEP_LIB=... PX4_LOCKSTEP_MISSION=... \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_iris_mission_plantsim.jl
```

`PX4_LOCKSTEP_LIB` is optional; if it is unset, the script falls back to the default
build tree search used by `PX4Lockstep.find_library()`.

You can choose the test solver via:

* `IRIS_TEST_SOLVER=RK4`
* `IRIS_TEST_SOLVER=RK23`
* `IRIS_TEST_SOLVER=RK45`

This is closed-loop, so trajectories may diverge over long horizons; treat it as a
"mission sensitivity to solver choice" metric, not a pure plant truncation error study.


Notes:

* These scripts are written to run quickly and print a small metrics table.
* The associated unit tests live in `test/verification_cases.jl`.
