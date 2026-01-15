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

Notes:

* These scripts are written to run quickly and print a small metrics table.
* The associated unit tests live in `test/verification_cases.jl`.
