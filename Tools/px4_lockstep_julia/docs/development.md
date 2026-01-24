# Development

## Install dependencies

From the PX4 root:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

## Run tests

The unit tests do not require a PX4 build (they focus on integrators, scheduling, record/replay machinery, and contract checks):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.test()'
```

## Formatting

Format the Julia sources:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JuliaFormatter; format("Tools/px4_lockstep_julia/src")'
```

## Static analysis

Project hygiene checks (Aqua):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Aqua, PX4Lockstep; Aqua.test_all(PX4Lockstep; stale_deps=false)'
```

Static analysis (JET):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JET; JET.report_package("PX4Lockstep"; analyze_from_definitions=false)'
```

## uORB code generation

`PX4Lockstep` talks to PX4 entirely via uORB in the lockstep ABI.

The Julia uORB `struct` definitions are generated from PX4’s generated uORB headers:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/uorb_codegen.jl \
  --headers build/px4_sitl_lockstep/uORB/topics \
  --topics "battery_status,vehicle_attitude" \
  --out Tools/px4_lockstep_julia/src/UORBGenerated.jl
```

Most users should not run this manually; the run helpers (`scripts/run_iris_*.sh`) will regenerate automatically when headers change.

## Adding new things

### New scenario behavior

Scenario logic is the intended place for hybrid events (arm/disarm, gust injections, motor failures, etc.). See:

- `src/sim/Scenario.jl`
- `src/sim/Events.jl`
- `docs/components/scenario-events.md`

### New integrator

Implement a `step_integrator(::YourIntegrator, f, t, x, u, dt)` method and (optionally) a `last_stats` accessor. See `src/sim/Integrators.jl` and `docs/components/integrators.md`.

### New airframe / plant model

The current end-to-end workflow is built around an `AircraftSpec`/builder abstraction.

- Specs: `src/sim/Aircraft/Spec.jl`
- Builder: `src/sim/Aircraft/Build.jl`

If you are adding a new multirotor layout, start with the mapping and validation helpers:

- `src/sim/Aircraft/Validate.jl`
- `test/multirotor_motor_map.jl`

## Determinism checklist

If you’re changing runtime semantics or adding randomness, keep these invariants intact:

- **Integer microsecond timebase** for scheduling and delays (`UInt64`).
- **No RNG in continuous RHS**; randomness only at event boundaries.
- **Sample-and-hold inputs** within each integration interval.
- Avoid hidden mutable side channels; record/replay should be driven by bus signals.

If you touch record/replay semantics, consider bumping `BUS_SCHEMA_VERSION` in `src/sim/Runtime/Bus.jl`.
