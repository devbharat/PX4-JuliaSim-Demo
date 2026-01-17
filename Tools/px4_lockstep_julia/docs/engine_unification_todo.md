# Engine Unification TODO

This file tracks remaining integration work to reach a clean “mergeable” state.

## Completed

- ✅ Unified record + replay onto `Sim.Runtime.Engine` (single canonical runtime)
- ✅ “Tier-0” recording for record/replay via `Sim.Recording.InMemoryRecorder`
- ✅ Iris mission wrappers (`Sim.Workflows.simulate_iris_mission`, `...compare_integrators_iris_mission`)
- ✅ Removed duplicated Iris example configuration (`examples/replay/iris_common.jl`) and simplified examples
- ✅ Integrated **structured log sinks** into the runtime (CSV sink now; HDF5 sink later)
- ✅ Added workflow-level test coverage for `Workflows.compare_integrators_recording`

## Next

### Schema/compat checks

- [ ] Add a `schema_version::UInt32` to `Recording.Tier0Recording` and verify it on load.
- [ ] Provide a `Recording.validate(recording; strict=true)` helper that checks:
  - required streams exist and are axis-aligned
  - monotonicity and shape constraints
  - optional scenario streams depending on flags

### Stage extensibility

- [ ] Decide how to support additional boundary hooks (sensors, custom telemetry) without modifying `Runtime.Engine`.
  - Likely: "hooks" vector with (stage, axis) metadata; executed deterministically.

### Logging sinks

- [x] CSV sink wired into `Runtime.Engine` via `log_sinks` kw
- [ ] Add an `HDF5LogSink` mirroring the CSV sink interface (same call signature)
- [ ] Add a small unit test for CSV sink formatting + row count on a tiny sim

### Workflows and examples

- [ ] Add a “no PX4 required” tutorial workflow for synthetic plant comparisons
- [ ] Ensure all example scripts use `Sim.Workflows` wrappers (avoid copying defaults)

### Performance and determinism

- [ ] Audit allocations in the per-boundary hotpath (especially stage dispatch + trace sampling)
- [ ] Decide whether adaptive integrators should be reset per interval or allowed to carry state across intervals

## Refactor review plan (audit)

- [ ] Inventory exported APIs and ensure only `Sim.Runtime.Engine` is user-facing
- [ ] Verify boundary ordering matches the canonical stage list (`Runtime.BoundaryProtocol`)
- [ ] Validate `Runtime.Timeline` + `Scheduler` axis membership and `dt_phys` semantics
- [ ] Confirm plant protocol coverage (`plant_outputs`, `plant_project`, `plant_on_autopilot_tick`)
- [ ] Review `Sim.Sources` for correct bus updates, RNG isolation, and fault handling
- [ ] Review `Sim.Recording` for stream naming, trace semantics, and schema enforcement
- [ ] Verify logging sinks are deterministic and close/flush on `run!`
- [ ] Validate workflow helpers + examples for parity with removed scripts
- [ ] Audit tests for coverage of scenario boundaries, actuator snap, and schema mismatch
