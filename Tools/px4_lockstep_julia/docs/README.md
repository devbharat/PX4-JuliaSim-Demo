# Documentation

## Start here

1. **Getting started (build + run):** [`getting-started.md`](getting-started.md)
2. **Workflows (Iris, record/replay, verification, plotting):** [`notes/workflows.md`](notes/workflows.md)
3. **Architecture & invariants:** [`architecture.md`](architecture.md)
4. **Public API entrypoints:** [`reference/api.md`](reference/api.md)

If you specifically care about replay infrastructure: [`notes/record-replay.md`](notes/record-replay.md).

## Reference

- **Frames and sign conventions (NED/FRD, wind, thrust, yaw):** [`reference/conventions.md`](reference/conventions.md)
- **uORB-only lockstep boundary + code generation:** [`reference/uorb.md`](reference/uorb.md)
- **Fault semantics (what faults mean, who consumes what):** [`reference/faults.md`](reference/faults.md)

## Internals

- Component-level notes live under [`components/`](components/).
- Longer-lived design rationale is captured in:
  - [`notes/runtime_engine.md`](notes/runtime_engine.md)
  - [`notes/verification_coverage.md`](notes/verification_coverage.md)
  - [`notes/performance_design_guidelines.md`](notes/performance_design_guidelines.md)
