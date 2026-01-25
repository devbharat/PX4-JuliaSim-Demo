# Record/Replay Examples

These examples exercise the unified simulation runtime:

- `PX4Lockstep.Sim.Runtime` (one canonical engine)
- `PX4Lockstep.Sim.Recording` (Tier-0 recorder + traces)
- `PX4Lockstep.Sim.Sources` (live + replay sources)
- `PX4Lockstep.Workflows` (convenience wrappers)

## Quick start: Iris integrator comparison (recommended)

This is the streamlined workflow wrapper:

- run a short PX4 lockstep mission **live**
- **record** Tier-0 streams
- **replay** the exact same inputs while sweeping plant integrators
- write CSV summaries under `examples/replay/out/`

```bash
julia --project=Tools/px4_lockstep_julia -O3 \
  Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl \
  /path/to/spec.toml
```

Your spec must include `px4.libpath` (and `px4.mission_path`) to record a live run.

To emit per-solver replay logs for plotting, pass `log_dir` to
`compare_integrators_iris_mission(...)` (see `examples/replay/iris_integrator_compare.jl`).

## Determinism check (same integrator repeated)

This workflow replays the same Tier-0 recording multiple times with the same
integrator to verify deterministic replay. Pass solver + repeat count explicitly:

```bash
julia --project=Tools/px4_lockstep_julia \
  Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl \
  RK4 3 /path/to/spec.toml
```

## Minimal deterministic demo (no PX4)

- `minimal_record_replay_demo.jl` demonstrates the core timeline + traces + replay loop.

## Where the real logic lives

The example scripts are intentionally thin. The canonical Iris workflow lives in:

- `PX4Lockstep.Workflows.simulate_iris_mission(...)`
- `PX4Lockstep.Workflows.compare_integrators_iris_mission(...)`

See:

- `docs/engine_unification.md`
- `docs/components/record-replay.md`
