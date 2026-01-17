# Record/Replay Examples

These examples exercise the unified simulation runtime:

- `PX4Lockstep.Sim.Runtime` (one canonical engine)
- `PX4Lockstep.Sim.Recording` (Tier-0 recorder + traces)
- `PX4Lockstep.Sim.Sources` (live + replay sources)
- `PX4Lockstep.Sim.Workflows` (convenience wrappers)

## Quick start: Iris integrator comparison (recommended)

This is the streamlined workflow wrapper:

- run a short PX4 lockstep mission **live**
- **record** Tier-0 streams
- **replay** the exact same inputs while sweeping plant integrators
- write CSV summaries under `examples/replay/out/`

```bash
PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib) \
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia -O3 Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
```

To emit per-solver replay logs for plotting, set `IRIS_LOG_DIR` (and optionally
`IRIS_LOG_PREFIX`). This will create CSV logs named like
`<prefix>_ref_log.csv` and `<prefix>_<solver>_log.csv`.

## Determinism check (same integrator repeated)

This workflow replays the same Tier-0 recording multiple times with the same
integrator to verify deterministic replay. Set `IRIS_DETERMINISM_SOLVER` and
`IRIS_DETERMINISM_N` to control which integrator and how many repeats.

```bash
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
IRIS_DETERMINISM_SOLVER=RK4 IRIS_DETERMINISM_N=3 \
IRIS_LOG_DIR=Tools/px4_lockstep_julia/examples/replay/out \
  julia --project=Tools/px4_lockstep_julia \
    Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl
```

## Minimal deterministic demo (no PX4)

- `minimal_record_replay_demo.jl` demonstrates the core timeline + traces + replay loop.

## Where the real logic lives

The example scripts are intentionally thin. The canonical Iris workflow lives in:

- `PX4Lockstep.Sim.Workflows.simulate_iris_mission(...)`
- `PX4Lockstep.Sim.Workflows.compare_integrators_iris_mission(...)`

See:

- `docs/engine_unification.md`
- `docs/components/record-replay.md`
