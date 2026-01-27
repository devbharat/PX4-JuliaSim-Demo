# uORB-Only Lockstep Interface

## Current state

The lockstep bridge is now **uORB-only**. Legacy `px4_lockstep_inputs_t` and
`px4_lockstep_outputs_t` have been removed along with `px4_lockstep_step()`.
The only stepping entrypoint is `px4_lockstep_step_uorb(handle, time_us)`.

Key points:

- **ABI v3**: `px4_lockstep_sizes` reports `in_sz=0`, `out_sz=0`, and the
  `px4_lockstep_config_t` size.
- **Inputs**: Julia publishes uORB messages via the generic publish API.
- **Outputs**: Julia subscribes to uORB outputs and builds `UORBOutputs`.
- **Mission loading**: still uses `px4_lockstep_load_mission_qgc_wpl`.

## Julia expectations

- `UORBGenerated.jl` must match the uORB headers from the active PX4 build.
- `PX4Lockstep.Sim.Autopilots.PX4LockstepAutopilot` always uses uORB outputs.
- `step_uorb!` is the only step call in the wrapper.

## Interface configuration

The uORB boundary is configured in TOML via `[px4.uorb]` with explicit `pubs` and
`subs`. For reuse, include a shared file with `extends` (see
`src/Workflows/assets/aircraft/iris_uorb.toml` and
`src/Workflows/assets/aircraft/minimal_uorb.toml`).
There are no code-side presets; TOML is the only source of uORB configuration.

Determinism note: uORB publisher `instance` defaults to auto (`-1`) when omitted. For
reproducible wiring, prefer explicit instances (e.g., `instance = 0` for singletons).

## Injection scheduling

The bridge currently publishes **all configured injection topics every autopilot tick**
(period = 0). This keeps behavior simple but means per-tick uORB traffic grows as
you add topics. The per-topic period machinery exists internally but is not exposed
via TOML yet. For latched topics (e.g., `home_position`), a “publish once” option would
be more efficient.

## Where the bridge lives

- ABI wrapper: `src/PX4Lockstep.jl`
- Topic registry + helpers: `src/sim/Autopilots/UORBBridge.jl`
- Autopilot integration: `src/sim/Autopilots.jl`

## Julia uORB type generation

Julia structs are generated from PX4’s generated uORB headers:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/uorb_codegen.jl \
  --headers build/px4_sitl_lockstep/uORB/topics \
  --topics "battery_status,vehicle_attitude" \
  --out Tools/px4_lockstep_julia/src/UORBGenerated.jl
```

In normal use you should not run this manually: the run helpers (`scripts/run_iris_*.sh`) regenerate `UORBGenerated.jl` automatically when headers change.

## Common failure modes

### Size/layout mismatch

If you see an error from `verify_uorb_type!` / `verify_uorb_contract!`, it usually means one of:

- your PX4 build regenerated uORB headers and `src/UORBGenerated.jl` is stale
- you are pointing at headers from a different PX4 build than the `libpx4_lockstep` you loaded

Fix: rebuild PX4, then re-run one of the helper scripts so codegen re-triggers.
