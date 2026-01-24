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

The uORB boundary is configured explicitly via `PX4UORBInterfaceConfig` presets in
`PX4Lockstep.Sim.Autopilots` (for example, `iris_state_injection_interface`). Use these
configs to select which topics are published/subscribed instead of env flags.

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
