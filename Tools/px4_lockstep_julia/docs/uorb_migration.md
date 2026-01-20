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

## Environment flags

Input topic publishers are enabled by default. Set env flags to `0` to disable
specific publishers (e.g. `PX4_LOCKSTEP_UORB_LOCAL_POSITION=0`). The defaults in
`Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh` keep everything enabled.
