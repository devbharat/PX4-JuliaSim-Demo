# uORB-Only Lockstep Migration Plan

## Current status (as of latest changes)

- Generic uORB pub/sub API is implemented in `libpx4_lockstep` and exposed in Julia.
- Opt-in env flags switch selected topics from legacy `publish_inputs()` to uORB:
  - `battery_status`, `vehicle_attitude`, `vehicle_local_position`,
    `vehicle_global_position`, `vehicle_angular_velocity`, `vehicle_land_detected`.
- Julia publishes those topics via queued uORB messages during `autopilot_step()`.
- Legacy `px4_lockstep_inputs_t` and `px4_lockstep_outputs_t` are still the primary ABI,
  and are required to drive PX4 and return actuator outputs.
- A Julia uORB struct generator exists: `scripts/uorb_codegen.jl`, with generated
  structs in `src/UORBGenerated.jl` and included from `src/PX4Lockstep.jl`.
- Publisher creation validates Julia struct sizes against uORB metadata at init.
- C++ uORB topic metadata lookup uses `uORB/topics/uORBTopics.hpp` and `orb_get_topics()`.
- uORB priority/queue features are ignored on this PX4 build (no `ORB_PRIO_DEFAULT` or
  `orb_advertise_multi_queue` in headers); the API accepts values but drops them.

## Remaining work (high-level)

- Replace all remaining legacy input injections with uORB publishes.
- Replace legacy outputs struct with uORB subscriptions for outputs.
- Introduce a new ABI version for uORB-only mode and deprecate the old structs.
- Update Julia APIs and workflows to use the new ABI path by default.
- Add robust layout/size validation for Julia uORB structs.
- Update docs, examples, and migration guidance.

## Detailed migration plan (fresh-session TODOs)

### Phase 0 — Prep and safety rails (complete)

- [x] **Inventory remaining legacy inputs**
  - `time_us` → PX4 timebase only (no uORB equivalent).
  - `armed`, `nav_auto_mission`, `nav_auto_rtl` → `vehicle_command` (Commander enabled)
    or `vehicle_status` + `vehicle_control_mode` + `actuator_armed` (Commander disabled).
  - `landed` → `vehicle_land_detected`.
  - `x/y/z`, `vx/vy/vz`, `yaw` → `vehicle_local_position`.
  - `lat_deg/lon_deg/alt_msl_m` → `vehicle_global_position` + local position ref.
  - `q` → `vehicle_attitude`.
  - `rates_xyz` → `vehicle_angular_velocity`.
  - `battery_*` → `battery_status`.
- [x] **Inventory legacy outputs**
  - `actuator_controls` → `vehicle_torque_setpoint` + `vehicle_thrust_setpoint`.
  - `actuator_motors` → `actuator_motors`.
  - `actuator_servos` → `actuator_servos`.
  - `attitude_setpoint_q`, `thrust_setpoint_body` → `vehicle_attitude_setpoint`.
  - `rates_setpoint_xyz` → `vehicle_rates_setpoint`.
  - `mission_seq`, `mission_count`, `mission_finished` → `mission_result`.
  - `nav_state`, `arming_state` → `vehicle_status`.
  - `battery_warning` → `battery_status`.
  - `trajectory_setpoint_*` → `trajectory_setpoint`.
- [x] **Add uORB layout validation**
  - `create_uorb_publisher_checked` compares `sizeof(T)` to the uORB message size at init.

### Phase 1 — Finish uORB input migration

- [ ] **Commander replacement topics**
  - Publish `vehicle_status`, `vehicle_control_mode`, `actuator_armed`,
    `home_position`, and `geofence_status` via uORB when Commander is disabled.
  - Add opt-in flags to skip legacy publishes in `publish_inputs()`.
- [ ] **Vehicle command requests**
  - For Commander-enabled runs, publish `vehicle_command` via uORB
    (edge-triggered) and disable the legacy direct publish path.
- [ ] **Remove remaining input fields**
  - Once all topics are migrated, stop filling unused fields in
    `px4_lockstep_inputs_t` when uORB-only mode is active.

### Phase 2 — uORB-only output pipeline

- [ ] **Create uORB subscriptions for outputs in Julia**
  - Add a `UORBOutputs` struct mirroring the old outputs (motors, servos,
    setpoints, mission status, nav/arming state, battery warning).
  - Use `uorb_check` / `uorb_copy` to sample updates each tick (or on log axis).
- [ ] **Define output sampling semantics**
  - Decide whether to sample outputs at every autopilot tick or only when updated.
  - Preserve sample-and-hold behavior to match current outputs struct semantics.
- [ ] **Plumb into `Sim.Sources.Autopilot`**
  - Replace `LockstepOutputs` with the uORB output container in the Julia
    autopilot source while keeping a compatibility adapter.

### Phase 3 — ABI versioning and uORB-only entrypoint

- [x] **Introduce ABI v2**
  - Added `px4_lockstep_step_uorb(handle, time_us)` and bumped the ABI to v2.
  - Julia accepts ABI v1 or v2 for compatibility.
- [x] **Dual-mode creation**
  - `PX4_LOCKSTEP_UORB_ONLY=1` selects the uORB-only step path.
  - When uORB-only is active, `px4_lockstep_step` rejects legacy inputs.

### Phase 4 — Deprecation and cleanup

- [ ] **Deprecate legacy structs**
  - Mark `px4_lockstep_inputs_t` / `px4_lockstep_outputs_t` as legacy in headers.
  - Update Julia wrapper to prefer uORB-only functions and warn on legacy usage.
- [ ] **Remove `publish_inputs()` usage**
  - Keep it only for ABI v1 builds or guard with compile-time flags.
- [ ] **Remove legacy output buffer**
  - Stop caching `px4_lockstep_outputs_t` when uORB-only output subscriptions
    are enabled.

### Phase 5 — Documentation and examples

- [ ] **Update API docs**
  - Add uORB-only usage patterns to `docs/API.md` and wrapper docs.
- [ ] **Update examples**
  - Provide a uORB-only Iris example and a backward-compatible example.
- [ ] **Migration guide**
  - Write a short guide describing which env flags/configs to toggle and
    how to validate layout compatibility.

## Suggested sequence of PRs

1. Input migration completion + size validation
2. Output pipeline via uORB subscriptions + Julia adapter
3. ABI v2 entrypoint + uORB-only mode
4. Deprecation notices + docs/examples update
