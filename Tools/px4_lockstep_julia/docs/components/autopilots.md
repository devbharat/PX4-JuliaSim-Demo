# Autopilot Bridge

## Role

`src/sim/Autopilots.jl` provides the PX4-facing autopilot implementation used by the
simulation runtime.

In a live run it:

- publishes estimated state + auxiliary inputs (battery, landed, commands) into PX4 via
  the lockstep uORB ABI
- steps PX4 with an exact microsecond timestamp
- samples uORB outputs (actuator commands, nav state, setpoints) for downstream use

The uORB plumbing (topic specifications, publisher/subscriber wiring, message builders,
and optional injection scheduling) lives in:

- `src/sim/Autopilots/UORBBridge.jl`
- `src/sim/Autopilots/UORBInjection.jl`

## Key decisions and rationale

- **Microsecond entry point:** `autopilot_step(ap, time_us, ...)` uses `UInt64`
  microseconds so PX4 time injection is exact and repeatable.
- **Explicit home/origin:** `HomeLocation` (an alias of `Sim.Types.WorldOrigin`) is the
  single source of truth for NED↔LLA conversions and home-related uORB messages.
- **Tick‑rate‑independent command semantics:** `edge_trigger=true` converts mission/RTL
  requests into one‑tick pulses so behavior does not depend on the autopilot tick rate.
- **Scheduling validation hooks:** `max_internal_rate_hz`, `recommended_step_dt_us`, and
  `injection_periods_us` allow the runtime to validate that the chosen `timeline.ap`
  cadence is compatible with internal PX4 module rates and configured uORB injections.

## Integration contracts

- `autopilot_step` is called at `timeline.ap` boundaries with the scheduler’s integer
  `time_us` (no float time conversion on the call boundary).
- Inputs are sourced from the runtime bus (`SimBus`): estimated state (`bus.est`),
  `bus.ap_cmd`, `bus.landed`, and `bus.batteries`.
- Outputs are treated as sample-and-hold until the next autopilot tick.
- Home/reference fields:
  - When `LockstepConfig.enable_commander == 0`, the bridge publishes `home_position`
    and uses `home` for the `vehicle_global_position.ref_*` fields.
  - When commander is enabled, `ref_*` fields follow the current LLA computed from the
    provided NED state.

## Caveats

- The NED↔LLA conversion is a spherical-Earth approximation intended for local missions
  (small geographic extent).
- With `edge_trigger=true`, mission/RTL requests become pulses; callers that want a
  sustained request must reassert it.
- Only one lockstep handle is allowed per process by default (unless
  `allow_multiple_handles=true` is used and the underlying lockstep runtime is known to
  be re-entrant).
