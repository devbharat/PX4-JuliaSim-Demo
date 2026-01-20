# Autopilot Bridge

## Role

`src/sim/Autopilots.jl` translates estimated state + battery status into PX4 lockstep
inputs and provides the interface used by the simulation engines.

The uORB bridge helpers (topic specs, pub/sub wiring, and message builders) live in
`src/sim/Autopilots/UORBBridge.jl`.

## Key Decisions and Rationale

- **Microsecond entry point:** `autopilot_step(::UInt64, ...)` avoids float rounding
  and guarantees exact PX4 time injection.
- **Shared `WorldOrigin`:** the PX4 home location is the same `WorldOrigin` used by the
  environment, keeping NED↔LLA conversions consistent (spherical Earth approximation).
- **Edge-triggered commands:** optional `edge_trigger` converts mission/RTL requests
  into pulses so their semantics do not depend on the tick rate.
- **Rate guard hook:** `max_internal_rate_hz` exposes internal PX4 task cadence so
  engines can validate `dt_autopilot`.

## Integration Contracts

- `autopilot_step` should be called with microsecond-quantized `time_us` in lockstep.
- Autopilot outputs are sample-and-hold until the next autopilot tick.
- Home/origin mismatches are synchronized when one side is default; otherwise a
  warning is emitted.

## Caveats

- The NED↔LLA conversion assumes a spherical Earth and is suitable only for local
  missions.
- `edge_trigger=true` converts mission/RTL requests into pulses; commands must be
  reasserted if the caller expects a sustained request.
- Only one lockstep handle is allowed per process by default.
