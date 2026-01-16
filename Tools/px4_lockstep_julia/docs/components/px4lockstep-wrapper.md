# PX4Lockstep C ABI Wrapper

## Role

`src/PX4Lockstep.jl` isolates all C ABI interaction. It is intentionally small so ABI
compatibility issues are contained and do not leak into the simulation framework.

## Key Decisions and Rationale

- **ABI handshake on load:** queries ABI version and struct sizes before creating a
  handle, turning layout mismatches into explicit errors instead of silent memory bugs.
- **Handle count guard:** only one handle per process is allowed by default because PX4
  lockstep is not guaranteed re-entrant.
- **Library search fallback:** if `PX4_LOCKSTEP_LIB` is unset, the wrapper checks the
  common PX4 build outputs (`px4_sitl_lockstep`, `px4_sitl_default`).
- **Symbol + output caching:** the wrapper caches resolved symbols and reuses a
  per-handle output buffer to avoid repeated allocations on every step.

## Integration Contracts

- `LockstepInputs`, `LockstepOutputs`, and `LockstepConfig` must match the C layout.
- `time_us` passed to PX4 is authoritative; upstream code must supply microsecond
  quantized time.

## Extension Notes

The wrapper should remain narrow. New simulation features belong in `PX4Lockstep.Sim`,
not the ABI boundary.

## Caveats

- Only one lockstep handle is allowed per process by default; enabling multiple handles
  is opt-in and unsafe unless PX4 is known to be re-entrant.
- Library discovery only checks common PX4 build outputs; custom paths should set
  `PX4_LOCKSTEP_LIB` explicitly.
- The wrapper does not serialize concurrent `step!` calls; callers must avoid parallel
  access to a single handle.
