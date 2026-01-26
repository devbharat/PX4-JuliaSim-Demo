# Improve physics patch (inertia + rotor gyro) — completed

This note records the work that landed full inertia‑tensor support and rotor
gyroscopic coupling. Keeping it as a short historical summary avoids stale TODOs.

## Summary of changes

- Full inertia tensor (including products of inertia) supported in spec and build.
- Rotor gyroscopic coupling added to rigid‑body dynamics.
- Propulsion outputs now include rotor acceleration, used for Ḣ.
- SPD validation added for inertia tensors.
- Tests added to lock in inertia + gyro behavior.
- Docs updated for the new conventions and parameters.

## Verification

- `Tools/px4_lockstep_julia/test/runtests.jl`
