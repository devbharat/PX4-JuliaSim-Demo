# Improve physics patch (inertia + rotor gyro) — TODOs

## Context
- Patch file: `px4_lockstep_julia_inertia_gyro.patch` (applied locally on `feat/improv_phy`).
- Goals: add off-diagonal inertia support and rotor gyroscopic coupling.
- Current main branch: TOML-first, multirotor-only sim; internal default spec at `src/sim/Aircraft/assets/multirotor_default.toml`.

## Findings from patch review
### Blocker
- [x] Fix invalid method signature in `rotor_inertia_kgm2`:
   - Current: `rotor_inertia_kgm2(u::MotorPropUnit{<:BLDCMotorParams})`
   - Problem: `MotorPropUnit` has two type parameters; method is invalid.
   - Fix: `rotor_inertia_kgm2(u::MotorPropUnit{M,P}) where {M<:BLDCMotorParams,P<:AbstractPropParams}`

### High
- [x] Update all `QuadrotorParams` call sites for new constructor signature.
   - Patch updates `Build.jl`, but tests still use old signature.
   - Likely impacted tests: `test/verification_contracts.jl`, `test/multirotor_motor_map.jl`.
   - Decide whether to update call sites or add a convenience constructor with defaults.

### Medium
- [x] Audit `RotorOutput` usage:
   - All usages are keyword constructors; no positional uses found.

- [x] Inertia SPD validation:
   - Patch checks SPD in `Build.jl`; consider adding validation to `Validate.jl` too.

### Low
- [x] Document rotor spin sign convention:
   - Patch uses `s_spin = -p.rotor_dir` for gyro torque sign.
   - Ensure docs mention the convention (reaction-torque sign vs. body spin direction).

## Follow-up after patch landing
- Run tests:
  - `Tools/px4_lockstep_julia/test/runtests.jl`
  - Any new tests added by the patch (e.g., `vehicles_inertia_gyro.jl`)
- Update docs/limits list if new requirements appear.
