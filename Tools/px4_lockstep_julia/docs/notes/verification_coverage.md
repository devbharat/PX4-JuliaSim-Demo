# Verification Coverage

This document summarizes **what is currently covered** by automated tests in
`Tools/px4_lockstep_julia/test/` and the **few remaining gaps** worth adding.

The goal is to keep verification deterministic, fast, and auditable.

---

## Current coverage

### Analytic / invariant checks (Tier 1)
These are fast, always‑on tests that catch numerical regressions and sign errors.

- Simple harmonic oscillator (analytic)
- Pendulum (small‑angle analytic + energy invariant)
- Circular Kepler orbit (analytic + invariants)
- Torque‑free rigid body invariants
- Constant body‑rate quaternion integration
- Free‑fall under gravity (analytic)

### Integrator correctness + determinism
- RK4 / RK23 / RK45 correctness on free‑fall and plant‑state variants
- Adaptive RK45 quantize‑us drift guard
- Plant‑aware error norm opt‑in behavior
- RK45 reference‑compare utility

### Runtime engine contracts
- Boundary stage ordering (scenario → wind → derived outputs → autopilot)
- Scheduler invariants (exact hits, constant time_us deltas)
- Strict lockstep rate mismatch errors
- Engine respects `t_end` (no overshoot)
- Autopilot command sampling (ZOH) and log samples pre‑step state
- Record/replay equivalence on log ticks (Tier0)

### Plant model / coupling contracts
- Plant outputs purity and RHS consistency
- Bus‑solve residual sweep (fixed‑point consistency)
- Power network current sharing + avionics load
- Multi‑bus voltage mapping
- Back‑EMF can clamp motor current to zero (bus does not droop)
- Fault semantics: motor disable, battery disconnect

### Power / battery
- Thevenin battery step‑load analytic checks
- Multi‑battery PlantState math
- init_plant_state with multiple batteries
- when_soc_below uses minimum SOC across batteries

### Propulsion + geometry
- Propulsor axis geometry (force/torque direction)
- Wrench composition (r×F + axis*Q)
- Vax sign from axis projection
- Wingtra‑style twin forward props (yaw via differential thrust)
- Reaction‑torque roll for forward props
- CA axis parameter sign convention
- Rotor_dir yaw‑torque sign ownership

### Environment
- ISA1976 spot checks
- OU wind discrete update contract
- Gust model delegation

### uORB bridge / injection
- uORB trait validation
- Injection scheduling with multiple battery_status publishers
- Duplicate instance guard

---

## Remaining gaps (short list)

1) **PX4 interface mapping golden test**
   - A deterministic PX4 run (e.g., on‑ground + arm + takeoff) with a golden log
     to verify uORB field mapping, frames, and sign conventions at the ABI boundary.

2) **Battery warning threshold mapping vs PX4 params**
   - If PX4 relies on `battery_status.warning`, add a regression that drains SOC
     and asserts warning transitions at the configured thresholds.

3) **Contact / ground interactions** (if/when used)
   - A drop test that asserts stability (no NaNs) and bounded penetration when
     contact is enabled.

If you add new subsystems (e.g., sensors, terrain), include one deterministic
contract test per subsystem before expanding scenarios.
