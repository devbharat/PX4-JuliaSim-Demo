## Var-step merge track update — `px4_lockstep_julia_readytoMerge0.zip`

This section is the **delta review + P0 implementation pass** for the variable-step
(`PlantSimulation`) engine, based on your `px4_lockstep_julia_readytoMerge0.zip` snapshot.

Focus: **correctness + determinism first**, then performance. Contact/ground is explicitly
de-prioritized (in-air flight focus).

### P0 completed in this pass

#### P0-A: `AtTime` scenario events are true event boundaries (hybrid-system correct)

**Why this matters**

In the previous fixed-step engine, one-off failures/gusts were effectively quantized to
`dt_autopilot` or `dt` depending on where they were injected. That can:

* hide timing-sensitive bugs (e.g. motor failure during a maneuver)
* make results depend on autopilot cadence (bad coupling)

**What’s now true** (PlantSimulation):

* `EventScenario` exposes `next_event_us(...)` which returns the next unfired `AtTime`.
* `step_to_next_event!` includes that time in the min() alongside periodic triggers and
  `t_end_us`, guaranteeing the integrator never crosses an `AtTime` discontinuity.
* `_process_events_at_current_time!` calls `process_events!` *first* so same-timestamp
  wind/autopilot/log ticks see the modified sim state.

**New test**

* `PlantSimulation: AtTime scenario events are true event boundaries` schedules a
  `fail_motor_at!(t=0.005)` with `dt_autopilot=0.01` and asserts the first integration
  interval ends at exactly 5 ms and the motor is disabled before the next interval.

#### P0-B: Harden battery↔bus↔motor bus-voltage solve contract (bisection assumptions + NaN guards)

**Why this matters**

Adaptive integrators amplify “rare” NaN sources: one non-finite RHS evaluation can poison
the whole step-size controller. The bus solve is the main coupled scalar root find inside
your RHS, so it needs strong invariants.

**What changed** (`src/sim/PlantSimulation.jl`):

* `_solve_bus_voltage(...)` now:
  * fails loudly on non-finite battery params or V0
  * validates the bisection bracket assumptions (monotone I(V), monotone g(V))
  * provides deterministic fixed-iteration bisection with **stringified state dumps**
    on any non-finite intermediate
* `_eval_propulsion_and_bus(...)` now checks `V_bus` and `I_bus_total` are finite and
  throws immediately with context if not.

### Notes on motor/servo actuator models (your question)

You’re right to question whether **1st/2nd-order “motor actuators”** are needed when you
already have rotor-ω dynamics in `Propulsion`:

* For multirotors with an explicit rotor speed state **ω**, the physically meaningful lag
  is primarily **ω̇** (motor+prop inertia + aero torque). In that case, keeping
  `motor_actuators = DirectActuators()` is typically the most correct default.
* A first/second-order motor actuator filter can still be useful if you want to model
  *command-path shaping* (e.g. ESC input filtering, DShot update rate limits), but it’s
  very easy to double-count dynamics and harm controller realism.

**Recommendation (merge-safe):**

* Keep the actuator types for **servos** (fixed-wing control surfaces) and for any future
  propulsion models that don’t include ω.
* For the quadrotor path, default to `DirectActuators` for motors and treat non-direct
  motor actuators as an “expert mode” configuration.

### P1 implemented in this pass (merge robustness upgrades)

1. **Plant-aware adaptive error norm mode** (optional, default off)
   * Added an explicit `plant_error_control::Bool` knob on `RK23Integrator` and
     `RK45Integrator` (default `false`).
   * When enabled, the adaptive error norm can include scaled terms for:
     - rotor ω
     - actuator outputs / rates
     - battery SOC / V1
   * Still uses the existing `atol_* = Inf` defaults to ignore groups unless you opt in.
   * Added a unit test that proves the “opt-in” gating works.

2. **Analytic bus solve in the unsaturated region, with deterministic fallback**
   * `_solve_bus_voltage(...)` now tries a one-shot analytic solve under the assumption
     that all active motors are in the linear (unsaturated) region, validates the region
     assumption, and returns immediately if valid.
   * If the region assumption fails, it falls back to the existing deterministic
     region-classified analytic iteration and then to fixed-iteration bisection.
   * Added unit tests that check the bus equation residual in both linear and saturated
     regimes.

### Remaining P1 backlog (documented, not implemented here)

1. **Second-order actuator rate-limit semantics**: current “hard-ish” limiter is stable
   but not strictly equivalent to discrete clamping. Decide on a consistent projection or
   saturating ODE.


---

## Consolidated review — `px4_lockstep_julia_latest3.zip` + `px4_lockstep_latest3.zip`

This is a fresh review of the *current* codebase (Julia + C lockstep library), integrating the previous review and the patches you documented at the bottom of the prior `review.md`.

Note: the original review was based on static inspection; the status callouts below now reflect implemented fixes and associated tests.

- Julia: `src/PX4Lockstep.jl`, `src/sim/*.jl`, `test/runtests.jl`
- C: `include/px4_lockstep/px4_lockstep.h`, `src/px4_lockstep.cpp`

---

## 0) System contract (what the simulator must guarantee)

This defines the intended contracts and invariants your code implies today. Where something is ambiguous, I call it out explicitly, but still proceed with best-effort assumptions.

### Lockstep + determinism contract with PX4

**Goal:** Given the same scenario, seed, initial conditions, model params, and the same `libpx4_lockstep` build, you should get identical:

- PX4 outputs per autopilot tick
- Sim trajectory (truth)
- Logs (byte-identical CSV on the same machine/software stack)

Your current implementation largely achieves this by enforcing an **integer microsecond** timebase and stepping everything off **integer step counters**.

**Contract / invariants:**

- **Single source of time**
  - Physics time advances in fixed steps `dt` (seconds) with `dt_us = round(dt*1e6)` enforced exact (`Simulation._dt_to_us`).
  - Lockstep time is `time_us = t0_us + step * dt_us` (`Simulation.time_us(sim)`), strictly monotonic.

- **Fixed PX4 cadence**
  - PX4 is stepped at `dt_autopilot = ap_steps * dt` (exact integer multiple; enforced by `_multiple_steps`).
  - PX4 stepping uses integer step scheduling (`StepTrigger`) — no float drift/jitter in cadence.

- **Replayability**
  - All randomness is derived from deterministic seeds and uses explicit RNG objects (split into `rng_wind`, `rng_est`, `rng_misc`).
  - No threading in the Julia sim loop (single-threaded, deterministic ordering).
  - C-side lockstep relies on injected `time_us` via `hrt_lockstep_set_absolute_time()` and does not reference wall-clock time (assumed; must remain true).

- **Controller ZOH semantics**
  - Between PX4 ticks, the actuator outputs are held constant (`sim.last_out` sample-and-hold).
  - Plant is advanced at `dt` with those held commands (plus actuator/motor dynamics inside the plant).

### State definitions / frames

From Julia code (`Types.jl`, `RigidBody.jl`, `Vehicles.jl`, `Simulation.jl`) and the C injection layer:

- **World frame:** NED
  - `pos_ned = (x_n, y_e, z_d)` [m]
  - `vel_ned` [m/s]
  - gravity is positive down: `g_ned = (0,0,+g)`.

- **Body frame:** FRD
  - +X forward, +Y right, +Z down.

- **Attitude:**
  - `q_bn` is quaternion rotating **Body → NED** in `(w,x,y,z)` order.
  - `quat_deriv(q_bn, ω_body)` uses `q̇ = 0.5*q⊗ω` consistent with Body→NED (good).

- **Body rates:**
  - `ω_body = (p,q,r)` rad/s in body FRD.

- **Wind conventions:**
  - `wind_velocity(...)` returns wind velocity in NED: **air relative ground**.
  - Two “relative wind” vectors appear:
    - Vehicle velocity relative air (airspeed): `v_rel_ned = vel_ned - wind_ned` (used for drag in `Vehicles.jl`).
    - Air velocity relative vehicle (flow past vehicle): `v_air_ned = wind_ned - vel_ned` (used for inflow in `Simulation.jl`, logged as `air_vel_body` after rotation).
  - This dual convention is fine, but it must remain *explicit* to avoid sign bugs.

### Update ordering (per physics tick)

From `Simulation.step!` (Julia):

1. Snapshot truth state `x0` (pre-step) for consistent logging.
2. Advance wind disturbance state once per physics tick:
   - `step_wind!(sim.env.wind, x0.pos_ned, t, dt, rng_wind)`
3. Scenario step:
   - produces `cmd::AutopilotCommand` and `landed::Bool`.
4. Sample wind once per tick (sample-and-hold):
   - `sample_wind!(sim.env.wind, x0.pos_ned, t)`
   - this held sample is used for drag, propulsion, and logging across RK4 stages.
5. Sample battery status for injection into PX4:
   - `batt = status(sim.battery)` (sampled at start of interval).
6. If autopilot is due:
   - `estimate!(...)` at fixed `dt_hint = sim.ap_dt`
   - `autopilot_step(..., time_us(sim), est..., cmd; landed, battery=batt)`
   - store `sim.last_out`.
7. Convert PX4 outputs to actuator commands:
   - sanitize NaNs and clamp motors to [0,1]
8. Step actuator dynamics at physics dt.
9. Step propulsion (motor/prop) using:
   - duty commands
   - battery voltage sample (`batt.voltage_v`)
   - density `ρ(alt_msl)`
   - air-relative velocity in body frame
10. Integrate rigid-body dynamics with Euler/RK4 using held propulsion output.
11. Update battery model using propulsion bus current.
12. If logging is due:
   - log the **pre-step** snapshot `x0` and other sampled values.

### Multi-rate stepping semantics

- **Physics dt**: fixed (`cfg.dt`)
- **Autopilot dt**: fixed integer multiple of dt:
  - `dt_autopilot = ap_steps * dt`
- **Logging dt**: fixed integer multiple of dt

**Key invariant:** `dt_autopilot` and `dt_log` must be exact multiples of `dt` (enforced).

### Estimator injection semantics

The estimator is *not* raw sensor simulation; it’s “EKF-like output injection”.

- Estimator is evaluated only when PX4 is stepped, and is advanced with **fixed dt_hint** (not `t - last_t`).
- Noise/bias processes are stepped deterministically using the estimator RNG stream.
- Optional delay is quantized in steps (ring buffer) and now enforces `dt_hint == dt_est`.

### Ambiguities (resolved)

These were correctness traps in earlier revisions; they are now handled explicitly:

1. **World origin**
- `HomeLocation` is now an alias of `WorldOrigin`, and `EnvironmentModel` stores the same origin.
- `SimulationInstance` synchronizes defaults and warns if both sides are custom but mismatched.

2. **Wind sampling semantics**
- Wind is stepped, then sampled once per tick via `SampledWind`, and held constant across RK4 stages.
- Logs record the held sample alongside `time_us` for unambiguous correlation.

3. **Gust boundaries**
- Gusts are half-open (`[t_on, t_off)`) and combined with sample-and-hold wind so forcing is constant over each tick.

4. **Multiple lockstep handles**
- The Julia wrapper now enforces one active handle by default; `allow_multiple_handles=true` is an explicit opt-out.

5. **Battery warning semantics**
- Battery models now emit warnings based on SOC thresholds, but you may still want to verify the mapping vs PX4 params.

---

## 1) Determinism + lockstep audit (highest priority)

You fixed the biggest determinism killers (strict lockstep rate enforcement, sampled wind semantics, delayed-estimator dt checks, half-open gusts, ABI handshake). The remaining risks are mostly C-side scheduling behavior.

### A) Determinism breakers / risks still present

Most earlier issues here are now resolved (strict lockstep rate checking, deterministic
wind sampling, gust boundary semantics, delayed estimator dt checks, and handle guards).
The main remaining determinism risk is on the C side:

#### 1) **C-side StepRateLimiter drift/jitter and “first tick doesn’t run” behavior**
`StepRateLimiter::should_run()` uses:

```cpp
if ((now_us - last_run_us) >= period_us) { last_run_us = now_us; return true; }
```

Two issues:

- **No catch-up**: if dt is bigger than period, the loop runs slower, not multiple times.
- **Phase jitter**: setting `last_run_us = now_us` (instead of `+= period_us`) causes drift if `dt_us` doesn’t divide `period_us`.
- **First tick**: with `last_run_us=0` and `now_us=0`, nothing runs at t=0 (often fine, but it’s a semantic choice worth making explicit).

**Where:** `px4_lockstep.cpp` — `StepRateLimiter`.

---

### B) Checklist of invariants to add as asserts/tests

These are “if violated, stop immediately” invariants.

#### Timebase + scheduling invariants

- Assert `dt_us == round(dt*1e6)` exactly (already enforced).
- Assert `t0_us` microsecond-quantized (already enforced).
- Assert `dt_autopilot` and `dt_log` are exact multiples of `dt` (already enforced).
- Assert lockstep `time_us` is strictly monotonic and increments by exactly `dt_autopilot_us` between PX4 steps.
  - Store last PX4 step time in `PX4LockstepAutopilot` and assert `Δ == dt_ap_us`.

- **For PX4 lockstep only:** assert `dt_autopilot <= 1/max_internal_rate_hz` unless explicitly overridden.

#### RNG invariants

- Assert wind/estimator/misc RNG are distinct objects (already true).
- Add a “no global RNG” guard in CI by grepping for `rand(` / `randn(` without an explicit RNG argument (you already pass RNG everywhere).

#### State validity invariants

- Quaternion must be finite and normalized within tolerance each physics step:
  - `abs(norm(q) - 1) < 1e-9` and all components finite.
- No NaNs in truth state:
  - `isfinite.(pos, vel, ω)`.

- Actuator commands must be finite:
  - After sanitize/clamp, motor duties must be finite in [0,1].

#### ABI invariants

- ABI version must match (already checked).
- Struct size must match (already checked).
- **Add (optional but strong):** offset checks for every field (see below).

#### “One handle per process” invariant

- Implemented: `PX4Lockstep.create()` enforces a single active handle by default and decrements on `destroy()`.

---

### C) Concrete refactors to harden determinism (and “don’t do X” rules)

#### Refactor 1 (P0): Make PX4 step-rate mismatch a hard error by default
- Add `strict_lockstep_rates::Bool = true` to `SimulationConfig` or `PX4LockstepAutopilot`.
- If `strict_lockstep_rates && ap_dt > 1/max_hz`: throw `ArgumentError`.
- Keep the warning behavior only when the user opts out.

**Why:** This prevents “looks stable” results from a fundamentally wrong PX4 timing.

**Status:** Implemented (strict by default; opt-out warns).

#### Refactor 2 (P0): Fix `DelayedEstimator` dt mismatch
- Store `dt_est_us` (or `dt_est`) in `DelayedEstimator`.
- In `estimate!(::DelayedEstimator, ..., dt_hint)`:
  - assert `abs(dt_hint - dt_est) <= 1e-12` (or compare microseconds).
  - if mismatch: throw with a clear message.

**Status:** Implemented (microsecond-quantized enforcement).

#### Refactor 3 (P0/P1): Make gusts half-open intervals and/or step-quantized
- Change GustStep to:
  - active if `t >= t_on && t < t_off`
- Change OUWind step gust to:
  - `t < step_until_s` not `<=`
- Better (preferred): remove `t` dependence entirely for step gusts:
  - model gust as state set/cleared by events at tick boundaries.

**Status:** Implemented half-open bounds + `SampledWind` sample-and-hold, so gust forcing is constant across RK4 stages.

#### Refactor 4 (P1): Improve C-side StepRateLimiter to avoid drift
- Add an `initialized` flag so first call runs immediately (optional).
- Replace `last_run_us = now_us` with:
  - `last_run_us += period_us` (possibly in a while loop if you want catch-up)
- If you do **not** want catch-up, still prefer:
  - `last_run_us = last_run_us + period_us * floor((now-last_run)/period_us)` to keep phase-locked.

#### Refactor 5 (P1): Optional uORB offset handshake
Size checks are good, but offsets are better.

Add a helper to expose uORB field offsets for key topics (home position, global
position, actuator outputs). Then compare against `fieldoffset` on the Julia side.

This eliminates the “same size, wrong layout” failure mode.

#### “Don’t do X” rules

- Don’t schedule anything periodic using float comparisons (`t >= next_t`); always schedule using step counters or integer microseconds.
- Don’t call `rand()` / `randn()` without an explicit RNG object.
- Don’t rely on `Dict`/`Set` iteration order for anything that affects stepping order.
- Don’t create more than one lockstep handle per process unless you’ve proven the C runtime is re-entrant.

---

## 2) Correctness review by subsystem

### Risk items table

| Subsystem | Symptom you’d see | Likely root cause | How to test (concrete) | Recommended fix |
|---|---|---|---|---|
| PX4 lockstep cadence | Flight “works” but different stability/response vs PX4 SITL | `dt_autopilot` slower than fastest PX4 loop (if strict guard disabled) | Log `time_us` per PX4 step + confirm `Δ=dt_ap_us`; compare against configured rates; run a step response and compare to reference SITL | Implemented: `strict_lockstep_rates` defaults to error; opt-out warns |
| DelayedEstimator | Delay sweep results don’t match configured delay | `delay_steps` computed from `dt_est` but stepping uses `dt_hint` | Unit test: ramp input; configured 50 ms delay at 10 ms dt should yield exact 5-step lag | Implemented: `dt_est_us` stored; mismatch throws |
| Wind/gust boundaries | Small “kinks” at gust end; RK4 behaves oddly on boundary ticks | inclusive `t<=t_off`; RK4 intermediate evaluation crosses boundary | Test: gust duration exactly one tick, verify wind constant across that tick including RK4 stages | Implemented: half-open intervals + sample-and-hold wind per tick |
| Wind sampling phase | Logged wind at t=0 is already “advanced”; confusing correlation with state | `step_wind!` called before logging at same t | Determinism test: log wind and confirm whether it represents [t,t+dt) sample | Implemented: `SampledWind` provides a held sample; logs record held wind + `time_us` |
| Home alt vs density alt | PX4 altitude and atmosphere density disagree | PX4 uses `HomeLocation.alt_msl_m`, atmosphere uses `origin.alt_msl_m` | Create sim where these differ by 500 m; compare hover throttle required vs PX4 estimate | Implemented: shared `WorldOrigin` + sync/warn on mismatch |
| Battery/prop coupling | Voltage sag response is delayed by 1 tick; thrust slightly “too high” at step changes | Propulsion uses pre-step `batt.voltage_v` and battery is updated after current computed | Step duty from 0→hover and check bus voltage and thrust response timing | Solve bus voltage under load (one iteration is enough) or document explicit “sampled voltage” semantics |
| Battery warning / failsafe | PX4 never triggers RTL/land on low battery | SOC warnings may not match PX4 config thresholds | Force SOC low and verify `battery_warning` changes and PX4 nav_state transitions to RTL | Battery models emit SOC-based warnings; verify mapping vs PX4 params |
| Quaternion integration | Long-run attitude drift; RK4 accuracy lower than expected | Normalizing quaternion inside RK4 stages (projection) alters vector field | Unit test: constant body rate for 60s vs analytic; compare yaw error scaling vs dt^4 | Normalize only at end of step, or use Lie-group (SO(3)) integrator |
| Contact model | Bouncy ground, chatter, or occasional NaNs at high impact | stiff penalty model + RK4 + discontinuity | Drop test from 2m at various dt; ensure no NaNs and penetration bounded | Add penetration clamp + energy dissipation; consider splitting contact integration or using semi-implicit step for contact |
| Propulsion inflow | Inflow correction affects climb and descent identically | inflow factor depends on `mu^2` (signless) | Compare thrust at +Vax vs -Vax same magnitude; should differ if modeling ascent/descent asymmetry | Decide model intent; if you care, use sign-aware inflow model or induced velocity model |
| Motor model | Current spikes clipped; unrealistic transients | quasi-static current clamp and explicit ω integration | Step duty from 0→1; validate ω and current are stable and plausible | Fine for now; later add electrical inductance if needed (fidelity) |
| C StepRateLimiter | Slight drift of loop execution phase, especially for odd dt/rates | `last_run_us=now_us` update rule | In C harness, log actual run times for a module when dt doesn’t divide period | Make limiter phase-locked (`+=period_us`), consider catch-up policy |
| Multiple PX4 handles | Strange cross-talk between sims or nondeterministic init | global static PX4/uORB state reused | Unit test: create/destroy twice and ensure outputs identical; create two in parallel should error | Implemented: handle guard; `allow_multiple_handles=true` opt-out |
| Logging contract | Confusion about whether log row is pre-step or post-step | You log pre-step truth but after stepping wind | Add explicit log field `time_us` + state “sample instant” in header | Implemented: `time_us` column + held wind sample per tick |
| Autopilot command timing | Mission/arm happens 1 tick later than expected | scenario updates at dt, autopilot sampled at dt_autopilot | Integration test: arm_time not aligned to dt_autopilot; confirm it arms at next PX4 tick | Document as ZOH sampling; provide helper that snaps event times to PX4 ticks |

### Specific “golden tests” (unit + integration) — 10+ with signals and tolerances

Below are tests I would add (or ensure exist) that are *directly* tied to the contracts and the common failure modes. I’m being specific about signals and what “pass” means.

#### A) Determinism / scheduling (P0)

1) **Bitwise replay test (same seed, same config)**
- Run the same sim twice.
- Dump CSV logs.
- **Pass:** files are byte-identical (or row-by-row exact equality for every numeric field).
- If you can’t guarantee byte-identical across platforms, require exact on the same machine and allow small tolerances cross-platform.

2) **PX4 cadence test**
- Instrument `PX4LockstepAutopilot` to record every `time_us` passed to `px4_lockstep_step_uorb`.
- **Pass:** `diff(time_us)` is exactly constant and equals `dt_autopilot_us` for the whole run.

3) **Multi-rate alignment test**
- Choose `dt=0.002`, `dt_autopilot=0.01`, `dt_log=0.02`.
- **Pass:** autopilot steps on steps {0,5,10,...}, logs on {0,10,20,...}. No drift after 10k steps.

#### B) Estimator injection (P0)

4) **DelayedEstimator step-quantized delay**
- Build `DelayedEstimator(inner=TruthEstimator(), delay_s=0.05, dt_est=0.01)`.
- Feed a known input: `pos_ned = (t,0,0)` sampled at 0.01.
- **Pass:** output `pos_ned.x` equals input from exactly 5 steps earlier (no fractional drift), for all steps after fill.
- Also add a **negative test**: call `estimate!(..., dt_hint=0.02)` and assert it throws.

#### C) Wind / gust correctness (P0/P1)

5) **Gust duration quantization test**
- Use `EventScenario.wind_step_at!(t_start=1.0, duration=0.03)` with `dt=0.01`.
- **Pass:** gust is active for exactly 3 logged ticks (or your chosen quantization rule), and never partially within a tick.

6) **OUWind stationary variance test (statistical)**
- Run OUWind alone with `tau=1.0`, `sigma=(1,1,1)`, `dt=0.01` for 200s (with fixed seed).
- Compute empirical variance of each component over the last 150s.
- **Pass:** variance within e.g. ±10% of `sigma^2`. (This is a regression guard more than physics truth.)

#### D) Rigid body + integrators (P0/P1)

7) **Free-fall analytic test (no drag, no thrust)**
- Set mass/inertia arbitrary, no thrust, no drag.
- Start at `pos_z=0`, `vel_z=0`.
- Integrate for 1.0s.
- **Pass:** `vel_z ≈ g*t` and `pos_z ≈ 0.5*g*t^2` with tolerance:
  - Euler: error O(dt), set tolerance ~ `2*dt*g`
  - RK4: much tighter, e.g. 1e-6–1e-5 depending on dt.

8) **Constant body-rate attitude test (already partially present)**
- Hold ω=(0,0,1) rad/s and integrate 10s.
- **Pass:** yaw error < 1e-6 rad (RK4) at dt=1e-3, and error scales ~dt^4 when halving dt.

#### E) Actuators + propulsion (P1)

9) **First-order actuator exact time constant**
- For `FirstOrderActuators{1}(τ=0.05)` with dt=0.01:
- Step input from 0→1 at t=0.
- **Pass:** after one τ (0.05s), output is `1 - exp(-1) ≈ 0.632` within 1e-3.

10) **Yaw torque sign test (rotor_dir correctness)**
- Freeze attitude level, apply equal thrust but set rotor directions with known pattern.
- Introduce a small differential that should yield positive yaw acceleration.
- **Pass:** sign of `ω_dot.z` matches expected given rotor_dir and km sign.

11) **Battery sag timing test**
- Apply a step in duty at t=0, log `batt_v` and `bus_current_a`.
- **Pass:** whichever semantics you choose, the response is consistent:
  - If “voltage under load same tick”: sag begins at t=0 tick.
  - If “sampled voltage”: sag begins at t=dt tick (but then document it).

11b) **Battery warning threshold test**
- Configure battery thresholds (e.g. SOC-based) and drive SOC below each threshold.
- **Pass:** `battery_warning` transitions through expected levels at the correct SOC/voltage, and PX4 reacts (e.g. nav_state enters RTL) if commander is enabled.

#### F) Contact model (P1)

12) **Drop test stability**
- Drop from 1m with zero thrust, dt=0.002.
- **Pass:** no NaNs; penetration `pos_z` never exceeds e.g. 1 cm (tune to your stiffness); total energy decreases after impact (with damping).

#### G) C ABI + integration safety (P0/P1)

13) **ABI mismatch test**
- Temporarily override expected ABI version in Julia (or mock C getter).
- **Pass:** `create()` throws with a message that includes expected/actual and struct sizes.

14) **Single-handle enforcement test**
- Create a handle, then attempt to create a second without destroying the first.
- **Pass:** second create throws (or returns error code) with a clear message.

15) **PX4 step monotonic test**
- Call lockstep step with time_us decreasing (intentional).
- **Pass:** Julia wrapper throws before calling C, or C returns error and Julia surfaces it.

---

## 3) Architecture / API design review

### Keep / Change / Remove

#### Keep

- **Microsecond-quantized timebase** (`dt_us`, `t0_us`) and step-based scheduling (`StepTrigger`). This is the right foundation.
- **RNG stream splitting** per subsystem.
- **Clear module separation** inside `PX4Lockstep.Sim`:
  - Environment / RigidBody / Integrators / Vehicles / Propulsion / Powertrain / Logging.
- **C ABI wrapper remains dependency-light** (StaticArrays + stdlib deps).

#### Change

- **Unify “world origin” concept** (implemented):
  - `WorldOrigin` is now shared by PX4 and `EnvironmentModel`, with automatic sync/warn on mismatch.

- **Make sampling semantics explicit and testable** (implemented)
  - Wind is sampled once per tick (`SampledWind`), held across RK4 stages, and logs include `time_us`.

- **Harden lockstep handle lifecycle** (implemented)
  - Single active handle enforced by default; `allow_multiple_handles=true` is an explicit opt-out.

- **Delay estimator correctness** (implemented)
  - `DelayedEstimator` stores `dt_est_us` and rejects mismatched `dt_hint`.

#### Remove (or quarantine)

- The float-based `autopilot_step(ap, t::Float64, ...)` overload should be treated as a convenience only.
  - Keep it, but mark as “not for lockstep-critical paths” (you already did), and avoid calling it from the sim engine.

### Module boundaries / dependency direction

Right now you include `Sim` from inside `PX4Lockstep` (single package). That’s fine while deps stay light.

Long-term, if Sim grows (terrain, geo, file formats, plotting), you will want either:

- a separate package `PX4LockstepSim.jl`, or
- make Sim an optional extension package using Julia’s package extension mechanism.

Don’t do this yet unless you start pulling in heavy deps.

### Type design and dispatch patterns

- `SimulationInstance{...}` is heavily parametric — good for type stability, but compile times can climb.
- `EventScheduler` uses `Vector{AbstractEvent}` (dynamic dispatch each tick). For a small number of events, fine; for lots of events, you may want a typed scheduler.

### Data ownership / mutation

Good pattern overall:

- truth state is immutable `RigidBodyState`, replaced each step
- models like batteries, wind, actuators are mutable and stepped in-place

One minor sharp edge:

- `dynfun = DynamicsWithContact(vehicle.model, env, contact)` captures env/contact by value.
  - If someone replaces `sim.env` or `sim.contact`, dynfun won’t see it.
  - Either document “env/contact objects are not replaced after construction” or rebuild dynfun if they are.

### Configuration ergonomics

You’re in a good place: defaults run, but power users can swap models.

A few high-payoff tweaks:

- Implemented: `EnvironmentModel` now takes `origin::WorldOrigin`, and PX4 home syncs to it.
- Provide a `SimDefaults.iris_px4_lockstep()` helper that returns a fully consistent stack (origin, density, rotor geometry, PX4 config).

### Logging API

- `SimLog` + `log_to_csv` is fine and deterministic.
- Implemented: `time_us` added to logs (`schema_version=2`).

Also consider logging lockstep config hash and sim config at the top of CSV (as comment lines) for provenance.

### Error handling / diagnostics

The sim is correctly “fail fast” in many places (bad dt, bad microsecond quantization, ABI mismatch).

Handled “fail louder” cases:

- PX4 cadence mismatch now errors by default (`strict_lockstep_rates`).
- DelayedEstimator dt mismatch now errors on mismatch.
- Multiple lockstep handles are guarded by default.

Remaining:

- C StepRateLimiter drift/jitter (needs phase-locked update).

### 3–5 concrete API changes that will pay off

1) **World origin unification** (implemented)
```julia
struct WorldOrigin
  lat_deg::Float64
  lon_deg::Float64
  alt_msl_m::Float64
end
```
Use it in both `EnvironmentModel` and `PX4LockstepAutopilot`.

2) **Add `time_us` to log schema** (implemented)
- store `UInt64` or `Float64(time_us)*1e-6` plus the integer.

3) **Strict lockstep rate option** (implemented)
- add `strict_lockstep_rates=true` default for PX4 autopilots.

4) **DelayedEstimator stores dt** (implemented)
- `DelayedEstimator(inner; delay_s, dt_est)` should store `dt_est_us` and enforce.

5) **LockstepSession** (optional)
- A small wrapper that manages “one PX4 runtime per process” and ensures clean shutdown.
- Current code enforces a single handle by default without a separate session type.

---

## 4) Performance + numerical stability review (determinism-preserving)

### What I would profile in Julia (and what to look for)

Even without threads, you want the sim to run fast enough for long sweeps.

Profile targets:

- `Simulation.step!`
- `Vehicles.dynamics(::IrisQuadrotor, ...)`
- `Propulsion.step_propulsion!` / `_step_unit!`
- `Integrators.step_integrator!` (Euler/RK4)
- `Logging.log_sample!`

In Julia, look for:

- allocations per tick (`@allocated step!(sim)` should be ~0 in steady state)
- type instabilities (`@code_warntype step!(sim)`; any `Any` in hot path is a red flag)
- dynamic dispatch in inner loops (often from abstract containers)

### Concrete allocation hot spots to watch

1) **`MVector` usage in propulsion**
`MVector` is a mutable struct. In Julia, mutable structs generally heap-allocate unless scalar-replaced.

If you see allocations in `step_propulsion!`, replace `MVector` with `ntuple`/`SVector` construction:

- build `NTuple{N,Float64}` with `ntuple` and then `SVector(...)`.

2) **EventScheduler dynamic dispatch**
`Vector{AbstractEvent}` + calling `_should_fire` per event per tick can become costly if you start adding lots of events.
Not urgent unless you go heavy on event-driven scenarios.

3) **Frequent `exp()` calls**
- `FirstOrderActuators` uses `exp(-dt/τ)` every tick.
- `OUWind` uses `exp(-dt/τ)` every tick.
- `AR1` uses `exp(-dt/τ)` every tick.

Since dt is constant, you can precompute coefficients (α, ϕ) once per sim config and store them inside these models when constructed.
**Status:** Implemented via cached coefficients in each model.

### Numerical stability concerns

- **RK4 + discontinuities**: contact steps are discontinuous. RK4 assumes smoothness.
  - Either:
    - keep dt small and accept it, or
    - treat contact forcing as tick-quantized (constant over dt), which makes RK4 “honest” again.

- **Contact stiffness**: penalty methods can go unstable if `k*dt^2/m` is too large.
  - Add a guideline/assert like:
    - `dt < 0.2*sqrt(m/k)` (rule of thumb) for stability.

- **SecondOrderActuators**: semi-implicit Euler is OK at typical dt, but can go unstable for large ωn.
  - Add a warning if `ωn*dt > ~0.3` (tunable).

---

## 5) Prioritized improvement backlog

Effort scale:
- **S**: 1–4 hours
- **M**: 1–3 days
- **L**: 1–2+ weeks

### P0 (completed)

**P0 — Enforce correct PX4 cadence by default**
- **Rationale:** Prevents invalid “lockstep” runs where PX4 loops execute too slowly.
- **Plan:**
  - Add `strict_lockstep_rates=true`.
  - If `max_internal_rate_hz` is known and `ap_dt > 1/max_hz`, throw.
  - Provide opt-out if needed.
- **Effort:** S
- **Acceptance criteria:**
  - With default settings, a mismatched dt triggers an exception with clear message.
  - With opt-out, it only warns.
  - **Status:** Implemented.

**P0 — Fix DelayedEstimator dt mismatch**
- **Rationale:** Silent wrong delay is unacceptable for delay sensitivity tests.
- **Plan:**
  - Store `dt_est_us` in `DelayedEstimator`.
  - Compare against `dt_hint` (microsecond-quantized) in `estimate!`.
- **Effort:** S
- **Acceptance criteria:**
  - Unit test passes for correct dt.
  - Unit test throws for mismatch.
  - **Status:** Implemented.

**P0 — Gust boundary semantics (half-open / tick-quantized)**
- **Rationale:** Avoid within-step switching with RK4; make gust timing unambiguous.
- **Plan:**
  - Change `<=` to `<` for end bounds.
  - Optionally rework step gust to be set/cleared via events only.
- **Effort:** S
- **Acceptance criteria:**
  - A gust scheduled for exactly N ticks remains constant across each tick’s RK4 stages.
  - Integration test: gust active for correct number of ticks.
  - **Status:** Implemented (half-open + sample-and-hold wind).

**P0 — Add `time_us` to log schema**
- **Rationale:** Debugging lockstep without time_us in logs is pain; also validates contract.
- **Plan:**
  - Add column `time_us` to log row.
  - Populate with `Simulation.time_us(sim)` for the logged tick.
- **Effort:** S
- **Acceptance criteria:**
  - CSV includes `time_us`.
  - Cadence test can be written purely from logs.
  - **Status:** Implemented (`schema_version=2`).

### P1 (high payoff, reduces “future debugging tax”)

**P1 — Unify world origin between PX4 injection and atmosphere**
- **Rationale:** Prevent subtle altitude/density inconsistencies.
- **Plan:**
  - Introduce `WorldOrigin` and use it in both `EnvironmentModel` and `PX4LockstepAutopilot`.
  - Warn or assert if mismatch.
- **Effort:** M
- **Acceptance criteria:**
  - Single source of truth for `(lat,lon,alt)`.
  - Hover thrust matches when density changes with altitude.
  - **Status:** Implemented (`WorldOrigin` + sync/warn).

**P1 — Decide and codify wind sampling semantics**
- **Rationale:** Avoid phase confusion; ensure “wind held constant over dt” is actually true in RK4.
- **Plan options (pick one):**
  1) Sample wind at tick start, store `wind_sample_ned`, use that everywhere (drag + propulsion + log).
  2) Step wind at end of tick so state corresponds to “value at t”.
- **Effort:** M
- **Acceptance criteria:**
  - Documented semantics.
  - Wind is constant across RK4 stage evaluations.
  - **Status:** Implemented (`SampledWind` sample-and-hold).

**P1 — Battery/proplusion coupling improvement**
- **Rationale:** Remove 1-tick lag in voltage sag and improve energy consistency.
- **Plan:**
  - One-iteration corrector:
    - compute current from V_prev
    - update battery RC state
    - recompute V under load and current
    - optionally average
  - Or solve algebraic loop explicitly (future).
- **Effort:** M
- **Acceptance criteria:**
  - Step duty response shows voltage sag timing consistent with chosen semantics.
  - No numerical instability.


**P1 — Implement battery warning semantics (if PX4 expects it)**
- **Rationale:** If PX4 uses `battery_status.warning`, you currently can’t test battery failsafe behavior (RTL/land) in lockstep.
- **Plan:**
  - Decide source of truth: compute warning in sim from SOC/voltage, or confirm PX4 computes warning and stop injecting it.
  - If sim-computed: implement threshold mapping (none/low/critical/emergency/failsafe) and test transitions.
- **Effort:** S/M
- **Acceptance criteria:**
  - With a scripted SOC drain, `battery_warning` changes at the configured thresholds.
  - PX4 transitions to expected failsafe modes when commander is enabled.
  - **Status:** Partially implemented (SOC-based warnings emitted; verify mapping vs PX4 params).

**P1 — Improve C StepRateLimiter**
- **Rationale:** Avoid drift/jitter in module scheduling; make behavior explicit.
- **Plan:**
  - phase-lock limiter (`+= period_us`)
  - decide catch-up policy (probably “no catch-up, but phase-locked”)
  - optionally run once at t=0
- **Effort:** M
- **Acceptance criteria:**
  - Module run timestamps match expected periodic schedule for a variety of dt/period combos.

**P1 — Enforce “one lockstep handle per process”**
- **Rationale:** Prevent non-obvious corruption in Monte Carlo/sweeps.
- **Plan:**
  - global refcount or singleton guard in Julia wrapper.
- **Effort:** S/M
- **Acceptance criteria:**
  - Second `create()` errors unless previous handle destroyed.
  - **Status:** Implemented (guard + `allow_multiple_handles` opt-out).

**P1 — Precompute dt-dependent coefficients**
- **Rationale:** Speed without sacrificing determinism.
- **Plan:**
  - Store `α`, `ϕ` etc in AR1/OUWind/FirstOrderActuators once per dt.
- **Effort:** M
- **Acceptance criteria:**
  - `@allocated step!(sim)` stays ~0.
  - tick runtime decreases measurably.
  - **Status:** Implemented (AR1/AR1Vec/OUWind/FirstOrderActuators cache dt coefficients).

### P2 (nice to have, but don’t distract from core correctness)

**P2 — Lie group attitude integration**
- **Rationale:** More accurate and robust attitude propagation, especially for large rates.
- **Plan:**
  - implement SO(3) exponential update: `R_{k+1} = R_k * Exp(ω*dt)`
  - keep quaternion as storage, convert via exp map
- **Effort:** M/L
- **Acceptance criteria:**
  - constant-rate test shows improved accuracy vs RK4 projection.

**P2 — Typed event scheduler**
- **Rationale:** Performance + clarity if you get heavy on events.
- **Plan:** parametric scheduler storing concrete event types.
- **Effort:** M
- **Acceptance criteria:** less dynamic dispatch in `step_events!`.

---

## Fast wins in a weekend (highest ROI)

1) Make PX4 cadence mismatch a hard error by default (done).
2) Fix DelayedEstimator dt mismatch (done).
3) Switch gust intervals to half-open `[t_on, t_off)` (done).
4) Add `time_us` column to logs (done).
5) Add a guard against multiple lockstep handles (done).
6) Replace `MVector` in `step_propulsion!` if it shows allocations (pending).

---

## Things not worth doing yet (until remaining P1s are done)

- Full WGS84/ECEF geodesy everywhere (origin unification is done; finish remaining P1s first).
- High-fidelity rotor aerodynamics (BEMT) before you lock down timing, sign conventions, and test coverage.
- Multithreading / parallel Monte Carlo inside one process (PX4 lockstep almost certainly isn’t re-entrant).
- Fancy terrain/contact (slopes, friction) before stabilizing the simple flat-ground model.
- Sensor-level simulation (IMU/mag/baro/GPS) before your “EKF-output injection” path is rock-solid and well-tested.

---

## Fidelity feature ideas (roadmap — after correctness/perf)

These are **feature** ideas, not correctness fixes. I’m listing them because they will eventually matter for realism, but they should not distract from determinism/correctness work.

### World / geodesy
- WGS84 ellipsoid origin + local tangent plane (ENU/NED) transforms (replace spherical approximation).
- ECEF representation + consistent gravity direction with latitude.
- Earth rotation / Coriolis (usually small, but can matter for long flights).
- Geoid vs ellipsoid altitude (if you care about matching PX4 SITL assumptions).

### Atmosphere / environment
- Full ISA (multiple layers to 86 km) + humidity (optional).
- Wind shear with altitude, log-law boundary layer profiles.
- Dryden / von Kármán turbulence models (tunable intensity vs altitude/airspeed).
- Thermals + convective updrafts (useful for fixed-wing / soaring).
- Orographic wind / ridge lift (if you simulate terrain).

### Aerodynamics / multirotor specifics
- Rotor induced velocity model (momentum theory) and descent “vortex ring state” approximation.
- Ground effect on thrust (function of height/rotor radius).
- Blade element / BEMT table for thrust/torque vs advance ratio.
- Frame drag with CdA in body axes instead of simple linear drag in NED.

### Sensors / estimation paths
- Raw IMU (accel/gyro) simulation with bias + noise + temperature drift.
- Magnetometer + Earth field model.
- Barometer with noise + lag + bias (altitude estimation).
- GPS noise/latency with discrete update rate (5–10 Hz).
- Option to run EKF2 in PX4 and feed raw sensors (for full-stack testing).

### Powertrain / failures
- Battery temperature-dependent OCV and internal resistance.
- Motor/ESC thermal model + derating.
- Motor failure modes (stuck, partial loss, noise) beyond on/off.
- Prop damage / imbalance (vibration injection to IMU).

### Numerical methods / integration
- Lie group integration for attitude (SO(3)).
- Symplectic/semi-implicit integrator option for better energy behavior.
- Event detection for contact (hit ground exactly) to reduce penetration sensitivity.
- Optional fixed-point iteration for coupled battery–motor electrical loop.

---


---

## P0/P1 fixes implemented (current)

The following P0/P1 items from this review have been implemented in this codebase:

1) **Enforce correct PX4 cadence by default**
   - Added `SimulationConfig.strict_lockstep_rates::Bool = true`.
   - `SimulationInstance` now **throws** when `ap_dt` is slower than the fastest enabled PX4 task rate.
   - Previous behavior (warning-only) is available via `strict_lockstep_rates=false`.

2) **Fix `DelayedEstimator` dt mismatch**
   - `DelayedEstimator` stores `dt_est_us` and validates `dt_hint` at runtime.

3) **Gust boundary semantics + wind sampling**
   - Gusts are half-open (`[t_on, t_off)`), and `SampledWind` holds wind constant across RK4 stages.

4) **Add `time_us` to the log schema**
   - `time_us` is logged in both `SimLog` and `CSVLogSink` (`CSV_SCHEMA_VERSION = 2`).

5) **World origin unification**
   - `WorldOrigin` is shared between PX4 and environment; mismatches are synchronized or warned.

6) **Single-handle guard**
   - `PX4Lockstep.create()` enforces one active handle by default with an `allow_multiple_handles` opt-out.

7) **Precomputed dt coefficients**
   - `AR1`, `AR1Vec`, `OUWind`, and `FirstOrderActuators` cache dt-dependent coefficients.

### Next steps TODO

If you continue with the remaining P1 items from the review, the next steps that will pay off most are:

- **C-side determinism tightening:** phase-lock `StepRateLimiter` (avoid drift/jitter).
- **Battery/prop coupling:** decide on sampled vs instantaneous voltage and close the loop.
- **Battery warning mapping:** verify SOC thresholds vs PX4 params and add a regression test.
- Add explicit **lockstep invariants** around time injection (monotonic `time_us`, expected delta) and make failures loud.
