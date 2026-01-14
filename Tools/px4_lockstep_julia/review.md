## 0) System contract (what the simulator must guarantee)

This is how your code *currently implies* the system is supposed to behave, plus the invariants that should be made explicit.

### Lockstep + determinism contract with PX4

**Goal:** If I run the same scenario twice (same seed, same initial state, same model params, same `libpx4_lockstep` build), I should get the same PX4 outputs and the same simulated trajectory.

Concretely:

* **Single source of simulated time**
  There is a simulated time `t` (seconds) and a corresponding PX4 time `time_us` (microseconds). PX4 must be stepped with **strictly monotonic** `time_us`.

* **Fixed time increment per PX4 step**
  Each call to `px4_lockstep_step()` should advance PX4 by a **constant** `Δt_px4` (in µs). If you choose `Δt_px4 = dt_autopilot`, then **every** PX4 step must be separated by exactly `dt_autopilot` in simulated time.

* **Cadence and replayability**

  * Given identical inputs at each PX4 step (state estimate + flags + battery), PX4 should produce identical actuator outputs.
  * The simulator must call PX4 at a deterministic cadence and must not allow occasional “missed” or “early” calls.

**Best-effort assumption (because it’s ambiguous):**
You’re intentionally running the plant at `dt` (physics) and PX4 at `dt_autopilot` (controller), with ZOH between PX4 updates. That’s a valid contract, but only if `dt_autopilot` is enforced as an **exact integer multiple** of `dt` and you don’t allow drift/jitter.

### State definitions / frames

From your code:

* **World frame:** NED

  * Position `pos_ned = (x_n, y_e, z_d)` in meters.
  * Velocity `vel_ned` in m/s.
  * Gravity is positive down: `g_ned = (0,0,+g)`.

* **Body frame:** FRD (implied throughout, and explicitly in `Propulsion.jl`)

  * +X forward, +Y right, +Z down.

* **Attitude representation:** `q_bn` is quaternion rotating **Body → NED** (w,x,y,z).

  * `quat_to_dcm(q_bn)` returns `R_bn` (Body→NED).
  * `yaw_from_quat(q_bn)` is yaw about NED +Z (down) using the standard PX4-style heading convention.

* **Wind convention:** `wind_velocity()` returns **air velocity relative to ground** in NED.

  * Vehicle air-relative velocity (vehicle relative air) is `v_rel_ned = vel_ned - wind_ned`. (Used for drag in `Vehicles.jl`.)
  * Air velocity relative vehicle is `v_air_ned = wind_ned - vel_ned`. (Used for propulsion inflow in `Simulation.jl` and logged as `air_vel_body` after rotation.)

### Update ordering (per physics tick)

Based on `Simulation.step!`:

1. **Advance environment disturbance state** (e.g., turbulence)
   `step_wind!(wind, pos, t, dt, rng)`.

2. **Scenario step**
   Produces high-level PX4 commands (`arm`, mission/RTL request) and a “landed” boolean.

3. **Battery status sample for PX4**
   `batt = status(battery_model)` (note: this is “previous-step” because you update the battery *after* computing current).

4. **Estimator injection + PX4 step** (multi-rate)
   If PX4 is “due”:

   * `est = estimate!(estimator, rng, t, truth_state, dt_hint)`
   * `out = autopilot_step(px4, t, est, cmd, landed, batt)`

   Otherwise: hold last PX4 output.

5. **Actuator command sanitize + actuator dynamics**

   * clamp motors to [0,1], servos to [-1,1]
   * apply actuator dynamics at physics rate `dt`.

6. **Propulsion update at physics rate**
   Using held motor duties, battery voltage, density, and **air velocity relative vehicle in body frame**, produce thrust/torque/current.

7. **Rigid-body dynamics integration over dt**
   Integrate with Euler/RK4 using **piecewise-constant** forces/torques for this tick (`u_dyn` held over dt).

8. **Battery state update**
   Step battery using the propulsion-computed bus current.

9. **Log** (multi-rate)
   Log truth state (currently: post-integration state but timestamped with the pre-step `t`).

10. **Advance sim time**: `t ← t + dt`

### Multi-rate stepping semantics

Your implementation intends:

* Physics runs at `dt`.
* PX4 is updated at `dt_autopilot` (≥ dt), and PX4 outputs are held constant between updates (ZOH).
* Logging happens at `dt_log` (≥ dt).

**Critical invariant (should be explicit):** `dt_autopilot / dt` and `dt_log / dt` must be integers (or you accept jitter, which breaks strict determinism).

### Estimator injection semantics

* The estimator runs **only when PX4 is stepped** (not every physics tick).
* Noise/bias processes advance at the estimator’s call times (with `dt = t - last_t`).
* Optional delay is implemented as a ring buffer with a fixed sample period `dt_est` and `delay_steps = round(delay_s/dt_est)`.

**Best-effort assumption:** you want estimator noise/bias/delay to be aligned to the PX4 update rate (i.e., estimator sample rate == PX4 step rate), unless you intentionally introduce mismatched rates.

### Ambiguities (but I’ll proceed)

1. **Does PX4 lockstep require being stepped at the physics tick?**
   Your config includes internal PX4 task rates (250 Hz, etc.). It’s unclear whether `libpx4_lockstep` internally catches up tasks if you step it at 100 Hz. I’ll assume it tolerates it, but it’s a correctness risk.

2. **Logging timestamp semantics**
   You log `new_state` with time `t` (pre-step). That is either:

   * an unintended off-by-dt, or
   * an intentional “state at end of interval starting at t”.
     Either way, it must be clearly defined and tested.

3. **Wind OU process discretization**
   Your OU is Euler–Maruyama (dt-dependent). If you truly want deterministic *and* dt-invariant statistics, you should use the exact OU discretization.

---

## 1) Determinism + lockstep audit (highest priority)

### A) Determinism breakers I see in your code

#### 1) **Float-based periodic scheduling causes cadence jitter**

Your `PeriodicTrigger` accumulates `t_next += period` in Float64 and compares against `t`.

That **will drift** over long runs even when `period/dt` is an integer. It’s not theoretical — it shows up in your own log:

* In `sim_log.csv`, `time_s` increments are **mostly 0.010**, but there are **0.008** and **0.012** steps (exactly ±1 physics tick at dt=0.002).

That means your “periodic” logging (and very likely PX4 stepping too) is occasionally running after 4 or 6 physics steps instead of 5.

**Impact:**

* PX4 step cadence is not constant → violates lockstep contract.
* Estimator dt becomes variable → noise/bias evolves with jitter.
* ZOH intervals are inconsistent → controller discretization changes run-to-run if any floating tie breaks differ.

This is a P0.

#### 2) **Simulation time represented as Float64 + additive accumulation**

`sim.t = t + dt` accumulates rounding error. On its own that’s usually fine, but combined with float-trigger scheduling it becomes a determinism hazard.

Even if it’s deterministic on one machine, it is fragile: any refactor that changes evaluation order can change the last-bit rounding and alter when triggers fire.

#### 3) **RNG stream coupling across subsystems**

You correctly pass an `rng` everywhere and seed with `MersenneTwister(cfg.seed)`. Good.

But you use **one RNG stream** for:

* OU wind
* estimator noise
* bias AR(1)
* potentially future noise sources

**Impact:** a tiny change in one subsystem (one extra `randn`) changes the entire future trajectory. This isn’t “nondeterministic” per run, but it’s a determinism *maintenance trap* (regression tests become extremely brittle and “unrelated changes” invalidate golden logs).

#### 4) **OU turbulence discretization is dt-dependent**

Euler–Maruyama changes statistical properties with dt. If someone changes dt (or you add physics sub-stepping), the wind statistics change, and you might incorrectly blame controllers.

This is more “model correctness” than determinism, but it’s a reproducibility trap.

#### 5) **C ABI boundary: layout + lifetime assumptions**

Risks:

* Struct layout mismatch (padding/alignment) between Julia structs and the C ABI.
* Reentrancy: global `_LIB_HANDLE` + `_SYMBOL_CACHE` + handle finalizer.
* Pointer lifetime: you’re good in `step!` because you pass by value/Ref and return a copied `LockstepOutputs`.

What’s missing is a **layout/version handshake**. If the C side changes struct fields, you can get silent corruption.

#### 6) **Logging side effects**

Not a numerical nondeterminism source (since sim time is not wall clock), but heavy allocation/IO increases GC frequency. If any finalizers touch simulation resources (e.g. lockstep handles), you can get weird nondeterministic teardown behavior in test harnesses.

### B) Checklist of invariants to add as asserts/tests

These are “make it impossible to silently drift” checks.

#### Timebase and scheduling invariants

* **`dt > 0` and finite**.
* **`dt_us = round(Int, dt * 1e6)` and `abs(dt_us*1e-6 - dt) < 1e-12`**
  If not, you have a fractional µs step and lockstep will be messy unless you do rational time.
* **`dt_autopilot_steps = dt_autopilot / dt` is integer within tolerance**
  Same for `dt_log`. Enforce at init:

  * `abs(dt_autopilot_steps - round(dt_autopilot_steps)) < 1e-12`
  * then store `ap_steps::Int = Int(round(...))`
* **PX4 step cadence invariant**
  When stepping PX4 at sim step `k`, assert:

  * `time_us == k * dt_us` (or if stepping at ap cadence: `time_us == ap_k * ap_dt_us`)
  * `time_us - last_time_us == ap_steps * dt_us` exactly.
* **Trigger determinism**
  Over long runs, trigger indices must be exactly periodic:

  * For ap_steps=5, triggers at k ∈ {0,5,10,15,...} exactly.

#### RNG invariants

* No use of global RNG (`rand()` / `randn()` without rng). Add a CI grep test for `randn(` not followed by `rng` in your repo.
* Each stochastic subsystem owns its own RNG stream or deterministic substream ID.

#### State validity invariants

* Quaternions always normalized within tolerance:

  * `abs(norm(q_bn) - 1) < 1e-10` after each integration step.
* Finite checks in the hot loop:

  * no NaNs in state, forces, battery voltage/current, motor outputs.
* Motor commands in range:

  * `0 ≤ motors[i] ≤ 1`, `-1 ≤ servos[i] ≤ 1`.
* Battery voltage clamped / non-negative.

#### ABI invariants

* At startup, assert `sizeof(LockstepInputs)` / `sizeof(LockstepOutputs)` matches what the C lib expects (needs a C API hook, see refactors below).

### C) Concrete refactors to guarantee determinism (and “don’t do X” rules)

#### Refactor 1 (P0): **Replace Float64 scheduling with integer step scheduling**

Do not schedule periodic tasks by accumulating `t_next` in Float64.

Instead:

* Maintain:

  * `step::Int` (physics ticks)
  * `dt_us::Int` (integer microseconds per physics tick)
  * `time_us::UInt64 = step * dt_us`
* Compute `t = step * dt` only for convenience/logging, not for scheduling.
* Multi-rate triggers become:

  * `if step % ap_steps == 0` (or equivalently track next_ap_step)

This eliminates drift completely.

#### Refactor 2 (P0): **Define explicit “sample time” semantics**

Pick one and enforce it everywhere:

* **Option A (recommended): “state is at time t”**
  Log the *pre-integration* state with timestamp `t`.
  If you want post-state, use `t+dt`.

* **Option B: “state is at end of interval”**
  Log the *post-integration* state with timestamp `t+dt`.

Right now you log post-state with pre-time. That will poison analysis and any attempt to compare to PX4 internal logs.

#### Refactor 3 (P0/P1): **Split RNG streams per subsystem**

Maintain a deterministic RNG plan, e.g.:

* `rng_wind = MersenneTwister(seed ⊻ 0xWIND…)`
* `rng_est  = MersenneTwister(seed ⊻ 0xEST…)`
* `rng_bias = MersenneTwister(seed ⊻ 0xBIAS…)`

Or use a counter-based RNG / stable “substream” derivation.

Rule: *Adding noise somewhere must not change the wind history*.

#### Refactor 4 (P0/P1): **Add an ABI/version handshake**

Add a small C function in `libpx4_lockstep` like:

* `uint32_t px4_lockstep_abi_version();`
* `size_t px4_lockstep_sizeof_inputs();`
* `size_t px4_lockstep_sizeof_outputs();`

Then in Julia `create()` assert these match `sizeof(LockstepInputs)` etc.

Without this, you’re one PX4 update away from silent breakage.

#### Refactor 5 (P1): **“Don’t do X” rules**

Write these down in CONTRIBUTING.md and enforce with tests:

* Don’t use Float64 time comparisons for cadence (only for reporting).
* Don’t use global RNG.
* Don’t iterate over `Dict`/`Set` in anything that influences physics/IO ordering; if you must, sort keys first.
* Don’t let GC/finalizers drive correctness (always explicitly `close!` handles).
* Don’t call lockstep from multiple threads. Assert `Threads.nthreads() == 1` for this package if you want hard guarantees.

---

## 2) Correctness review by subsystem

### Risk items table

Here’s a concrete risk table (symptom → root cause → test → fix). I’m prioritizing things that can silently be wrong due to frames/units or discrete-time semantics.

| Subsystem                              | Symptom you’d see                                                                                  | Likely root cause in this codebase                                                                     | How to test (specific)                                                                             | Recommended fix                                                                                                                                                              |
| -------------------------------------- | -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Multi-rate scheduling**              | Occasional “hiccups” in PX4 update intervals; controller feels inconsistent; logs show dt jitter   | `PeriodicTrigger` float accumulation (`t_next += period`)                                              | Run 90s, record trigger step indices, assert they’re exact multiples (no ±1 tick slips)            | Use integer step counters for all periodic tasks                                                                                                                             |
| **Logging**                            | First log sample already “moved”; derivative plots look phase-shifted; hard to compare to PX4 logs | Logging post-integration state with pre-step timestamp (`log!(t, new_state)`)                          | Check: at time=0, pos/vel should equal initial; currently it doesn’t                               | Define “sample time” contract; log pre-state at `t` or post-state at `t+dt`                                                                                                  |
| **Rigid-body 6DOF**                    | Yaw/roll/pitch drift, or wrong sign yaw response                                                   | Quaternion convention mismatch vs ω integration / DCM formula                                          | Integrate known constant ω, compare to analytic quaternion; check yaw sign                         | Keep q_bn convention documented; add unit tests for yaw sign and DCM orthonormality                                                                                          |
| **Rigid-body drag**                    | Tilted flight has wrong drag direction; high attitude behaves weirdly                              | Drag computed in NED with linear coefficient, not body-axis aero drag                                  | Simulate 90° roll with forward speed; check drag direction vs body frame                           | If you want simple drag: compute in body frame and rotate back, or explicitly document NED-linear drag assumption                                                            |
| **Integrator (RK4) + quaternion**      | RK4 accuracy lower than expected; small attitude errors accumulate                                 | Normalizing quaternion in intermediate RK4 stages (`rb_add` normalizes) changes RK stages              | Constant ω test: compare RK4 vs exact; look for dt-dependent bias                                  | Use exponential map for attitude integration, or integrate rotation vector separately; normalize only at end-of-step                                                         |
| **Integrator + discontinuities**       | Contacts cause instability or nonphysical bounce with RK4                                          | Contact force discontinuous at z=0, RK4 overshoots                                                     | Drop test from 1m: measure penetration and rebound energy                                          | Consider semi-implicit / symplectic integration for contact or clamp penetration; add damping on both compression/extension                                                  |
| **Actuator dynamics (1st order)**      | Wrong effective time constant; dt changes behavior                                                 | First-order update uses `α = clamp(dt/τ,0,1)` (Euler-ish), not exact                                   | Step response: compare to analytic `1-exp(-t/τ)`; allow <1% error                                  | Use exact discretization `α = 1 - exp(-dt/τ)`                                                                                                                                |
| **Actuator dynamics (2nd order)**      | Overshoot beyond limits; instability if dt large                                                   | Semi-implicit Euler without output clamping                                                            | Step input with ω_n high and dt near stability limit                                               | Add explicit output saturation + rate limiting; optionally discretize with Tustin for stability                                                                              |
| **Propulsion duty→ω→thrust**           | Thrust vs duty scaling wrong; hover duty weird                                                     | Motor constants / units mismatch (Kv, Ke, Kt), or ESC model mismatch                                   | Static check: at ω_hover and ρ, thrust formula must match calibrated thrust                        | Keep calibration test: `prop_thrust(ω_hover)=T_hover` and `prop_torque(ω_hover)=Q_hover` within 1e-9                                                                         |
| **Propulsion inflow**                  | Inflow correction “does nothing” or wrong sign; ascent/descent symmetry unintended                 | Uses `mu^2` → symmetric; `v_air_body` is **air velocity relative vehicle**, not “vehicle relative air” | Construct simple case: vel down 1 m/s, wind 0, verify Vax positive and matches convention          | Rename variables (`v_air_rel_vehicle_body`), document sign; if you need descent-only effects, make factor sign-aware                                                         |
| **Propulsion energy consistency**      | Battery power doesn’t match mechanical power trends; throttle cuts feel wrong                      | Clamp motor current to ≥0 → no regen; energy removed with no accounting                                | Spin at ω, set duty=0: ω decays but battery current stays ~0 (ok) — but energy accounting mismatch | Either (a) explicitly accept “no regen” and add dissipative sink model, or (b) allow negative current and route to battery/sink                                              |
| **Reaction torque / rotor directions** | Yaw response inverted or unstable                                                                  | Wrong `rotor_dir` sign or mapping from PX4 motor order to geometry                                     | Apply +Δ to yaw command (or differential motor commands) and check yaw acceleration sign           | Add a “sign test” harness; document motor order mapping; enforce with unit tests                                                                                             |
| **Battery Thevenin**                   | Voltage sag unstable with stiff parameters; SOC drift                                              | Parameter unit mismatch or stiff RC if τ small; SOC clamp hides issues                                 | Constant current discharge: compare to analytic V(t) and SOC(t)                                    | Add parameter validation (R,C>0); use exact RC discretization (you already do); add tests                                                                                    |
| **Battery injection timing**           | PX4 sees delayed voltage sag; low-voltage failsafe late/early                                      | `status(battery)` sampled before stepping battery for current of this tick                             | Compare logged batt_v vs current: sag appears one tick late                                        | Decide explicitly: “battery is sampled at start of tick” vs “instantaneous”; if needed, predict sag using current from previous tick or do a fixed-point iteration (careful) |
| **Environment OU wind**                | Wind variance changes when dt changes; “turbulence strength” depends on step size                  | Euler–Maruyama discretization                                                                          | Run long sim at dt=0.01 and dt=0.02; compare var(v_gust)                                           | Use exact OU discretization: `v=ϕ v + σ sqrt(1-ϕ²) ξ` (same as your AR(1))                                                                                                   |
| **GustStep wrapper**                   | GustStep(mean=OUWind) produces constant (non-evolving) mean turbulence                             | `step_wind!(::GustStep)=nothing` doesn’t delegate to mean                                              | Wrap OUWind in GustStep and check v_gust evolves over time                                         | Implement `step_wind!(w::GustStep, ...) = step_wind!(w.mean, ...)`                                                                                                           |
| **Contacts**                           | Vehicle penetrates ground too much or oscillates                                                   | Penalty spring/damper tuning; damping only on compression                                              | Drop test: max penetration and settle time                                                         | Add symmetric damping and/or velocity projection; enforce stable penetration bounds                                                                                          |
| **Estimator injection**                | Delay off by one; noise rate mismatch; bias stats wrong                                            | Delay steps computed by rounding; estimator only steps at PX4 rate; dt jitter from triggers            | Feed a ramp truth and verify output is exactly delayed by N samples; check noise PSD vs expected   | Enforce `delay_s = N*dt_est` exactly; align estimator update steps to PX4 schedule using integer ticks                                                                       |
| **PX4 bridge mapping**                 | Mission works “most of the time” but odd nav transitions                                           | Missing/incorrect required signals or wrong landed behavior                                            | Compare PX4 uORB expectations: does it need additional flags/sensor validity?                      | Add a “bridge conformance” test suite: required fields present, consistent frames, monotonic time_us                                                                         |

### Specific “golden tests” (unit + integration) — at least 10, with signals and tolerances

I’m listing tests that you can actually automate and that will catch the frame/sign bugs you *will* otherwise rediscover later.

#### Timebase / scheduling (P0)

1. **Periodic cadence test (no drift)**

* Setup: dt=0.002, ap_steps=5, log_steps=5, run step counter to 100k.
* Signals: indices where PX4/log triggers fire.
* Assert: `trigger_steps == 0:5:...` exactly (integer equality).
* Tolerance: 0.

2. **PX4 time_us monotonic + constant increment**

* Setup: step PX4 for 10k controller updates.
* Signals: `time_us` passed to lockstep.
* Assert: strictly increasing; `Δtime_us == ap_steps * dt_us` every call.
* Tolerance: 0.

3. **Log timestamp semantics test**
   Pick your contract and enforce it:

* If logging pre-state at `t`: first sample must equal initial state exactly.
* If logging post-state at `t+dt`: first sample time must be `dt` and state must match 1-step integration.
* Tolerance: exact equality for time stamps; state tol ~1e-12 for floats if deterministic.

#### Rigid-body + integrators (P0/P1)

4. **Free-fall analytic check**

* Setup: no thrust, no drag, no wind, no contact, initial pos=0, vel=0.
* Expected: `vel_z(t)=g t`, `pos_z(t)=0.5 g t^2` in NED.
* Signals: pos_z, vel_z at t=1.0.
* Tolerance:

  * RK4 with dt=0.01: pos error < 1e-6 m, vel error < 1e-6 m/s
  * Euler: looser, but still bounded/predictable.

5. **Constant-rate quaternion integration**

* Setup: ω_body = (0,0,π) rad/s, start q=identity, integrate 1.0 s.
* Expected: yaw = π rad (mod wrap), quaternion near `(0,0,0,1)` (or sign-flipped).
* Signals: `q_bn`, `yaw_from_quat(q_bn)`.
* Tolerance: angle error < 1e-4 rad; `|‖q‖-1| < 1e-12`.

6. **DCM orthonormality invariant**

* Setup: random q, compute R=quat_to_dcm(q).
* Assert: `R*R'` ≈ I, det(R) ≈ +1.
* Tolerance: `‖R*R' - I‖_F < 1e-12`, `|det(R)-1| < 1e-12`.

#### Actuators (P1)

7. **First-order actuator exact step response**

* Setup: τ=0.1, dt=0.01, cmd steps 0→1 at t=0, simulate 1s.
* Expected: y(t) = 1 - exp(-t/τ).
* Signal: y[k].
* Tolerance: max abs error < 1e-4 (with exact discretization it’ll be much smaller).

#### Propulsion / frames / signs (P0/P1)

8. **Air-relative velocity convention test**

* Setup: wind=0, q=identity, vel_ned=(0,0,+1) m/s (descending).
* Expected: `v_air_ned = wind - vel = (0,0,-1)`; `v_air_body.z = -1`; `Vax = -v_air_body.z = +1`.
* Signals: computed `v_air_body`, computed Vax in propulsion.
* Tolerance: exact equality for these small integers (within 1e-12).

9. **Prop calibration identity**

* Setup: default Iris prop params. Compute thrust at ω_hover and ρ=1.225.
* Expected: `T(ω_hover)=thrust_hover_per_rotor` exactly (since you calibrate kT that way).
* Signal: prop_thrust result.
* Tolerance: < 1e-12 relative error.

10. **Yaw torque sign test**

* Setup: hover state, inject differential motor duties that should produce +yaw moment (per your rotor_dir convention).
* Expected: ω_dot_z sign (body) matches convention you document; yaw acceleration sign matches.
* Signal: `τ_body.z`, `ω_dot.z`.
* Tolerance: sign only (≠0 and correct sign).

#### Battery / powertrain (P1)

11. **Thevenin transient analytic check**

* Setup: constant current draw I=10A for 1s, known R0,R1,C1, constant OCV.
* Expected: `v1(t)=I*R1*(1-exp(-t/τ))`, `V=OCV - I*R0 - v1`.
* Signal: battery voltage after 1s.
* Tolerance: < 1e-4 V.

12. **SOC coulomb counting**

* Setup: capacity=5Ah, discharge at 5A for 3600s equivalent (scaled test time).
* Expected: SOC drops by exactly 1.0 over 1 hour at 5A.
* Signal: SOC.
* Tolerance: < 1e-6.

#### Wind / estimator (P1)

13. **OU stationary variance dt-invariance**

* Setup: σ=1, τ=5s. Run long sequence at dt=0.01 and dt=0.02.
* Expected: var(v_gust) ≈ σ² in both cases (after burn-in) and the two variances are within ~5%.
* Tolerance: |var-1| < 0.05; |var_dt1-var_dt2| < 0.05.

14. **Delay buffer exactness**

* Setup: delay_s=0.03, dt_est=0.01 → 3 steps. Feed deterministic ramp.
* Expected: output = input shifted by exactly 3 samples.
* Tolerance: 0 (exact) for positions if you use isbits values.

---

## 3) Architecture / API design review

### Keep / Change / Remove

#### Keep

* **Subsystem separation in `Sim/`** (Types, RigidBody, Environment, Propulsion, Powertrain, Estimators, Contacts, Scenario, Events). This is the right level of modularity for maintainable flight sim code.
* **Single-threaded, fixed-step core**. Determinism is dramatically easier here.
* **StaticArrays for small vectors/quats**. Good for performance and clarity.

#### Change

* **Timebase abstraction**: Float64 time as the “truth” is the root cause of your cadence drift. Replace with integer ticks.
* **Multi-rate scheduler**: stop using `PeriodicTrigger` with float accumulation.
* **Autopilot output typing**: `last_out::Any` is both a performance problem and a correctness risk (it hides type errors and encourages “just getproperty whatever”). Make outputs concrete.
* **Wind OU discretization**: re-use your AR(1) exact form to make it dt-stable.
* **Gust wrapper semantics**: if you keep `GustStep`, it must delegate stepping to the wrapped wind model.

#### Remove (or quarantine)

* **Including Sim in the base wrapper module** if you truly want a dependency-light wrapper. Right now, `PX4Lockstep` can’t be used without pulling in Sim codepaths at load time.
* **Float-time periodic triggers** (full removal, not “tweak eps”).

### Module boundaries / dependency direction

You said “wrapper should stay small” — but today the package-level dependencies include StaticArrays, Random, Printf, Aqua, JET, JuliaFormatter. The wrapper isn’t actually light in practice.

If you care about that contract, use **Julia package extensions**:

* Base package `PX4Lockstep` contains only the C ABI wrapper (Libdl only).
* `ext/PX4LockstepSimExt.jl` defines `PX4Lockstep.Sim` and brings in StaticArrays/Random/Printf/etc.

That gives you a clean dependency direction without a separate repo.

### Type design and dispatch patterns

* `SimulationInstance{E,V,A,EST,I,S,B,L,C,R}` is aggressively parametric. That can be good for performance, but compile-time might get painful as you add options.
* The bigger issue is `last_out::Any`. That defeats specialization at exactly the spot you don’t want it.

**Fix:** make `SimulationInstance` parametric on output type, or store:

* `last_out::Union{Nothing,LockstepOutputs}` for PX4
* or define an `AbstractAutopilotOutput` trait.

### Data ownership / mutation

You’re mostly doing the right thing:

* Immutable “params”, mutable “state” (vehicle state, propulsion ω, battery state).
* But you still allocate in hot loops (propulsion arrays, closure creation). That will force GC eventually.

### Configuration ergonomics

Overall decent (`@kwdef` structs). A few improvements that pay off:

* Enforce all discrete-time relationships at construction (dt ratios must be integer).
* Make “model wiring” explicit and validated: if a vehicle requires propulsion, make it non-optional in that vehicle type rather than runtime `error`.

### Logging API

Current logging is explicit and fast-ish, but schema evolution will hurt:

* `_CSV_HEADER` must be updated manually.
* log fields are positional in CSV and must remain aligned.

**Better pattern:**
Versioned schema + column selection:

* `LogSchema(version=1, columns=[...])`
* sink writes header including schema version
* add fields in a backward-compatible way.

Also: decide the sample-time contract (see P0).

### Error handling / diagnostics

* Some “silent” returns (e.g., failure injection ignoring bad indices) should become warnings in debug mode.
* Add `@assert` guards behind a `cfg.debug::Bool` or compile-time flag so you can flip between “fast” and “paranoid”.

### 3–5 concrete API changes that will pay off

1. **Introduce `SimTime`**

```julia
struct SimTime
    step::Int
    dt::Float64
    dt_us::Int
end
time_s(st::SimTime) = st.step * st.dt
time_us(st::SimTime) = UInt64(st.step * st.dt_us)
```

Use `step` for scheduling, `time_s` only for reporting.

2. **Replace `PeriodicTrigger` with `StepTrigger`**

```julia
struct StepTrigger
    period_steps::Int
end
due(tr::StepTrigger, step::Int) = (step % tr.period_steps == 0)
```

3. **Make autopilot output type explicit**

* `autopilot_step(::PX4LockstepAutopilot, ...)::LockstepOutputs`
* store `last_out::Union{Nothing,LockstepOutputs}` (no `Any`)

4. **Change `step_integrator` API to avoid per-tick closure allocation**
   Pass a callable struct (functor) or the simulation + state directly:

```julia
step_integrator!(integrator, sim, state, u_dyn, t, dt)
```

so you don’t create a new closure each tick.

5. **Standardize naming for air-relative velocity**
   Rename:

* `v_air_ned` → `v_air_rel_vehicle_ned` (air velocity relative vehicle)
* `v_rel_ned` → `v_vehicle_rel_air_ned` (vehicle relative air)

This removes a whole class of sign bugs.

---

## 4) Performance + numerical stability review (determinism-preserving)

### What I would profile in Julia (and what to look for)

Even without threads, performance matters because GC pressure and unpredictable allocations make debugging miserable.

Use:

* `@time` / `@allocated` around a loop of `step!` (10k steps)
* `@code_warntype step!(sim)` to catch red `Any`
* `Profile.@profile` + `Profile.print()` for hotspots
* `BenchmarkTools.@btime` for microbenching (dev dependency is fine)

**Signatures to look for:**

* Allocations per tick > 0 (ideally near-zero in the core loop)
* Type instability: `Any`, `Abstract...` containers in hot loops
* Closure allocations (anonymous function created each tick)
* Repeated heap allocations from `zeros(...)` in propulsion

### Concrete allocation hot spots in your code

* `Propulsion.step_propulsion!` allocates `zeros(Float64,N)` arrays every tick (`omegas`, `thrusts`, `torques`).
  At 500 Hz this is guaranteed GC churn.

* `Simulation.step!` builds an anonymous function `f = (τ,state,uu)->...` every tick.
  That is very likely allocating (and even if it’s optimized away sometimes, don’t rely on it).

* `last_out::Any` forces dynamic dispatch / boxing when you do `getproperty(out, ...)`.

* `SimLog` pushes onto growing vectors; if you know the run length, preallocate capacity (or allocate exact length if fixed).

### Numerical stability concerns

* **RK4 + discontinuities** (contacts, saturation, gust on/off) can be worse than Euler because it samples the discontinuity multiple times in one step and can overshoot. If you care about contact realism, consider:

  * event detection (stop integration at contact),
  * or a contact-stable integrator (semi-implicit) for the contact axis,
  * or increase damping and clamp penetration.

* **Actuator and motor integration** is explicit Euler. Stability is dt-dependent. Add guards:

  * assert `dt < 0.2 * τ_motor` or similar heuristic,
  * or use exact discretization where possible (first-order states).

* **Battery RC** can get stiff if τ is small. Your exact discretization for v1 is good; keep that.

All fixes above preserve determinism because they are purely algorithmic and single-threaded.

---

## 5) Prioritized improvement backlog

### P0 (must-fix for correctness + determinism)

**P0-1: Replace Float64 periodic triggers with integer step scheduling**

* **Rationale:** Your own log shows cadence jitter (0.008/0.012 instead of 0.010). That breaks lockstep determinism and controller semantics.
* **Plan:**

  * Add `step::Int` to `SimulationInstance`
  * Precompute `ap_steps`, `log_steps` as integers at init; assert they’re valid
  * Replace `PeriodicTrigger` with step modulo or next-step counters
* **Effort:** M
* **Acceptance criteria:**

  * Over 100k physics steps, PX4 is stepped exactly every `ap_steps` with zero jitter.
  * Log sample times are exactly periodic (no ±dt slips).
  * Regression: `unique(diff(time_s)) == {dt_log}` exactly.

**P0-2: Define and enforce log sample-time semantics**

* **Rationale:** You currently log post-step state with pre-step timestamp; that is analytically wrong and will mask/control bugs.
* **Plan:**

  * Decide: log pre-state at `t` OR post-state at `t+dt`
  * Update `Simulation.step!` and plotting scripts accordingly
  * Add a unit test that checks the first sample matches initial conditions (or has correct timestamp)
* **Effort:** S
* **Acceptance criteria:**

  * First log sample is consistent with declared semantics.
  * A free-fall analytic test passes using log values without extra manual time shifts.

**P0-3: Make PX4 stepping cadence and `time_us` increment exact**

* **Rationale:** Lockstep libraries typically assume deterministic time increments; jitter defeats the whole point.
* **Plan:**

  * Derive `time_us` from integer tick counters, not from Float64 `t*1e6`
  * Assert `time_us` increments exactly by constant Δ each PX4 step
* **Effort:** S
* **Acceptance criteria:**

  * `Δtime_us` is constant across the entire run (exact integer equality).

**P0-4: Add ABI/layout handshake for LockstepInputs/Outputs**

* **Rationale:** Silent corruption risk whenever the C ABI changes.
* **Plan:**

  * Add C-side functions for ABI version and struct sizes (if you own that lib)
  * Assert in Julia at `create()`
* **Effort:** M (depends on C side access)
* **Acceptance criteria:**

  * Wrapper throws a clear error if ABI mismatches rather than running with garbage.

### P1 (high payoff, reduces “future debugging tax”)

**P1-1: Split RNG streams per subsystem**

* **Rationale:** Keeps regression tests stable; avoids “change wind because I added sensor noise”.
* **Plan:**

  * Add `rng_wind`, `rng_est`, `rng_noise` fields
  * Seed via deterministic mixing of `cfg.seed`
* **Effort:** S
* **Acceptance criteria:**

  * Adding a new noise source does not change wind history (verified by wind-only golden test).

**P1-2: Replace OUWind Euler–Maruyama with exact OU discretization**

* **Rationale:** dt-dependent wind stats is a correctness trap.
* **Plan:**

  * Use `ϕ = exp(-dt/τ)` and `v = ϕ v + σ*sqrt(1-ϕ^2)*ξ`
  * Reuse your existing AR(1) pattern
* **Effort:** S
* **Acceptance criteria:**

  * Stationary variance matches σ² within 5%.
  * Variance is stable when dt changes (dt-invariance test).

**P1-3: Fix GustStep stepping delegation**

* **Rationale:** `GustStep(mean=OUWind)` is currently broken (mean never advances).
* **Plan:** implement `step_wind!(w::GustStep, ...) = step_wind!(w.mean, ...)`
* **Effort:** XS
* **Acceptance criteria:** wrapping OUWind in GustStep still produces evolving turbulence.

**P1-4: Remove `last_out::Any` (type stability)**

* **Rationale:** `Any` in the hot loop kills specialization and hides interface mistakes.
* **Plan:** make it `Union{Nothing,LockstepOutputs}` (for PX4 sim) or parametric output type.
* **Effort:** S
* **Acceptance criteria:**

  * `@code_warntype step!(sim)` shows no `Any` on the critical path.

**P1-5: Kill per-tick allocations in propulsion**

* **Rationale:** `zeros(...)` every tick will force GC churn; hurts performance and can complicate determinism in test harnesses.
* **Plan:**

  * Use `NTuple{N,Float64}` or `MVector{N,Float64}` buffers stored in the propulsion state
  * Return `SVector` without allocating intermediate arrays
* **Effort:** M
* **Acceptance criteria:**

  * `@allocated` for 10k `step!` iterations is ~0 (or a small constant not proportional to steps).

**P1-6: Avoid per-tick closure creation in integrator call**

* **Rationale:** likely allocates; also harder to reason about type stability.
* **Plan:** pass a functor or call a dedicated `dynamics_step!(...)`.
* **Effort:** M
* **Acceptance criteria:** no closure allocations; simpler `@code_warntype`.

**P1-7: Actuator model discretization upgrades**

* **Rationale:** wrong time constants when dt changes; can destabilize control tuning.
* **Plan:** use exact discretization for 1st order; clamp outputs for 2nd order.
* **Effort:** S
* **Acceptance criteria:** step response matches analytic within 1e-4.

**P1-8: Document and test rotor ordering + yaw torque sign**

* **Rationale:** classic “it flies but yaw is weird” pitfall; hard to debug later.
* **Plan:** add a dedicated test that applies known differential thrust and checks yaw acceleration sign and magnitude.
* **Effort:** S
* **Acceptance criteria:** sign test passes and is locked in CI.

### P2 (nice to have, but don’t distract from core determinism/correctness)

**P2-1: Battery sensing model (explicit sampling delay + noise)**

* **Rationale:** More realistic than accidental 1-tick delay.
* **Plan:** model a voltage/current low-pass + sensor delay explicitly, and decide what PX4 sees.
* **Effort:** M
* **Acceptance criteria:** battery measurement dynamics are explicit and tested.

**P2-2: More realistic drag/aero model (body-axis, quadratic)**

* **Rationale:** Current NED-linear drag is “toy” level; might be ok for now.
* **Effort:** M/L
* **Acceptance criteria:** body-axis aero drag produces expected behavior in tilted flight tests.

**P2-3: Contact model upgrades (moment, friction cone, static friction)**

* **Rationale:** Only needed if you care about landing/ground handling realism.
* **Effort:** L
* **Acceptance criteria:** stable landing + taxi scenarios with bounded penetration.

---

## Fast wins in a weekend (highest ROI)

1. **Integer step scheduling + `time_us` from ticks** (P0-1 + P0-3)
2. **Fix logging timestamp semantics** (P0-2)
3. **Remove `last_out::Any`** (P1-4)
4. **Fix GustStep stepping** (P1-3)
5. **Exact OU discretization** (P1-2)
6. Add 5 core tests in CI:

   * scheduler cadence
   * time_us increments
   * free-fall
   * quaternion constant-rate
   * delay buffer exactness

These will dramatically improve confidence and make every future change cheaper.

---

## Things not worth doing yet (until P0/P1 are done)

* Full Dryden/von Kármán turbulence / spatially correlated wind fields
* Full IMU + baro + GPS sensor simulation feeding EKF2 (you intentionally bypassed it; keep the scope tight)
* Terrain meshes, multi-contact landing gear, restitution modeling
* Multi-threading for performance (it will destroy your determinism story unless you’re extremely disciplined)
* Switching to a big ODE framework (adds dependency weight + hides ZOH/multi-rate semantics unless you’re careful)

---

If you want one “north star” principle to keep this project sane: **use integers for simulated time and scheduling, and treat Float64 time as a derived view.** Everything else (PX4 lockstep cadence, estimator delay correctness, log alignment, regression stability) gets much easier once you do that.


---

# Update: Implemented fixes (Jan 2026)

This section records what was implemented in response to the review.

## ✅ Done

### P0 / determinism contract

1) **Integer-step scheduling for multi-rate tasks (no Float64 drift)**
   - Replaced `PeriodicTrigger` with `Scheduling.StepTrigger`.
   - `SimulationInstance` now schedules autopilot + logging with `due(trig, step)` where `step` is an integer physics tick.
   - `dt_autopilot` and `dt_log` are **required to be integer multiples of `dt`** (enforced).

2) **Log sample-time semantics are now defined and consistent**
   - The logging contract is now: **log the pre-step state `x_k` at time `t_k`**.
   - In the engine, logging happens after the physics update (for convenience), but it logs the saved snapshot `x0` and `t` from the start of the tick.
   - This fixes the previous “post-state with pre-time” inconsistency.

3) **PX4 time injection uses an exact microsecond clock**
   - `SimulationInstance` derives `time_us` from the integer step counter:
     `time_us = t0_us + step * dt_us`.
   - `dt` is enforced to be **exactly representable as an integer number of microseconds**.
   - `t0` is enforced to be **non-negative and microsecond-quantized**.

### P1 / high-signal realism + performance / maintenance

4) **RNG streams split by subsystem**
   - Simulation now creates independent RNG streams (`rng_wind`, `rng_est`, `rng_misc`) derived from the base seed using SplitMix64.
   - This prevents “noise coupling” when adding/removing randomness in one subsystem.

5) **OU wind update is now the exact discretization**
   - `OUWind.step_wind!` now uses:
     `v[k+1] = ϕ v[k] + σ sqrt(1-ϕ^2) ξ`, `ϕ = exp(-dt/τ)`.
   - This preserves the intended stationary variance across different `dt`.

6) **GustStep now advances its wrapped mean wind**
   - `GustStep.step_wind!` delegates to `step_wind!(mean, ...)`.

7) **Removed the `last_out::Any` type-instability trap**
   - Added `Autopilots.autopilot_output_type(ap)`.
   - `SimulationInstance` stores `last_out::Union{Nothing,O}` where `O` is the autopilot’s concrete output type.

8) **Removed per-tick propulsion allocations**
   - `Propulsion.step_propulsion!` no longer allocates heap arrays each tick; it uses `MVector`/`SVector` buffers.

9) **Removed per-tick closure allocations in integrator calls**
   - `SimulationInstance` caches a `DynamicsWithContact` functor and passes it to the integrator.

10) **First-order actuator dynamics uses the exact ZOH discretization**
   - `FirstOrderActuators` now uses `α = 1 - exp(-dt/τ)`.

11) **Log preallocation**
   - Added `Logging.reserve!(log::SimLog, n)` and the engine calls it based on `t_end/dt_log`.

## ✅ Tests added/updated

- Updated scheduling test to `StepTrigger`.
- Added regression tests for:
  - `GustStep` stepping delegation.
  - Exact first-order actuator discretization.
  - Engine microsecond clock increments and log pre-step semantics using a dummy autopilot.

*(Note: tests could not be executed in the current environment because Julia was not available, but they should run in a normal Julia dev environment.)*

## 🔜 Next (recommended)

### ABI / integration safety

- **Add ABI compatibility handshake between Julia ↔︎ libpx4_lockstep**
  - Add exported functions on the C/C++ side (e.g. `px4_lockstep_abi_version()`) and require the Julia side to validate before stepping.

### Additional high-signal tests

- Add analytic/closed-form regression tests:
  - free-fall under gravity (pos/vel vs closed form)
  - constant-rate quaternion integration (small-angle and longer time)
  - delay buffer edge cases (DelayedEstimator)
  - cadence tests: verify exact autopilot/log firing times over long horizons

### Battery/prop sampling semantics (make it explicit)

- Decide and document whether battery voltage sag/current draw are “sample-at-start-of-tick” or “sample-at-end-of-tick”.
- If needed, consider a predictor/corrector update for battery voltage within a tick to remove the 1-tick lag.

### Wind model roadmap

- Once OU is stable and deterministic, add a Dryden-ish or von Karman turbulence option (still seeded and stateful).

