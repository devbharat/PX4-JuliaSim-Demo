# Verification Plan

This document is the **canonical plan** for expanding verification coverage.
It is intentionally written in terms of *deterministic, auditable checks* that
catch the most common (and most expensive) simulator bugs: sign/frames issues,
coupling mistakes, and hybrid engine boundary semantics.

This plan is organized as:

* **A. System-level verification cases**: cheap deterministic checks that exercise
  multiple subsystems together (plant RHS + environment + powertrain + runtime ordering).
* **B. Missing unit-test coverage**: targeted component tests for models with high bug risk.
* **C. Verification ladder**: a recommended tiering so CI stays fast while still guarding
  correctness.
* **D. PX4 bridge correctness**: explicit note that PX4 field mapping needs its own
  verification approach.

---

## A. System-level verification cases worth adding

These are “system-level” in the sense that they exercise **multiple submodels together**
(plant RHS + environment + powertrain/propulsion + engine boundary semantics), but remain
deterministic and cheap enough to run frequently.

### 1) Full-plant “ballistic” regression (no thrust, no wind)

**Goal:** Verify frames/signs + gravity + integrator + engine event stepping.

**Setup**

* Use the full `PlantModels.CoupledMultirotorModel` (Iris default vehicle + battery),
  but force motor duties = 0.
* Wind = zero, no contact.
* Initial state: altitude 10 m (in NED that’s `pos_ned[3] = -10`), vertical velocity 0,
  identity attitude.
* Run with the canonical engine and a fixed timeline (dt_ap/log/wind arbitrary).

**Checks**

* Compare `z(t)` and `vz(t)` to analytic free fall:

  * `vz(t) = g*t` (NED +down), `z(t) = z0 + 0.5*g*t^2`.
* Tolerances:

  * RK4 dt=0.01: expect cm-level at 1–2s; with RK45 tight: ~machine eps.
* Also assert invariants every log tick:

  * quaternion norm ~ 1
  * rotor ω ≥ 0, SOC ∈ [0,1], finite V/I

**Why this catches bugs**

* Any sign mistake in gravity, NED/body, or thrust direction shows up immediately.

---

### 2) Hover equilibrium “static acceleration” check (force balance at t=0)

**Goal:** Verify thrust direction and magnitude mapping into rigid-body forces (frames!).

This is a *contract test*, not a long simulation.

**Setup**

* Build the Iris plant model at sea-level density, zero wind, identity attitude, zero
  velocities.
* Choose a rotor ω state (or duty) that gives **exactly** `T_total = m*g`.

  * This can be computed with a quick root solve on ω using the prop model
    (`prop_thrust ~ kT ρ ω²`), or by using the calibrated “2× hover thrust at duty=1”
    relationship to find a duty in-range.

**Check**

* Evaluate the RHS once:

  * `ẋ.rb.vel_dot[3]` should be ~0 (vertical accel in NED)
    i.e. `a_down ≈ g - T_total/m ≈ 0`
* Also check lateral accelerations are ≈0 (symmetry):

  * `|ax|, |ay| < 1e-9` for a symmetric rotor set at identity attitude.

**Why this catches bugs**

* This catches the classic “thrust is applied in the wrong direction / wrong frame /
  wrong sign” bug without needing a closed-loop run.

---

### 3) Quad symmetry torque test (roll/pitch cancel, yaw matches rotor_dir)

**Goal:** Verify torque aggregation + rotor direction conventions.

**Setup**

* Identity attitude, zero wind, zero rates.
* Equal rotor speeds on all 4 rotors.

**Checks**

* Net body torque should satisfy:

  * roll ≈ 0, pitch ≈ 0
  * yaw torque equals the sum of rotor reaction torques with `rotor_dir` signs (nonzero
    unless model cancels).

If exact magnitudes are not required, check **sign**:

* Flip `rotor_dir` signs → yaw torque should flip sign.

**Why this catches bugs**

* Rotor direction sign mistakes are extremely common and often only show up under yaw
  maneuvers.

---

### 4) Thevenin battery step-load analytic test (component + coupled)

**Goal:** Verify battery ODE (SOC + V1) and terminal voltage equation.

**Two variants**

1. **Battery-only**: integrate the Thevenin ODE under constant current `I`.

   * Closed form exists for V1:

     * `V1(t) = V1(0)*exp(-t/τ) + I*R1*(1-exp(-t/τ))`, τ=R1*C1
   * SOC linear: `SOC(t) = SOC0 - I*t/Q`
2. **Plant-coupled**: run the plant RHS with “motors drawing constant current” replaced
   by a synthetic fixed bus current (or use a simple “constant duty with fixed ω” in a
   stripped plant model).

**Checks**

* Compare integrated `v1` and `soc` to analytic at log ticks.
* Compare `V_bus = ocv(soc) - v1 - R0*I` (with clamp) to computed terminal voltage.

**Why this catches bugs**

* Battery models are easy to subtly unit-mess (Ah vs Coulombs, RC time constant,
  sign conventions).

---

### 5) Bus-solve residual test (algebraic consistency)

You already have analytic bus solve + deterministic fallback. Good. Now **verify it**.

**Setup**

* Pick random but deterministic (seeded) cases over a safe envelope:

  * `soc ∈ [0.2, 1.0]`, `v1 ∈ [-0.5, 0.5]`
  * `ω ∈ [0, ω_max]`, `duty ∈ [0,1]`
* For each case, call your `_solve_bus_voltage(...)`.

**Checks**

* Verify the solved V satisfies the fixed-point equation:

  * `V ≈ clamp(V_min, ocv(soc) - v1 - R0*I_total(V))`
* Assert residual < `1e-6…1e-4` (depending on model conditioning).
* Assert monotonic sanity:

  * Increasing duty should not *increase* V_bus (should generally sag more) in regions
    without clamps.

**Why this catches bugs**

* Any change in motor current model or deadzone handling can break solver assumptions
  silently.

---

### 6) OU wind “discretization contract” test

**Goal:** Verify the OU discretization is actually the exact discrete update and is
dt-consistent.

**Setup**

* Set OUWind with known τ and σ, and use a deterministic RNG.
* Run one step and verify the recurrence:

  * `v_next = φ v + σ sqrt(1-φ²) ξ`
  * where `φ = exp(-dt/τ)`

**Checks**

* Confirm `φ` and `scale = sqrt(1-φ²)` match computed values.
* Confirm gust update uses those.

**Add-on**

* A *statistical* check can be an example script (not a unit test):

  * long run, verify sample variance ~ σ² and autocorrelation ~ exp(-Δt/τ)

---

### 7) Engine boundary ordering correctness test

This is system-level determinism, not physics, but it prevents “engine drift.”

**Setup**

* Create tiny “probe” sources:

  * Scenario source sets `bus.marker = 1`
  * Wind source asserts it sees marker==1 then sets marker=2
  * Estimator asserts marker==2 then sets marker=3
  * Autopilot asserts marker==3 then sets marker=4
* Run a short timeline and ensure ordering is enforced at every boundary.

**Why this catches bugs**

* If boundary ordering accidentally changes, record/replay comparisons become
  meaningless.

---

### 8) Fault semantics tests (motor disable / battery disconnect / sensor failure)

Fault behavior should be **defined as an engine + plant contract** so it’s testable and stable.

**Motor disable**

* Contract: `FaultState.motor_disable_mask` forces the corresponding motor duty input to **0.0** *before* motor/ESC evaluation.
* Immediate consequences (at that same RHS evaluation):

  * motor current for the disabled motor clamps to **0 A**
  * electrical torque is **0 Nm**
  * rotor angular acceleration is negative if ω>0 due to prop load + viscous friction
  * thrust remains a function of ω and therefore **decays as ω spins down** (it is not forced to 0 instantaneously)

**Battery disconnect**

* Contract: `FaultState.battery_connected == false` forces the electrical bus to an “open” state:

  * `bus_voltage_v = 0.0`
  * `bus_current_a = 0.0`
  * all motor currents clamp to **0 A**

* Mechanical consequence: rotors spin down due to prop load + friction.

**Sensor failure / estimator freeze**

* Contract: `SENSOR_FAULT_EST_FREEZE` makes `LiveEstimatorSource` **hold last estimate** (it does not update `bus.est` while the bit is set).

---

### 9) Iris mission open-loop local defect test

This is the most important “system-level integrator verification” in the presence of
open-loop divergence.

Instead of measuring **cumulative divergence**, measure **per-interval map error**:

For each log interval `[t_k, t_{k+1}]`:

* Start both solvers from the *same* state `x_ref(t_k)`
* Integrate to `t_{k+1}` using the same held inputs (from recording)
* Error = `x_test(t_{k+1}) - x_ref(t_{k+1})`

This error stays interpretable and should scale with solver order/tolerances.

**Checks**

* Plot or record `max(error)` over time (it should not “blow up” purely due to
  compounding).
* Step-halving convergence:

  * RK4 dt_phys → dt_phys/2 should reduce local error ~16× (when smooth).

**Why this matters**

* This validates solver envelopes without “unstable open-loop drift” confounding
  the result.

---

## B. Missing unit-test coverage to add

### Powertrain

* Thevenin analytic step response (SOC/V1/terminal V).
* OCV interpolation correctness (monotonic curve, clamping, endpoints).
* Warning threshold mapping (remaining → warning enum).

### Propulsion

* Motor current/torque sign: duty>0 should give positive torque at ω=0.
* ESC deadzone correctness: duty in [0, deadzone] produces ~0 effective voltage/current.
* Thrust monotonicity: increasing ω increases thrust for fixed density and inflow=0.
* Inflow correction sanity: thrust doesn’t increase when axial inflow magnitude increases
  in the expected regime.

### PlantModels.CoupledMultirotorModel

* Bus solve residual.
* Fault consumption (motor_disable, battery_disconnect).
* plant_project clamps (rotor ω ≥ 0, SOC bounds).

### Environment

* ISA1976 spot checks at 0m and 11km.
* OU coefficient update correctness.
* OU recurrence (deterministic single-step).

### Sources / Runtime / Recording

* Boundary protocol enforcement.
* Timeline integrity.
* Record/replay equivalence.
* Schema/header checks.

### Integrators

* RK23 quantization edge cases.
* Interval boundary behavior (“remaining < h_min”).
* Plant-aware error norm: enabled vs disabled behavior.

---

## C. Verification ladder

### Tier 1: Analytic / invariant unit tests (fast, always-on)

* SHO, pendulum, Kepler, torque-free rigid body (already)
* add battery analytic, OU recurrence, ISA spot checks

### Tier 2: Full-plant contract tests (still fast, always-on)

* ballistic no-thrust
* hover force-balance RHS check
* symmetry torque check
* bus residual check

### Tier 3: System regression (optional CI job / local)

* record-once Iris mission into Tier0 (or keep a small canned Tier0 recording checked in)
* run determinism + local defect integrator comparison vs tight RK45 reference

---

## D. PX4 bridge correctness needs its own verification path

The largest unaddressed risk area is the **PX4 interface mapping** (frames, signs,
sensor semantics). This requires either:

* A “mock PX4” contract test that asserts inputs match PX4 expectations, OR
* A golden record from PX4 with known conditions (sit on ground, arm, takeoff) and
  verify sensor fields match expectations.

This can reside outside unit tests (since it depends on the PX4 C library), but it
should be repeatable and automated.
