# Fault semantics and consumption map

This document defines the **precise semantics** of bus-level faults (`Sim.Faults.FaultState`) and
lists **who consumes what**. The goal is to make failures:

* deterministic,
* record/replayable,
* and debuggable (no hidden mutation side-effects).

When semantics change, update this doc and bump any relevant schema versions.

---

## 1) Where faults live

Faults are a first-class bus signal:

* `bus.faults::FaultState`

The bus value is treated as **piecewise constant** between event boundaries:

* Scenario updates can change `bus.faults` at boundary time `t_us`.
* The plant consumes the held fault state for integration over `[t_k, t_{k+1})`.

### Why this matters

This avoids failure semantics that depend on mutation order or update ordering.
It also makes record/replay stable by replaying **fault signals**, not “event side effects”.

---

## 2) FaultState definition

`FaultState` is intentionally compact and versionable:

* `motor_disable_mask::UInt32`
* `battery_connected::Bool`
* `sensor_fault_mask::UInt64`

### 2.1 Motor disable mask

**Bit meaning:** bit `(i-1)` disables motor `i` (1-indexed). Supports `i ∈ [1, 32]`.

#### Semantics (current)

“Motor disabled” means:

* the commanded duty for motor `i` is forced to `0.0` **inside the plant RHS**, i.e. the ESC is
  effectively commanded off.
* the rotor remains physically present: aerodynamic drag / inflow effects still act on rotor ω.
  (So the rotor can **windmill** under airflow and vehicle motion.)
* there is no special “stuck/jammed” rotor constraint; i.e. this is not a rotor jam.

This is intentionally the simplest failure that preserves continuous dynamics and avoids algebraic
constraints.

#### Not implemented (explicitly)

For these additional fault modes, add separate bus-level signals so replay is explicit:

* rotor jam (ω forced to 0 or locked to body)
* thrust loss fraction
* stuck duty (hold last duty)
* motor reversal

### 2.2 Battery disconnect

`battery_connected=false` means:

* bus solver is bypassed
* `V_bus = 0.0`
* `I_bus = 0.0`
* battery SOC is held constant (current is zero)
* Thevenin polarization voltage `v1` relaxes toward 0 via its RC time constant
* telemetry reports `BatteryStatus.connected=false`, voltage/current forced to 0

This is “battery unplugged / main power removed”, not “brownout”.

### 2.3 Sensor fault mask

This mask is a **generic fault vector** for sensor/estimator failures.

Defined bits (see `src/sim/Faults.jl`):

* `SENSOR_FAULT_GYRO`
* `SENSOR_FAULT_ACCEL`
* `SENSOR_FAULT_MAG`
* `SENSOR_FAULT_BARO`
* `SENSOR_FAULT_GPS`
* `SENSOR_FAULT_VISION`
* `SENSOR_FAULT_EST_FREEZE` (sim-level fault)

#### Semantics (currently implemented)

Only one bit has active semantics today:

* `SENSOR_FAULT_EST_FREEZE`: the estimator source skips its update and holds `bus.est` constant.

The sensor-specific bits are **reserved** for future use. The intended direction is:

* map these bits to bus-level “sensor stream validity” or “dropout” signals,
* and/or map them into PX4 sensor topics (device error flags) at the bridge.

---

## 3) Fault consumption map

This is the authoritative map of “who reads which faults”.

### 3.1 Scenario (publisher)

* **Publishes:** `bus.faults`
* **Mechanism:** `Scenario.EventScenario` updates its internal `faults::FaultState` and
  `LiveScenarioSource` publishes it.
* **Replay:** `ReplayScenarioSource` publishes recorded `FaultState` on the bus.

### 3.2 Plant RHS (continuous consumer)

Consumes:

* `motor_disable_mask` → forces duty to 0 for disabled motors
* `battery_connected` → bypass bus solve and force bus V/I to 0

Location:

* `src/sim/PlantModels/CoupledMultirotor.jl` (`_eval_propulsion_and_bus` and bus solve path)

### 3.3 Estimator source (discrete consumer)

Consumes:

* `sensor_fault_mask & SENSOR_FAULT_EST_FREEZE` → hold `bus.est` constant

Location:

* `src/sim/Sources/Estimator.jl` (`LiveEstimatorSource.update!`)

### 3.4 Autopilot source (discrete consumer)

Consumes:

* `bus.est` (estimated state)
* `bus.battery` (telemetry)
* `bus.ap_cmd`, `bus.landed`

Notes:

* Autopilot does **not** currently read `bus.faults` directly.
  PX4 should infer failures through sensor/battery telemetry and/or actuator effectiveness.

### 3.5 Logging/recording

* `bus.faults` should be recorded as a stream so replays cannot miss transitions.
* The current implementation records `faults_evt` on `timeline.evt` (see below).

---

## 4) Recording semantics for dynamic faults

Scenarios can contain dynamic “When(...)” conditions that change faults based on state.

To avoid missing transitions, a **fault sample is recorded at every event boundary**:

* stream name: `:faults_evt`
* axis: `timeline.evt`
* interpolation: ZOH (held constant between boundaries)

This guarantees that any fault change that is visible to the engine at a boundary time will be
captured and replayed deterministically.
