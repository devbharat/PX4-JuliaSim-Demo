# Frames and Sign Conventions

This repository is intentionally “convention heavy”: **most subtle sim/controller bugs are sign or frame mismatches**.
This page documents the *canonical* conventions used by the simulator **as implemented in the code**.

If you change any of these semantics, treat it as an interface break:

- Update this document.
- Consider bumping `BUS_SCHEMA_VERSION` if bus-level meanings change.

---

## Coordinate frames

### World frame: local NED

The simulator uses a **local tangent** frame with axes:

- **N**: North (positive)
- **E**: East (positive)
- **D**: Down (positive)

This is referred to throughout the code as **NED**.

**State variables**

- `pos_ned` is position in meters.
- `vel_ned` is velocity in meters/second.

**Down is positive.**

That means:

- “Altitude increase” → `pos_ned[3]` becomes **more negative**.
- Gravity acceleration is **positive** in the D axis (see [Gravity](#gravity)).


### Body frame: FRD

Vehicle body axes follow the PX4 convention:

- **F**: Forward (positive X)
- **R**: Right (positive Y)
- **D**: Down (positive Z)

This is referred to as the **body** frame.

Body angular rates are stored as:

- `ω_body = (p, q, r)` in **rad/s**, expressed in **body/FRD**.

All rotations and cross products use a **right-hand rule**.


### Propulsor axis convention

Propulsor/rotor axes are defined in body coordinates as unit vectors `axis_b[i]` such that:

```
Fᵢ_body = -Tᵢ * axis_b[i]
```

where:

- `Tᵢ ≥ 0` is the **thrust magnitude** (Newtons)
- `Fᵢ_body` is the force applied to the rigid body, expressed in **body/FRD**

For a “standard” multirotor where thrust points **up** (negative body Z), you therefore use:

- `axis_b = (0, 0, 1)` (points along **+body Z / down**)
- thrust force becomes `(0, 0, -T)` (up)

This is implemented in `src/sim/Vehicles.jl`.

---

## Attitude representation and transforms

### Quaternion definition

Attitude is stored as:

- `q_bn`: quaternion **body → NED**
- component order: **(w, x, y, z)**

Rotation matrix:

- `R_bn = quat_to_dcm(q_bn)`

Vector transforms:

- **body → NED**: `v_ned  = R_bn * v_body`
- **NED → body**: `v_body = R_bn' * v_ned` (equivalently `quat_rotate_inv(q_bn, v_ned)`)

See `src/sim/Types.jl` for the canonical helpers.


### Quaternion kinematics

The quaternion derivative uses body rates (expressed in body):

```
q̇ = 0.5 * q ⊗ (0, p, q, r)
```

where `⊗` is quaternion multiplication.

This is implemented in `src/sim/RigidBody.jl` as `quat_deriv(q_bn, ω_body)`.


### Yaw definition

`yaw_from_quat(q_bn)` returns yaw about **NED +Z (down)**.

Interpretation:

- `yaw = 0` → body X (forward) aligned with North
- positive yaw rotates body X toward East (clockwise when viewed from above)

This matches the standard PX4/NED yaw convention.

---

## Forces, moments, and dynamics

### Translational dynamics

Rigid-body translational acceleration is computed in **NED**.

For the baseline multirotor model (`Vehicles.dynamics`):

```
vel̇_ned = (F_ned + F_drag) / m + g_ned
```

where:

- `F_ned` is the summed force from propulsors, rotated into NED.
- `F_drag` is a simple linear drag term based on air-relative velocity.
- `g_ned` is gravity acceleration in NED.


### Rotating body forces into NED

Forces computed in body are rotated using `q_bn`:

```
F_ned = R_bn * F_body
```

where `R_bn = quat_to_dcm(q_bn)`.


### Moments / torques

In the multirotor rigid-body model (`Vehicles.dynamics`), total body-frame torque is:

```
τ_body = Σ ( rᵢ × Fᵢ_body ) + Σ ( axis_b[i] * Qᵢ )
```

where:

- `rᵢ` is rotor position in body (meters)
- `Fᵢ_body = -Tᵢ * axis_b[i]`
- `Qᵢ` is the **signed** shaft/reaction torque (N·m)

**Important:** the vehicle model assumes `Qᵢ` is already signed correctly.
The propulsion model is the single source of truth for yaw-torque sign.


### Contact forces

Contact models return an *external* force in **NED** via:

- `Contacts.contact_force_ned(contact, rb_state, t)`

For the built-in flat ground plane:

- ground is at `z = 0` (NED)
- penetration occurs when `pos_ned[3] > 0` (below the plane)
- normal force is upward → **negative** NED Z

---

## Wind and air-relative quantities

### What `wind_ned` means

`wind_ned` is the **air mass velocity in NED**, in m/s.

It is carried on the runtime bus as `bus.wind_ned` and treated as **sample-and-hold**
between wind ticks (important for determinism and record/replay).

From that definition:

- Vehicle velocity relative to air:

  ```
  v_rel_ned = vel_ned - wind_ned
  ```

- Airflow (“relative wind”) seen by the vehicle:

  ```
  v_air_ned = wind_ned - vel_ned = -v_rel_ned
  v_air_body = R_bn' * v_air_ned
  ```

The plant model reports `PlantOutputs.air_vel_body = v_air_body`.


### Converting “meteorological wind” to `wind_ned`

If you have a wind **speed** `V` and a **direction-from** angle `ψ_from` (degrees, where
`0° = from North`, `90° = from East`), then:

```
wind_ned = -V * [cosd(ψ_from), sind(ψ_from), 0]
```

Reason: “from North” means the air is moving **toward South**.

This conversion is a common source of sign bugs; keep it explicit in scenario tooling.


### Axial inflow sign used by propulsion

The coupled multirotor plant computes an axial inflow scalar per rotor as:

```
Vaxᵢ = -dot(v_air_body, axis_b[i])
```

Given the axis convention `F = -T * axis_b`, this yields:

- `Vaxᵢ > 0` when airflow is along the **thrust direction** (i.e. along `-axis_b`)
  - e.g. in a vertical descent for a standard multirotor

Currently the inflow correction uses `Vaxᵢ²`, so only magnitude matters, but the sign is
documented here to prevent future extensions from silently flipping it.

---

## Gravity

Gravity is expressed in **NED** as a positive down acceleration:

- uniform gravity: `g_ned = (0, 0, +g)`

So an aircraft “hovering” with upward thrust will have negative Z force in NED that
cancels the positive-down gravity term.

See `Environment.UniformGravity` and `Environment.gravity_accel`.

---

## Altitude, home, and atmosphere

The environment defines a `WorldOrigin` / `HomeLocation` with:

- `lat_deg`, `lon_deg`
- `alt_msl_m` (meters above mean sea level)

The simulator converts NED down position into altitude above MSL as:

```
alt_msl_m = origin.alt_msl_m - pos_ned[3]
```

Atmosphere functions (`air_density`, `air_temperature`, `air_pressure`) expect altitude
**above MSL**.

The PX4 bridge uses the same origin to derive approximate global position for uORB
injection.

---

## Actuator command conventions

The runtime bus carries an `ActuatorCommand` packet:

- `motors[1:12]` are normalized ESC duties in **[0, 1]**
- `servos[1:8]` are normalized deflections in **[-1, 1]**

Physical airframes may have fewer actuators than the ABI sizes. Mapping is explicit:

- `Vehicles.MotorMap{N}` maps PX4 motor output channels → physical propulsors `1..N`.
- `FaultState.motor_disable_mask` is defined over **physical propulsors**, not PX4 channels.

---

## Electrical sign conventions

Electrical currents use:

- **positive current = discharge** (power leaving the battery)

This applies to:

- `RotorOutput.bus_current_a`
- `PlantOutputs.bus_current_a`
- battery telemetry injected into PX4

---

## Quick sanity checks

These are fast tests you can do mentally or in a unit test to validate sign/frame work:

1. **Hover direction** (level attitude, `q_bn = identity`):
   - rotor `axis_b = (0,0,1)`
   - positive thrust magnitudes should produce `F_body.z < 0` and therefore `F_ned.z < 0`
     (upward force in NED).

2. **Wind push** (vehicle stationary, `vel_ned = 0`):
   - if `wind_ned = (5,0,0)` (air moving North), drag should accelerate the vehicle North.

3. **Altitude conversion**:
   - if `pos_ned[3] = +10` (10 m down), then `alt_msl_m` should be 10 m **lower**.

4. **Yaw sign**:
   - yaw +90° should align body X with East.

---

## Where this is defined in code

- Core types + rotations: `src/sim/Types.jl`
- Rigid-body state + quaternion kinematics: `src/sim/RigidBody.jl`
- Vehicle/actuator conventions + multirotor forces: `src/sim/Vehicles.jl`
- Wind/atmosphere/gravity models: `src/sim/Environment.jl`
- Runtime bus wind semantics: `src/sim/Runtime/Bus.jl`
- Coupled propulsion inflow definition: `src/sim/PlantModels/CoupledMultirotor.jl`
