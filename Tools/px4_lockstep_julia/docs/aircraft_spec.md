# AircraftSpec TOML schema

This repo supports **declarative aircraft configuration** via TOML files.
The intent is that a single file can describe:

- the vehicle geometry (motors/axes/inertia)
- the power topology (batteries + buses)
- PX4 lockstep configuration (lockstep config, uORB boundary, optional params)
- the simulator configuration that matters for determinism (timeline, integrator, contact)

The loader lives in:

- `PX4Lockstep.Sim.Aircraft.load_spec(path)`
- `PX4Lockstep.Sim.Aircraft.run_spec(path; ...)`

Example specs are in `examples/specs/`.
The loader is strict by default and does **not** apply internal defaults.
Use `extends` to compose from shared defaults (e.g., the TOMLs under
`src/Workflows/assets/aircraft`). If you explicitly want the built-in generic
multirotor defaults, call `default_multirotor_spec()` or pass
`base_spec=:default` to `load_spec`.

## Quick start

```bash
julia --project=. examples/run_spec.jl examples/specs/iris.toml
```

Note: `--project=.` assumes you run from `Tools/px4_lockstep_julia`.
From the PX4 root, use `--project=Tools/px4_lockstep_julia`.

## Key features

### `extends` (deep-merge)

Specs can inherit from one or more base TOMLs:

```toml
schema_version = 1
extends = ["base/iris_base.toml"]

[timeline]
t_end_s = 60.0
```

Merge rules:

- tables merge recursively
- arrays replace (not append)
- scalars replace

### Path resolution

Relative paths are resolved against the TOML file location.

## Schema overview

Top-level keys:

- `schema_version = 1`
- `extends = [ ... ]` (optional)
- `[aircraft]` (name/seed)
- `[home]` (lat/lon/alt)
- `[px4]` (mission, lockstep config, uORB boundary, params)
- `[timeline]` (dt + horizon)
- `[plant]` (integrator + contact)
- `[airframe]` (mass/inertia/rotor layout/initial state/propulsion)
- `[actuation]` (motor + servo instances, actuator dynamics models)
- `[power]` (batteries + buses)
- `[sensors]` (placeholders, currently not wired into the plant)
- `[run]` (optional: defaults for `run_spec`)

### `[aircraft]`

```toml
[aircraft]
name = "iris"          # becomes Symbol(:iris)
seed = 1
```

### `[px4]`

```toml
[px4]
mission_path = "../../src/Workflows/assets/missions/simple_mission.waypoints"  # optional in replay mode
libpath = "/path/to/libpx4_lockstep.(so|dylib)"  # required for live/record runs
edge_trigger = false
derive_ca_params = true

[px4.lockstep]
enable_commander = 0
enable_control_allocator = 1

[[px4.params]]
name = "MC_AIRMODE"
value = 1
```

### uORB interface

Define the uORB boundary directly:

```toml
[px4.uorb]
[[px4.uorb.pubs]]
key = "battery_status"
type = "BatteryStatusMsg"

[[px4.uorb.subs]]
key = "actuator_motors"
type = "ActuatorMotorsMsg"
```

For reuse, include a shared uORB file with `extends` (e.g. `iris_uorb.toml` or
`minimal_uorb.toml` under `src/Workflows/assets/aircraft`).

`px4.uorb` is required for live/record runs.

Note: publisher instances default to `-1` (auto) if not specified. For determinism,
prefer pinning explicit `instance = 0` for singleton topics and explicit instances
for multi-instance topics. Auto-instance behavior may change if publisher ordering changes.

### `[timeline]`

```toml
[timeline]
t_end_s = 20.0
dt_autopilot_s = 0.004
dt_wind_s = 0.001
dt_log_s = 0.01
```

### `[plant]`

```toml
[plant]
integrator = "RK45"       # Euler|RK4|RK23|RK45
contact = "flat_ground"   # flat_ground|no_contact

# Or a full table:
# [plant.integrator]
# kind = "RK45"            # Euler|RK4|RK23|RK45
# rtol_pos = 1e-7
# atol_pos = 1e-6
# rtol_vel = 1e-7
# atol_vel = 1e-6
# rtol_omega = 1e-7        # alias: rtol_ω
# atol_omega = 1e-6        # alias: atol_ω
# atol_att_rad = 1e-6
# plant_error_control = false
# rtol_act = 0.0
# atol_act = 1.0e-3
# rtol_actdot = 0.0
# atol_actdot = 1.0e-2
# rtol_rotor = 0.0
# atol_rotor = 1.0
# rtol_soc = 0.0
# atol_soc = 1.0e-4
# rtol_v1 = 0.0
# atol_v1 = 1.0
# h_min = 1.0e-6
# h_max = 0.01
# h_init = 0.0
# max_substeps = 50000
# safety = 0.9
# min_factor = 0.2
# max_factor = 5.0
# quantize_us = true
#
# [plant.contact]
# kind = "flat_ground"
# k_n_per_m = 5000.0
# c_n_per_mps = 600.0
# mu = 0.8
```

### `[environment]`

```toml
[environment]
wind = "ou"                # ou|none|constant
wind_mean_ned = [0.0, 0.0, 0.0]
wind_sigma_ned = [1.5, 1.5, 0.5]
wind_tau_s = 3.0

atmosphere = "isa1976"

gravity = "uniform"        # uniform|spherical
gravity_mps2 = 9.80665
# gravity_mu = 3.986004418e14
# gravity_r0_m = 6_371_000.0
```

### `[scenario]`

```toml
[scenario]
arm_time_s = 1.0
mission_time_s = 2.0
```

### `[estimator]`

```toml
[estimator]
kind = "noisy_delayed"    # noisy_delayed|none
pos_sigma_m = [0.02, 0.02, 0.02]
vel_sigma_mps = [0.05, 0.05, 0.05]
yaw_sigma_rad = 0.01
rate_sigma_rad_s = [0.005, 0.005, 0.005]
bias_tau_s = 50.0
rate_bias_sigma_rad_s = [0.001, 0.001, 0.001]
delay_s = 0.008           # must be multiple of dt_est_s
dt_est_s = 0.004          # defaults to dt_autopilot_s (must match it for now)
```

**Note:** the estimator is stepped at the autopilot cadence. For now, `dt_est_s`
must equal `timeline.dt_autopilot_s` to keep the delayed-estimator timing exact.
Decimated estimator ticks are a future extension.

### `[airframe]`

```toml
[airframe]
kind = "multirotor"
mass_kg = 1.5
inertia_diag_kgm2 = [0.03, 0.03, 0.06]

# Optional products of inertia (body frame): [Ixy, Ixz, Iyz].
# inertia_products_kgm2 = [0.0, 0.0, 0.0]

# Alternatively, provide the full symmetric 3x3 tensor (either nested 3x3 or flat 9 values):
# inertia_kgm2 = [
#   [0.03, 0.0, 0.0],
#   [0.0, 0.03, 0.0],
#   [0.0, 0.0, 0.06],
# ]
rotor_pos_body_m = [
  [0.15, 0.25, 0.0],
  ...
]
rotor_axis_body_m = [
  [0.0, 0.0, 1.0],
  ...
]

[airframe.x0]
pos_ned = [0.0, 0.0, 0.0]
vel_ned = [0.0, 0.0, 0.0]
euler_deg = [0.0, 0.0, 0.0]  # or q_bn = [w,x,y,z]
omega_body = [0.0, 0.0, 0.0]

[airframe.propulsion]
kind = "multirotor_default"
km_m = 0.05
V_nom = 12.0
rho_nom = 1.225
rotor_radius_m = 0.127
inflow_kT = 8.0
inflow_kQ = 8.0
# thrust_calibration_mult = 2.0
# rotor_dir = [1, 1, -1, -1]

# ESC + motor internal parameters (optional):
# [airframe.propulsion.esc]
# eta = 0.98
# deadzone = 0.02
#
# [airframe.propulsion.motor]
# kv_rpm_per_volt = 920.0
# r_ohm = 0.25
# j_kgm2 = 1.2e-5
# i0_a = 0.6
# viscous_friction_nm_per_rad_s = 2.0e-6
# max_current_a = 60.0
```

Currently only `multirotor` is supported; other kinds will error during validation.

**Notes:**

- `rotor_dir` encodes the **reaction torque sign** on the body. Rotor spin direction
  is opposite this sign and is what the gyro-coupling uses. Values must be ±1.
- `thrust_calibration_mult` scales the hover thrust target used to calibrate `kT`.
  The default `2.0` preserves the original tuning (hover at the previous duty).

### `[power]`

```toml
[power]
share_mode = "inv_r0"   # inv_r0|equal

[[power.batteries]]
id = "bat1"
model = "thevenin"
capacity_ah = 5.0
soc0 = 1.0
ocv_soc = [0.0, 1.0]
ocv_v = [10.8, 12.6]
r0 = 0.020
r1 = 0.010
c1 = 2000.0
min_voltage_v = 9.9

[[power.buses]]
id = "main"
battery_ids = ["bat1"]
motor_ids = ["motor1", "motor2", "motor3", "motor4"]
servo_ids = []
avionics_load_w = 0.0
```

### `[run]` (optional convenience)

```toml
[run]
mode = "live"               # live|record|replay
recording_in = "..."        # used for replay
recording_out = "..."       # used for record
log_csv = "sim_log.csv"     # optional CSV log sink
```

`run_spec(...)` reads these defaults; if you pass `mode`/`recording_*` arguments, they override the `[run]` section.
