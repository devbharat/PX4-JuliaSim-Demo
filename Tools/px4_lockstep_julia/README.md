# PX4Lockstep.jl

This directory contains two layers:

1) **`PX4Lockstep`** – a small, dependency-light Julia wrapper around the `libpx4_lockstep` C ABI.
2) **`PX4Lockstep.Sim`** – a deterministic, single-threaded simulation framework designed for PX4-in-the-loop lockstep SITL.

The goal is to keep the lockstep bridge thin and deterministic, while the Julia side owns:

* vehicle truth dynamics
* environment models
* battery / powertrain models
* logging, scenarios, and analysis

## Quick start

1. Build `libpx4_lockstep` in your PX4 tree.
2. Set environment variables (optional, `PX4Lockstep.jl` will also search the build tree):

```bash
export PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.so  # or .dylib on macOS
export PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints
```

3. Install dependencies (note the command has no trailing `.`):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

4. Run the example:

```bash
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia -e 'using PX4Lockstep.Sim; Sim.simulate_iris_mission(mode=:live)'
```

5. Recommended: run the **record → replay integrator comparison** (PX4 live, then plant-only replay sweep):

```bash
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
```

Outputs:

* `sim_log.csv`
  * includes position/velocity setpoints for tracking plots
  * includes air-relative velocity in body axes (`air_bx/air_by/air_bz`) for inflow-aware propulsion debugging

Notes:

* `libpx4_lockstep` is not re-entrant; by default only one lockstep handle is allowed per
  process. For Monte Carlo, launch separate processes or pass
  `allow_multiple_handles=true` when calling `PX4Lockstep.create` / `Sim.Autopilots.init!`.

## Python plotting

Lightweight Python scripts live in `Tools/px4_lockstep_julia/scripts`.
See `Tools/px4_lockstep_julia/scripts/README.md` for setup and usage.

## Verification problems (integrator correctness)

This repo includes a small set of deterministic verification problems with analytic
solutions and/or conserved quantities. These are meant to catch numerical regressions
and to help you compare solver configurations without needing PX4 in the loop.

Run them from the repo root:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/sho.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/pendulum.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/kepler_circular.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/torque_free_rigid_body.jl

# Reference-trajectory comparison (RK45 "truth"; no analytic solution required)
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_pendulum_large_angle.jl
```

## Developer tooling

Install tooling deps (once):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

Format Julia source:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JuliaFormatter; format("Tools/px4_lockstep_julia/src")'
```

Run Aqua checks (project hygiene + method ambiguities). We disable `stale_deps` because
tooling packages are kept as direct deps for convenience:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Aqua, PX4Lockstep; Aqua.test_all(PX4Lockstep; stale_deps=false)'
```

Run JET static analysis (disable `analyze_from_definitions` for Julia 1.12 compatibility):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JET; JET.report_package("PX4Lockstep"; analyze_from_definitions=false)'
```

## Architecture

The simulation framework is organized as composable modules:

* `PX4Lockstep.Sim.Environment`
  * atmosphere (ISA1976), wind models (incl. OU turbulence + gusts), gravity models
* `PX4Lockstep.Sim.Vehicles`
  * rigid-body 6DOF baseline model (Iris quad)
  * actuator dynamics (`DirectActuators`, `FirstOrderActuators`, `SecondOrderActuators`)
* `PX4Lockstep.Sim.Plant`
  * `PlantState` / `PlantDeriv` for full continuous plant state (rigid body + actuators + rotors + battery)
  * `PlantInput` / `PlantOutputs` glue for variable-step integration
* `PX4Lockstep.Sim.Powertrain`
  * `IdealBattery` baseline
  * `TheveninBattery` (OCV + R0 + RC) for better voltage sag realism
* `PX4Lockstep.Sim.Propulsion`
  * motor+ESC+prop split: duty → current/torque → ω → thrust/drag torque
  * inflow-aware thrust/torque correction (vertical descent + wind projection reduce thrust for a given ω)
* `PX4Lockstep.Sim.Contacts`
  * pluggable contact model (`NoContact` by default; `FlatGroundContact` optional)
* `PX4Lockstep.Sim.Events` / `PX4Lockstep.Sim.Scenario`
  * deterministic event scheduler (arm at t, gust injection, motor failure, SOC triggers, ...)
* `PX4Lockstep.Sim.Scheduling`
  * deterministic periodic triggers for multi-rate stepping (no threads)
* `PX4Lockstep.Sim.Noise`
  * AR(1) bias + Gaussian noise utilities
* `PX4Lockstep.Sim.Estimators`
  * truth → estimated state (noise, bias, delay) feeding PX4 without EKF2
* `PX4Lockstep.Sim.Integrators`
  * fixed-step Euler and RK4 (default)
  * adaptive (variable-step) RK23 and RK45 (Dormand–Prince) integrators (optional)
* `PX4Lockstep.Sim.Autopilots`
  * thin bridge from sim truth → PX4 lockstep inputs
* `PX4Lockstep.Sim.Logging`
  * `SimLog` in-memory logging and `CSVLogSink` streaming logs
* `PX4Lockstep.Sim.Runtime`
  * **canonical** event-driven engine (live / record / replay)
  * integrates full plant between event boundaries using fixed or adaptive integrators
* `PX4Lockstep.Sim.Recording` / `PX4Lockstep.Sim.Sources`
  * traces, recorders, and live/replay sources feeding the runtime engine
* `PX4Lockstep.Sim.PlantModels`
  * plant RHS functors and bus-coupled plant output evaluation (`plant_outputs`)

### Estimator injection (noise + delay)

Estimator injection (noise, bias, and delay) is provided by discrete **sources**.

- Live runs typically use `Sim.Sources.LiveEstimatorSource(estimator_model, rng; dt_est_s=...)`.
- Replay runs can use `Sim.Sources.ReplayEstimatorSource(trace)`.

See `src/sim/Workflows/Iris.jl` for an end-to-end configuration (it wires a noisy + delayed estimator into the runtime engine at the autopilot cadence).

### Multi-rate stepping

The simulator is event-driven. You configure cadences via a `Sim.Runtime.Timeline`:

- `ap` axis: PX4 lockstep ticks
- `wind` axis: wind sample-and-hold updates
- `log` axis: logging samples
- optional `phys` axis: fixed physics step boundaries (if you want strict fixed-step integration)

Example:

```julia
using PX4Lockstep.Sim

timeline = Sim.Runtime.build_timeline(
    t_end_s=20.0,
    dt_autopilot_s=0.004,
    dt_wind_s=0.004,
    dt_log_s=0.01,
    dt_phys_s=0.002,  # optional
)
```

The runtime engine integrates the full plant between the **next** boundaries in the union axis.
Between boundaries, inputs are held constant (ZOH).

For PX4 lockstep, the autopilot cadence must be compatible with the PX4 internal scheduling. The autopilot bridge enforces this by default.

### Extending

To add a new aircraft model, implement a new `AbstractVehicleModel` and a corresponding `dynamics(model, env, t, state, u)` method that returns a `RigidBodyDeriv`.

To add a new integrator, implement `step_integrator(::YourIntegrator, f, t, x, u, dt)`.

To add new worlds, compose new `EnvironmentModel(atmosphere=..., wind=..., gravity=...)` types.

## Notes

* The framework uses NED as the world frame to match PX4.
* Controller outputs are treated as piecewise-constant over each sim `dt`, which is the standard assumption for fixed-step closed-loop simulation.
* Wind turbulence is advanced once per tick (seeded RNG) and held constant over the integration step for determinism.
* CSV logs include `time_us` (exact lockstep microsecond time) as of `schema_version=2`.

## Example Run
```bash
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia -e 'using PX4Lockstep.Sim; Sim.simulate_iris_mission(mode=:live)'

python Tools/px4_lockstep_julia/scripts/plot_sim_log.py \
  --log sim_log.csv \
  --output sim_plot.png \
  --inflow-output sim_inflow.png
```
<img width="1800" height="1500" alt="sim_plot" src="https://github.com/user-attachments/assets/471d5aef-7533-461b-a4b8-528f4beb6d4a" />
<img width="1650" height="1350" alt="sim_inflow" src="https://github.com/user-attachments/assets/9a56fcc1-7016-4707-ba5f-b48d3257bf92" />
