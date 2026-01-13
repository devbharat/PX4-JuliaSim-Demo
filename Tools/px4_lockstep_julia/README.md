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
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl
```

Outputs:

* `sim_log.csv`
  * includes position/velocity setpoints for tracking plots

## Python plotting

Lightweight Python scripts live in `Tools/px4_lockstep_julia/scripts`.
See `Tools/px4_lockstep_julia/scripts/README.md` for setup and usage.

## Developer tooling

Install tooling deps (once):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

Format Julia source:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JuliaFormatter; format("Tools/px4_lockstep_julia/src")'
```

Run Aqua checks (project hygiene + method ambiguities):

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Aqua, PX4Lockstep; Aqua.test_all(PX4Lockstep)'
```

Run JET static analysis:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using JET; JET.report_package("PX4Lockstep")'
```

## Architecture

The simulation framework is organized as composable modules:

* `PX4Lockstep.Sim.Environment`
  * atmosphere (ISA1976), wind models (incl. gusts), gravity models
* `PX4Lockstep.Sim.Vehicles`
  * rigid-body 6DOF baseline model (Iris quad)
  * actuator dynamics (`DirectActuators`, `FirstOrderActuators`, `SecondOrderActuators`)
  * quad thrust nonlinearity via `thrust_exponent`
* `PX4Lockstep.Sim.Powertrain`
  * `IdealBattery` baseline
  * `TheveninBattery` (OCV + R0 + RC) for better voltage sag realism
* `PX4Lockstep.Sim.Integrators`
  * fixed-step Euler and RK4 (default)
* `PX4Lockstep.Sim.Scheduling`
  * deterministic periodic triggers for multi-rate stepping (no threads)
* `PX4Lockstep.Sim.Noise`
  * AR(1) bias + Gaussian noise utilities
* `PX4Lockstep.Sim.Estimators`
  * truth → estimated state (noise, bias, delay) feeding PX4 without EKF2
* `PX4Lockstep.Sim.Autopilots`
  * thin bridge from sim truth → PX4 lockstep inputs
* `PX4Lockstep.Sim.Logging`
  * `SimLog` in-memory logging and `CSVLogSink` streaming logs
* `PX4Lockstep.Sim.Simulation`
  * deterministic engine: scenario → PX4 → actuators → integrate → log

### Estimator injection (noise + delay)

You can inject estimated-state noise and latency before PX4 sees the state:

```julia
base_est = Sim.Estimators.NoisyEstimator(
    pos_sigma_m=Sim.Types.vec3(0.3, 0.3, 0.5),
    vel_sigma_mps=Sim.Types.vec3(0.1, 0.1, 0.2),
    yaw_sigma_rad=deg2rad(2.0),
    rate_sigma_rad_s=Sim.Types.vec3(0.02, 0.02, 0.03),
    bias_tau_s=20.0,
    pos_bias_sigma_m=Sim.Types.vec3(0.5, 0.5, 0.5),
)

est = Sim.Estimators.DelayedEstimator(base_est; delay_s=0.02, dt_est=0.01)

sim = Sim.Simulation.SimulationInstance(
    ...,
    estimator=est,
)
```

### Extending

To add a new aircraft model, implement a new `AbstractVehicleModel` and a corresponding `dynamics(model, env, t, state, u)` method that returns a `RigidBodyDeriv`.

To add a new integrator, implement `step_integrator(::YourIntegrator, f, t, x, u, dt)`.

To add new worlds, compose new `EnvironmentModel(atmosphere=..., wind=..., gravity=...)` types.

## Notes

* The framework uses NED as the world frame to match PX4.
* Controller outputs are treated as piecewise-constant over each sim `dt`, which is the standard assumption for fixed-step closed-loop simulation.
