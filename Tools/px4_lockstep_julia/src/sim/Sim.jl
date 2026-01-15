"""PX4Lockstep.Sim

Composable, deterministic simulation framework intended for PX4-in-the-loop lockstep SITL.

This is a lightweight foundation designed to scale to a longer-term architecture:

* environment: ISA1976 atmosphere, wind, gravity
* aircraft: 6DOF rigid-body + aero + powertrain + actuators
* integrators: fixed-step Euler/RK4 and adaptive RK23/RK45
* scenarios: scripted scenarios, wind cases, etc.

The key idea is to keep the simulation loop explicit and single-threaded so it integrates cleanly with the PX4 lockstep library.

Note on variable-step plant integration
--------------------------------------
Adaptive integrators (RK23/RK45) exist in `Integrators.jl`.

For adaptive solvers to be meaningful, the integrated state vector must include the
*full continuous plant*: actuator states, rotor speed states, and battery states in
addition to the rigid body.

This refactor is implemented via:
* `Plant.jl`: defines `PlantState`/`PlantDeriv` and math helpers used by integrators.
* `PlantSimulation.jl`: event-driven engine that integrates the full plant between
  discrete event boundaries (autopilot/wind/log) with a coupled RHS for battery, bus,
  motor, propulsion, and actuator dynamics.
"""
module Sim

include("Types.jl")
include("Scheduling.jl")
include("Noise.jl")
include("RigidBody.jl")
include("Environment.jl")
include("Powertrain.jl")
include("Propulsion.jl")
include("Vehicles.jl")

# Continuous-time plant state for full variable-step integration.
include("Plant.jl")

include("Integrators.jl")
include("Estimators.jl")
include("Contacts.jl")
include("Events.jl")
include("Autopilots.jl")
include("Scenario.jl")
include("Logging.jl")
include("Simulation.jl")

# Event-driven variable-step engine for full-plant integration.
include("PlantSimulation.jl")

# Deterministic verification utilities and reference problems.
include("Verification.jl")

end # module Sim
