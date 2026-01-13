"""PX4Lockstep.Sim

Composable, deterministic simulation framework intended for PX4-in-the-loop lockstep SITL.

This is a lightweight foundation designed to scale to the long-term architecture you described:

* environment: ISA1976 atmosphere, wind, gravity, (later: terrain)
* aircraft: 6DOF rigid-body + aero + powertrain + actuators
* engines: Euler/RK4 now; additional fixed/adaptive solvers later
* scenarios: scripted, monte-carlo, wind sweeps, etc.

The key idea is to keep the simulation loop explicit and single-threaded so it integrates cleanly with the PX4 lockstep library.
"""
module Sim

include("Types.jl")
include("Environment.jl")
include("Powertrain.jl")
include("RigidBody.jl")
include("Integrators.jl")
include("Vehicles.jl")
include("Autopilots.jl")
include("Scenario.jl")
include("Logging.jl")
include("Simulation.jl")

end # module Sim
