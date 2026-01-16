"""Plant-side protocol utilities.

This file defines **protocol functions** that are shared across multiple simulation
engines (e.g., `PlantSimulation` and `RecordReplay.BusEngine`).

Why this exists
---------------
We want multiple engines to be able to query *algebraic* plant outputs at discrete
event boundaries (battery telemetry, rotor outputs, bus voltage/current, etc.)
without tightly coupling to a specific engine implementation.

`plant_outputs(f, t, x, u)` is that protocol:

* `f` is a dynamics functor (RHS) used by an integrator.
* `t` is time in seconds.
* `x` is the continuous plant state.
* `u` is a sample-and-hold input packet.

Engines may call `plant_outputs` at event boundaries to:

* inject battery telemetry into PX4
* generate deterministic logs

Notes
-----
* This is intentionally a **generic function with no fallback method**.
  Engines should check `applicable(plant_outputs, ...)` before calling.
* Concrete RHS functors (e.g., `PlantSimulation.PlantDynamicsWithContact`) should
  implement a method returning `Plant.PlantOutputs` or another appropriate
  outputs struct.
"""

"""Protocol: evaluate algebraic plant outputs at a boundary time.

See the module docstring above.
"""
function plant_outputs end
