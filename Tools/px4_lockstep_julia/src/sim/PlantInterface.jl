"""Plant-side protocol utilities.

This file defines **protocol functions** that are shared across multiple simulation
engines/harnesses. In the target architecture, the single canonical run loop is
`Runtime.Engine`, but protocol functions remain useful for unit tests and offline tools.

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
* Concrete RHS functors (e.g., `PlantModels.CoupledMultirotorModel`) should
  implement a method returning `Plant.PlantOutputs` or another appropriate
  outputs struct.
"""

"""Protocol: evaluate algebraic plant outputs at a boundary time.

See the module docstring above.
"""
function plant_outputs end

"""Protocol: project a plant state back into the valid/physical set.

Why this exists
---------------
Some plant models have hard physical bounds (e.g. rotor speed ω ≥ 0, SOC ∈ [0,1]) or
actuator output ranges that must be enforced deterministically.

Rather than duplicating post-step clamping/projection logic in multiple engines, the
canonical engine may call `plant_project(f, x)` after each integrated interval.

Notes
-----
* This is intentionally an optional protocol: engines should check
  `applicable(plant_project, f, x)` before calling.
* Implementations should be pure (return a new state) or mutate only local, owned
  state; they must not use RNG.
"""
function plant_project end

"""Protocol: boundary-time updates at autopilot ticks.

This is intended for *hybrid* discontinuities that occur exactly at an autopilot
tick boundary and must be applied before integrating the next interval.

Primary current use:
- **Direct actuators**: snap `PlantState.motors_y/servos_y` to the newly published
  `ActuatorCommand` at an autopilot boundary.

Signature:
```
plant_on_autopilot_tick(model, x, cmd) -> x2
```

The default is "no-op" (no method defined).
"""
function plant_on_autopilot_tick end
