"""Plant-side protocol utilities.

This file defines **protocol functions** that are shared across multiple simulation
engines/harnesses. In the target architecture, the single canonical run loop is
`Runtime.Engine`, but protocol functions remain useful for unit tests and offline tools.

Why this exists
---------------
Multiple engines may need to query *algebraic* plant outputs at discrete
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
  The canonical engine caches capability detection at construction time.
* Concrete RHS functors (e.g., `PlantModels.CoupledMultirotorModel`) should
  implement a method returning `Plant.PlantOutputs` or another appropriate
  outputs struct.
"""

using .RigidBody: RigidBodyState

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
* This is intentionally an optional protocol: the canonical engine caches
  capability detection at construction time.
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

"""Protocol: integrate a plant state across a timeline interval.

Why this exists
---------------
Most plant models can be advanced by calling `Integrators.step_integrator` directly.

Ground contact, however, is inherently *hybrid* (it introduces discontinuities at
touchdown / liftoff). Handling that robustly and deterministically often requires
event localization (zero-crossing / time-of-impact search) and an impact map that
modifies the state at an internal time within the interval.

Rather than baking any specific contact logic into the canonical runtime engine,
an advanced plant model can override interval integration by providing this method.

Signature
---------
```
plant_integrate_interval(dynfun, integrator, t0_us, x0, u, dt_us) -> x1
```

Arguments:
- `dynfun`: dynamics functor used by the integrator (`f(t, x, u)`)
- `integrator`: any integrator supported by `Integrators.step_integrator`
- `t0_us`: interval start time in integer microseconds
- `x0`: plant state at `t0_us`
- `u`: held plant input over the interval (sample-and-hold)
- `dt_us`: interval length in integer microseconds

Notes
-----
* Implementations should be deterministic and must not use RNG.
* The runtime engine will still apply `plant_project` after the interval if that
  protocol is implemented.
"""
function plant_integrate_interval end

"""Protocol: extract a rigid-body state from a plant state.

Concrete plant states should either be `RigidBodyState` or expose an `rb` field.
"""
@inline rb_state(x::RigidBodyState) = x

@inline rb_state(x::T) where {T} = _rb_state_from_field(x, Val(hasfield(T, :rb)))

@inline _rb_state_from_field(x::T, ::Val{true}) where {T} = getfield(x, :rb)

@inline function _rb_state_from_field(x::T, ::Val{false}) where {T}
    error(
        "Plant state must expose an `rb` field to emit rigid-body logs. Got: " * string(T),
    )
end
