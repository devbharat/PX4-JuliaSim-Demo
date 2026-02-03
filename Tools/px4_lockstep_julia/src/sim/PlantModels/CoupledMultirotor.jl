"""PlantModels.CoupledMultirotor

Coupled multirotor plant model that is suitable for fixed-step *and* adaptive-step
integration.

This is the canonical full-plant model used by `Sim.Runtime.Engine`.

Included physics (coupled inside the RHS)
----------------------------------------
* actuator dynamics (motor/servo)
* propulsion (motor/ESC electrical torque/current + prop inflow corrections)
* battery Thevenin (SOC + polarization) with **analytic bus solve** + deterministic
  fallback when region assumptions are violated
* rigid-body 6DOF dynamics
* optional contact penalty forces (primarily for in-air + simple ground)

Determinism contract
--------------------
* The RHS must be a pure function of `(t, x, u)`.
* No RNG, no mutation of shared state, no dependence on container iteration order.
* Discrete faults and commands are sample-and-hold via `PlantInput`.

Post-step projection
--------------------
Some state variables have hard physical bounds (e.g. ω ≥ 0, SOC ∈ [0,1]).
The canonical engine calls the `plant_project(model, x)` protocol after each
integrated interval (if implemented) so multiple engines do not duplicate clamp
logic.
"""

include("coupled_multirotor/Types.jl")
include("coupled_multirotor/Actuators.jl")
include("coupled_multirotor/PowerCoupling.jl")
include("coupled_multirotor/Contacts.jl")
include("coupled_multirotor/Outputs.jl")
include("coupled_multirotor/Integrate.jl")
