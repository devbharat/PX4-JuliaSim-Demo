"""PX4Lockstep.Sim.PlantModels

Concrete plant model implementations.

`Sim.Plant` defines the integration state (`PlantState`) and arithmetic.
Plant models implement pure continuous-time dynamics for the canonical engine:

- RHS: `f(t, x::PlantState{N,B}, u::PlantInput) -> PlantDeriv{N,B}`
- Algebraic outputs: `plant_outputs(t, x, u) -> PlantOutputs{N,B,K}`

Design rules:
- No RNG inside the RHS.
- No orchestration (no event logic, no scheduling, no record/replay).
"""
module PlantModels

include("PowerNetwork.jl")
include("CoupledMultirotor.jl")

end # module PlantModels
