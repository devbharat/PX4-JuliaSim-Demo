"""PX4Lockstep.Sim.Sources

Pluggable **sources** that publish into the canonical `Sim.Runtime` signal bus.

Sources are responsible for all discrete-time updates:

* scenario commands + faults
* wind / turbulence samples
* estimator output
* autopilot (PX4) actuator commands

Record/replay is achieved by swapping Live vs Replay source implementations.

This module is the canonical home for source implementations.
"""
module Sources

using ..Types: Vec3
using ..RigidBody: RigidBodyState
using ..Plant: PlantState
using ..Vehicles: ActuatorCommand
using ..Estimators: EstimatedState
using ..Faults: FaultState

using ..Recording: SampledTrace, ZOHTrace, SampleHoldTrace, sample
import ..Runtime: SimBus, update!, event_times_us

"""Base marker for any discrete-time source."""
abstract type AbstractSource end

"""Extract a rigid-body state from either `RigidBodyState` or `PlantState`."""
@inline _rb_state(state::RigidBodyState) = state
@inline _rb_state(state::PlantState) = state.rb
function _rb_state(state)
    error(
        "Sources: cannot extract rigid-body state from plant_state of type $(typeof(state))",
    )
end

@inline function _sanitize_bool(x)
    return x ? true : false
end

@inline function _sanitize_faults(f::FaultState)
    # Hook for schema validation if/when faults gain invariants.
    return f
end

include("Autopilot.jl")
include("Wind.jl")
include("Scenario.jl")
include("Estimator.jl")

end # module Sources
