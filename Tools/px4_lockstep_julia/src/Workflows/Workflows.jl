"""PX4Lockstep.Workflows

Aircraft- and mission-specific workflows built on top of the generic Sim engine.
"""
module Workflows

using ..Sim:
    Aircraft, Integrators, Logging, Recording, Runtime, Sources, Verification, Powertrain

include("Assets.jl")
include("Iris.jl")
include("RecordReplay.jl")

export spec_path,
    mission_path,
    list_specs,
    list_missions,
    simulate_iris_mission,
    compare_integrators_recording,
    compare_integrators_iris_mission

end
