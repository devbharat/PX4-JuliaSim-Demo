"""PX4Lockstep.Tests

Precompile-friendly test workloads and shared helpers for test code.
"""
module Tests

using Random
using StaticArrays: SVector

using ..Sim
using ..Workflows

# Re-export commonly-used helpers for test code.
export tier1_subsystems,
    tier2_fullplant_results,
    compare_integrators_recording_results,
    record_replay_equivalence_log_ticks,
    record_replay_equivalence_wind_disturbance,
    iris_replay_parity_result,
    iris_spec_for_tests,
    iris_home_for_tests,
    iris_contact_for_tests,
    iris_env_live_for_tests,
    iris_env_replay_for_tests,
    iris_vehicle_for_tests,
    iris_batteries_for_tests,
    iris_battery_for_tests,
    iris_dynfun_for_tests,
    iris_scenario_for_tests,
    iris_timeline_for_tests,
    integrator_from_symbol

# Short module aliases to keep helper code readable.
const Types = Sim.Types
const Environment = Sim.Environment
const Powertrain = Sim.Powertrain
const Aircraft = Sim.Aircraft
const RigidBody = Sim.RigidBody
const Integrators = Sim.Integrators
const Scenario = Sim.Scenario
const Events = Sim.Events
const Vehicles = Sim.Vehicles
const Propulsion = Sim.Propulsion
const Contacts = Sim.Contacts
const Plant = Sim.Plant
const PlantModels = Sim.PlantModels
const Faults = Sim.Faults
const Runtime = Sim.Runtime
const Sources = Sim.Sources
const Recording = Sim.Recording
const Autopilots = Sim.Autopilots
const Estimators = Sim.Estimators
const Verification = Sim.Verification

include("Core.jl")
include("Tier1.jl")
include("Tier2.jl")
include("RecordReplay.jl")
include("Fixtures.jl")
include("Workloads.jl")

end # module Tests
