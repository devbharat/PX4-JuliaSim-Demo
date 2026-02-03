module BuildPlant

"""Plant, environment, and integrator assembly helpers."""

using StaticArrays

using ..Aircraft:
    AircraftSpec,
    BatterySpec,
    BatteryId,
    MotorId,
    EnvironmentSpec,
    AbstractActuatorModelSpec,
    DirectActuatorSpec,
    FirstOrderActuatorSpec,
    SecondOrderActuatorSpec
using ..BatteryAssets
using ...Environment
using ...Integrators
using ...Vehicles
using ...Propulsion
using ...Powertrain
using ...PlantModels
using ...PlantModels: PowerNetwork
using ...Plant
using ...Types
using ...RigidBody: RigidBodyState

export _build_env_live,
    _build_env_replay,
    _build_vehicle,
    _build_batteries,
    _build_power_network,
    _resolve_integrator,
    build_battery

include("plant/Integrators.jl")
include("plant/Environment.jl")
include("plant/Actuators.jl")
include("plant/Batteries.jl")
include("plant/PowerNetwork.jl")
include("plant/Vehicle.jl")

end # module BuildPlant
