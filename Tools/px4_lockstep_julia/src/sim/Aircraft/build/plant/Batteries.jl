"""Battery model helpers."""

include("battery/Assets.jl")
include("battery/Thermal.jl")
include("battery/Model.jl")

function _select_battery_spec(spec::AircraftSpec, id::BatteryId)
    for b in spec.power.batteries
        b.id == id && return b
    end
    throw(ArgumentError("Battery id=$(id) not found in spec.power.batteries"))
end

"""Build a battery model object from a `BatterySpec` (programmatic TOML-equivalent)."""
build_battery(spec::BatterySpec) = _build_battery(spec)

"""Build the tuple of battery *parameter objects* from `spec.power.batteries`.

Battery *state* lives in `Plant.PlantState` and is integrated by the plant model.
"""
function _build_batteries(spec::AircraftSpec)
    B = length(spec.power.batteries)
    B > 0 || throw(ArgumentError("power.batteries must be non-empty"))
    return ntuple(i -> _build_battery(spec.power.batteries[i]), Val(B))
end
