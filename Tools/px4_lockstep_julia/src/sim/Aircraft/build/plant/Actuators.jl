"""Actuator model helpers."""

function _build_actuator_model(mspec::AbstractActuatorModelSpec, N::Int)
    if mspec isa DirectActuatorSpec
        return Vehicles.DirectActuators()
    elseif mspec isa FirstOrderActuatorSpec
        return Vehicles.FirstOrderActuators{N}(τ = mspec.τ)
    elseif mspec isa SecondOrderActuatorSpec
        return Vehicles.SecondOrderActuators{N}(
            ωn = mspec.ωn,
            ζ = mspec.ζ,
            rate_limit = mspec.rate_limit,
        )
    else
        throw(ArgumentError("Unknown actuator model spec: $(typeof(mspec))"))
    end
end
