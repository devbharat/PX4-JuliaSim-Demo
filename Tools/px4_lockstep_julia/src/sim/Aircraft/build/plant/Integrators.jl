"""Integrator selection helpers."""

@inline function _integrator_from_symbol(name::Symbol)
    if name === :Euler
        return Integrators.EulerIntegrator()
    elseif name === :RK4
        return Integrators.RK4Integrator()
    elseif name === :RK23
        return Integrators.RK23Integrator()
    elseif name === :RK45
        return Integrators.RK45Integrator()
    else
        throw(
            ArgumentError(
                "Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)",
            ),
        )
    end
end

@inline function _resolve_integrator(spec::AircraftSpec)
    integ = spec.plant.integrator
    if integ isa Symbol
        return _integrator_from_symbol(integ)
    end
    return integ
end
