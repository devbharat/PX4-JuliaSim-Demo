"""Environment assembly helpers."""

function _build_atmosphere(env::EnvironmentSpec)
    env.atmosphere === :isa1976 ||
        throw(ArgumentError("Unsupported atmosphere=$(env.atmosphere)"))
    return Environment.ISA1976()
end

function _build_gravity(env::EnvironmentSpec)
    if env.gravity === :uniform
        return Environment.UniformGravity(env.gravity_mps2)
    elseif env.gravity === :spherical
        return Environment.SphericalGravity(env.gravity_mu, env.gravity_r0_m)
    end
    throw(ArgumentError("Unsupported gravity=$(env.gravity)"))
end

function _build_wind(env::EnvironmentSpec)
    if env.wind === :none
        return Environment.NoWind()
    elseif env.wind === :ou
        return Environment.OUWind(
            mean = env.wind_mean_ned,
            σ = env.wind_sigma_ned,
            τ_s = env.wind_tau_s,
        )
    elseif env.wind === :constant
        return Environment.ConstantWind(env.wind_mean_ned)
    end
    throw(ArgumentError("Unsupported wind=$(env.wind)"))
end

function _build_env_live(spec::AircraftSpec)
    envspec = spec.environment
    return Environment.EnvironmentModel(
        atmosphere = _build_atmosphere(envspec),
        wind = _build_wind(envspec),
        gravity = _build_gravity(envspec),
        origin = spec.home,
    )
end

function _build_env_replay(spec::AircraftSpec)
    envspec = spec.environment
    return Environment.EnvironmentModel(
        atmosphere = _build_atmosphere(envspec),
        wind = Environment.NoWind(),
        gravity = _build_gravity(envspec),
        origin = spec.home,
    )
end
