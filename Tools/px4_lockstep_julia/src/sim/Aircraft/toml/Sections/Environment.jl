function _parse_environment(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::EnvironmentSpec,
)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "wind",
        "wind_mean_ned",
        "wind_sigma_ned",
        "wind_tau_s",
        "atmosphere",
        "gravity",
        "gravity_mps2",
        "gravity_mu",
        "gravity_r0_m",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    wind = haskey(tbl, "wind") ? _norm_kind(tbl["wind"], "$ctx.wind") : base.wind
    wind =
        wind in (:none, :no, :off, :no_wind) ? :none :
        wind in (:ou, :ou_wind, :turbulence) ? :ou :
        wind in (:constant, :const) ? :constant : wind
    wind in (:none, :ou, :constant) ||
        error("$ctx.wind must be one of 'none', 'ou', 'constant' (got $(wind))")

    wind_mean_ned =
        haskey(tbl, "wind_mean_ned") ?
        _parse_vec3(tbl["wind_mean_ned"], "$ctx.wind_mean_ned") : base.wind_mean_ned
    wind_sigma_ned =
        haskey(tbl, "wind_sigma_ned") ?
        _parse_vec3(tbl["wind_sigma_ned"], "$ctx.wind_sigma_ned") : base.wind_sigma_ned
    wind_tau_s =
        haskey(tbl, "wind_tau_s") ? _as_f64(tbl["wind_tau_s"], "$ctx.wind_tau_s") :
        base.wind_tau_s

    atmosphere =
        haskey(tbl, "atmosphere") ? _norm_kind(tbl["atmosphere"], "$ctx.atmosphere") :
        base.atmosphere
    atmosphere in (:isa1976, :isa) || error("$ctx.atmosphere must be 'isa1976'")

    gravity =
        haskey(tbl, "gravity") ? _norm_kind(tbl["gravity"], "$ctx.gravity") : base.gravity
    gravity = gravity in (:uniform, :spherical) ? gravity : gravity
    gravity in (:uniform, :spherical) ||
        error("$ctx.gravity must be 'uniform' or 'spherical'")

    g_mps2 =
        haskey(tbl, "gravity_mps2") ? _as_f64(tbl["gravity_mps2"], "$ctx.gravity_mps2") :
        base.gravity_mps2
    g_mu =
        haskey(tbl, "gravity_mu") ? _as_f64(tbl["gravity_mu"], "$ctx.gravity_mu") :
        base.gravity_mu
    g_r0 =
        haskey(tbl, "gravity_r0_m") ? _as_f64(tbl["gravity_r0_m"], "$ctx.gravity_r0_m") :
        base.gravity_r0_m

    return EnvironmentSpec(
        wind = wind,
        wind_mean_ned = wind_mean_ned,
        wind_sigma_ned = wind_sigma_ned,
        wind_tau_s = wind_tau_s,
        atmosphere = atmosphere == :isa ? :isa1976 : atmosphere,
        gravity = gravity,
        gravity_mps2 = g_mps2,
        gravity_mu = g_mu,
        gravity_r0_m = g_r0,
    )
end
