function _parse_estimator(tbl_any; strict::Bool, ctx::AbstractString, base::EstimatorSpec)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "kind",
        "pos_sigma_m",
        "vel_sigma_mps",
        "yaw_sigma_rad",
        "rate_sigma_rad_s",
        "bias_tau_s",
        "rate_bias_sigma_rad_s",
        "delay_s",
        "dt_est_s",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    kind = haskey(tbl, "kind") ? _norm_kind(tbl["kind"], "$ctx.kind") : base.kind
    kind = kind in (:none, :no, :off) ? :none : kind
    kind in (:none, :noisy_delayed) || error("$ctx.kind must be 'noisy_delayed' or 'none'")

    pos_sigma_m =
        haskey(tbl, "pos_sigma_m") ? _parse_vec3(tbl["pos_sigma_m"], "$ctx.pos_sigma_m") :
        base.pos_sigma_m
    vel_sigma_mps =
        haskey(tbl, "vel_sigma_mps") ?
        _parse_vec3(tbl["vel_sigma_mps"], "$ctx.vel_sigma_mps") : base.vel_sigma_mps
    yaw_sigma_rad =
        haskey(tbl, "yaw_sigma_rad") ? _as_f64(tbl["yaw_sigma_rad"], "$ctx.yaw_sigma_rad") :
        base.yaw_sigma_rad
    rate_sigma_rad_s =
        haskey(tbl, "rate_sigma_rad_s") ?
        _parse_vec3(tbl["rate_sigma_rad_s"], "$ctx.rate_sigma_rad_s") :
        base.rate_sigma_rad_s
    bias_tau_s =
        haskey(tbl, "bias_tau_s") ? _as_f64(tbl["bias_tau_s"], "$ctx.bias_tau_s") :
        base.bias_tau_s
    rate_bias_sigma_rad_s =
        haskey(tbl, "rate_bias_sigma_rad_s") ?
        _parse_vec3(tbl["rate_bias_sigma_rad_s"], "$ctx.rate_bias_sigma_rad_s") :
        base.rate_bias_sigma_rad_s
    delay_s =
        haskey(tbl, "delay_s") ?
        (tbl["delay_s"] === nothing ? nothing : _as_f64(tbl["delay_s"], "$ctx.delay_s")) :
        base.delay_s
    dt_est_s =
        haskey(tbl, "dt_est_s") ?
        (
            tbl["dt_est_s"] === nothing ? nothing :
            _as_f64(tbl["dt_est_s"], "$ctx.dt_est_s")
        ) : base.dt_est_s

    return EstimatorSpec(
        kind = kind,
        pos_sigma_m = pos_sigma_m,
        vel_sigma_mps = vel_sigma_mps,
        yaw_sigma_rad = yaw_sigma_rad,
        rate_sigma_rad_s = rate_sigma_rad_s,
        bias_tau_s = bias_tau_s,
        rate_bias_sigma_rad_s = rate_bias_sigma_rad_s,
        delay_s = delay_s,
        dt_est_s = dt_est_s,
    )
end
