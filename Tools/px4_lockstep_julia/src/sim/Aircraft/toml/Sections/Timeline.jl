function _parse_timeline(tbl_any; strict::Bool, ctx::AbstractString, base::TimelineSpec)
    t = _as_table(tbl_any, ctx)
    strict && _known_keys!(
        t,
        Set(["t_end_s", "dt_autopilot_s", "dt_wind_s", "dt_log_s", "dt_phys_s"]),
        ctx,
    )
    return TimelineSpec(
        t_end_s = haskey(t, "t_end_s") ? _as_f64(t["t_end_s"], "$ctx.t_end_s") :
                  base.t_end_s,
        dt_autopilot_s = haskey(t, "dt_autopilot_s") ?
                         _as_f64(t["dt_autopilot_s"], "$ctx.dt_autopilot_s") :
                         base.dt_autopilot_s,
        dt_wind_s = haskey(t, "dt_wind_s") ? _as_f64(t["dt_wind_s"], "$ctx.dt_wind_s") :
                    base.dt_wind_s,
        dt_log_s = haskey(t, "dt_log_s") ? _as_f64(t["dt_log_s"], "$ctx.dt_log_s") :
                   base.dt_log_s,
        dt_phys_s = haskey(t, "dt_phys_s") ?
                    (
            t["dt_phys_s"] === nothing ? nothing :
            _as_f64(t["dt_phys_s"], "$ctx.dt_phys_s")
        ) : base.dt_phys_s,
    )
end
