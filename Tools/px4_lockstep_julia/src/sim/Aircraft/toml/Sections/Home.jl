function _parse_home(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::Autopilots.HomeLocation,
)
    h = _as_table(tbl_any, ctx)
    strict && _known_keys!(h, Set(["lat_deg", "lon_deg", "alt_msl_m"]), ctx)
    return Autopilots.HomeLocation(
        lat_deg = haskey(h, "lat_deg") ? _as_f64(h["lat_deg"], "$ctx.lat_deg") :
                  base.lat_deg,
        lon_deg = haskey(h, "lon_deg") ? _as_f64(h["lon_deg"], "$ctx.lon_deg") :
                  base.lon_deg,
        alt_msl_m = haskey(h, "alt_msl_m") ? _as_f64(h["alt_msl_m"], "$ctx.alt_msl_m") :
                    base.alt_msl_m,
    )
end
