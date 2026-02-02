function _parse_sensors(tbl_any; strict::Bool, ctx::AbstractString)
    # Support both:
    #   [[sensors]] kind="gps" ...   (top-level array)
    # and:
    #   [sensors]
    #   sensors = [ ... ]            (table wrapping)
    #   or
    #   [[sensors.sensors]] ...      (array under [sensors])

    arr_any = nothing
    if tbl_any isa AbstractVector
        arr_any = tbl_any
    else
        tbl = _as_table(tbl_any, ctx)
        strict && _known_keys!(tbl, Set(["sensors"]), ctx)
        arr_any = get(tbl, "sensors", nothing)
    end

    arr_any === nothing && return AbstractSensorSpec[]
    arr = _as_array(arr_any, ctx)
    out = AbstractSensorSpec[]
    for (i, s_any) in enumerate(arr)
        s = _as_table(s_any, "$ctx[$i]")
        strict && _known_keys!(s, Set(["kind", "id", "uorb_instance"]), "$ctx[$i]")
        kind = lowercase(_as_string(get(s, "kind", nothing), "$ctx[$i].kind"))
        id = _sym(_as_string(get(s, "id", nothing), "$ctx[$i].id"))
        inst =
            haskey(s, "uorb_instance") ?
            _as_int(s["uorb_instance"], "$ctx[$i].uorb_instance") : 0
        if kind == "gps"
            push!(out, GpsSpec(id = id, uorb_instance = inst))
        elseif kind in ("rangefinder", "distance")
            push!(out, RangefinderSpec(id = id, uorb_instance = inst))
        elseif kind == "radar"
            push!(out, RadarSpec(id = id, uorb_instance = inst))
        else
            error(
                "$ctx[$i].kind: unknown sensor kind '$kind' (expected gps|rangefinder|radar)",
            )
        end
    end
    return out
end
