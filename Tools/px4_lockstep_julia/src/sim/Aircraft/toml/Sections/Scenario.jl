function _parse_scenario(tbl_any; strict::Bool, ctx::AbstractString, base::ScenarioSpec)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set(["arm_time_s", "mission_time_s"])
    strict && _known_keys!(tbl, allowed, ctx)
    arm_time_s =
        haskey(tbl, "arm_time_s") ? _as_f64(tbl["arm_time_s"], "$ctx.arm_time_s") :
        base.arm_time_s
    mission_time_s =
        haskey(tbl, "mission_time_s") ?
        _as_f64(tbl["mission_time_s"], "$ctx.mission_time_s") : base.mission_time_s
    return ScenarioSpec(arm_time_s = arm_time_s, mission_time_s = mission_time_s)
end
