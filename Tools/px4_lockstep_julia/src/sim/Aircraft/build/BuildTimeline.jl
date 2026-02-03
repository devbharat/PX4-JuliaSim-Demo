module BuildTimeline

"""Timeline assembly helpers."""

using ...Runtime

export _build_default_timeline

function _build_default_timeline(;
    t_end_s::Float64,
    dt_autopilot_s::Float64,
    dt_wind_s::Float64,
    dt_log_s::Float64,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    scenario_source = nothing,
)
    t0_us = UInt64(0)
    t_end_us = Runtime.dt_to_us(t_end_s)
    dt_ap_us = Runtime.dt_to_us(dt_autopilot_s)
    dt_wind_us = Runtime.dt_to_us(dt_wind_s)
    dt_log_us = Runtime.dt_to_us(dt_log_s)
    dt_phys_us = dt_phys_s === nothing ? nothing : Runtime.dt_to_us(dt_phys_s)

    return Runtime.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scenario = scenario_source,
    )
end

end # module BuildTimeline
