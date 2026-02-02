"""Internal build output.

This is intentionally *not* exported yet. It is the structured result of
`build_aircraft_instance(...)` and is consumed by `build_engine(...)`.
"""
Base.@kwdef struct BuildParts{TL,PS,D,INT,S}
    timeline::TL
    plant0::PS
    dynfun::D
    integrator::INT
    sources::S
    meta::Dict{Symbol,Any} = Dict{Symbol,Any}()
    run_mode::Symbol = :live
    telemetry = nothing
    log_sinks = nothing
    px4_handle = nothing
end
