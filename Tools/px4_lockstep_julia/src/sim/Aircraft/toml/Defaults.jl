# Internal default spec (generic multirotor)

const _DEFAULT_MULTIROTOR_SPEC_PATH =
    joinpath(@__DIR__, "..", "assets", "multirotor_default.toml")
const _DEFAULT_MULTIROTOR_SPEC_CACHE = Ref{Union{Nothing,AircraftSpec}}(nothing)

"""Path to the built-in generic multirotor spec (Sim internal)."""
default_multirotor_spec_path() = _DEFAULT_MULTIROTOR_SPEC_PATH

"""Load the built-in generic multirotor spec (Sim internal)."""
function default_multirotor_spec(; strict::Bool = true)
    return _default_multirotor_spec_cached(strict = strict)
end

function _default_multirotor_spec_cached(; strict::Bool)
    spec = _DEFAULT_MULTIROTOR_SPEC_CACHE[]
    spec !== nothing && return spec
    cfg = _load_toml_with_extends(_DEFAULT_MULTIROTOR_SPEC_PATH; strict = strict)
    base_dir = dirname(abspath(_DEFAULT_MULTIROTOR_SPEC_PATH))
    spec = spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict)
    _DEFAULT_MULTIROTOR_SPEC_CACHE[] = spec
    return spec
end

function _resolve_base_spec(path::AbstractString, base_spec; strict::Bool)
    if base_spec === :default
        abspath(path) == abspath(_DEFAULT_MULTIROTOR_SPEC_PATH) && return nothing
        return _default_multirotor_spec_cached(strict = strict)
    elseif base_spec === nothing || base_spec isa AircraftSpec
        return base_spec
    end
    error("base_spec must be :default, nothing, or an AircraftSpec")
end
