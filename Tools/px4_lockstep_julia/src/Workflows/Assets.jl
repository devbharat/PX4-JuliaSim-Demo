"""Workflows assets registry.

Provides named access to built-in spec and mission files.
"""

const _WORKFLOWS_ROOT = @__DIR__
const _ASSETS_ROOT = joinpath(_WORKFLOWS_ROOT, "assets")

const _SPEC_MAP = Dict{Symbol,String}(
    :iris_default => joinpath(_ASSETS_ROOT, "aircraft", "iris_default.toml"),
    :iris_uorb => joinpath(_ASSETS_ROOT, "aircraft", "iris_uorb.toml"),
    :minimal_uorb => joinpath(_ASSETS_ROOT, "aircraft", "minimal_uorb.toml"),
    :octa_default => joinpath(_ASSETS_ROOT, "aircraft", "octa_default.toml"),
)

const _MISSION_MAP = Dict{Symbol,String}(
    :simple => joinpath(_ASSETS_ROOT, "missions", "simple_mission.waypoints"),
    :long_20min => joinpath(_ASSETS_ROOT, "missions", "long_mission_20min.waypoints"),
)

"""Return a named built-in spec path.

Supported names: :iris_default, :iris_uorb, :minimal_uorb, :octa_default.
"""
function spec_path(name::Symbol)
    path = get(_SPEC_MAP, name, nothing)
    path === nothing && error(
        "Unknown spec name=$(name). Available: $(join(sort!(collect(keys(_SPEC_MAP))), ", "))",
    )
    return path
end

"""Return a named built-in mission path.

Supported names: :simple, :long_20min.
"""
function mission_path(name::Symbol)
    path = get(_MISSION_MAP, name, nothing)
    path === nothing && error(
        "Unknown mission name=$(name). Available: $(join(sort!(collect(keys(_MISSION_MAP))), ", "))",
    )
    return path
end

"""List available spec names."""
list_specs() = sort!(collect(keys(_SPEC_MAP)))

"""List available mission names."""
list_missions() = sort!(collect(keys(_MISSION_MAP)))

export spec_path, mission_path, list_specs, list_missions
