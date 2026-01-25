"""Iris workflow helpers.

This module is TOML-first: the Iris configuration lives in the built-in TOML
assets, and workflows load those specs explicitly.
"""

"""Run an Iris mission with a clean, top-level UX.

Modes
-----
- `mode=:live`   : step PX4 live (no recording unless a recorder is provided)
- `mode=:record` : live PX4 + in-memory recorder, returns a `Tier0Recording`
- `mode=:replay` : replay from a Tier0Recording (or path) with deterministic sources

This is intended as the go-to user workflow.
You must provide either:
- `spec_path` (explicit path), or
- `spec_name` (named lookup via `Workflows.spec_path`).
"""
function simulate_iris_mission(;
    spec_path::Union{Nothing,AbstractString} = nothing,
    spec_name::Union{Nothing,Symbol} = nothing,
    mode::Symbol = :live,
    recording_in::Union{Nothing,AbstractString} = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
)
    if spec_path === nothing && spec_name === nothing
        error("spec_path or spec_name is required (e.g. spec_name=:iris_default).")
    end
    resolved_spec_path =
        spec_path === nothing ? Workflows.spec_path(spec_name) : String(spec_path)
    return Aircraft.run_spec(
        resolved_spec_path;
        mode = mode,
        recording_in = recording_in,
        recording_out = recording_out,
        strict = true,
    )
end

export simulate_iris_mission
