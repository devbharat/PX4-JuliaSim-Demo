# Iris integrator compare (clean UX)
#
# This script is intentionally thin: it delegates all logic to the
# first-class workflow wrapper in `Workflows/RecordReplay.jl`.
#
# Typical usage:
#
#   julia --project=Tools/px4_lockstep_julia \
#     Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl \
#     /path/to/spec.toml
#
# Or replay an existing Tier-0 recording (no PX4 lib needed):
#
#   julia --project=Tools/px4_lockstep_julia \
#     Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl \
#     /path/to/spec.toml out/iris_20260101_120000_tier0.jls

using PX4Lockstep
const Workflows = PX4Lockstep.Workflows

out_dir = joinpath(@__DIR__, "out")

spec_path = isempty(ARGS) ? nothing : ARGS[1]
recording_in = length(ARGS) >= 2 ? ARGS[2] : nothing

spec_path = spec_path === "" ? nothing : spec_path
recording_in = recording_in === "" ? nothing : recording_in
spec_name = spec_path === nothing ? :iris_default : nothing

Workflows.compare_integrators_iris_mission(
    out_dir = out_dir,
    spec_path = spec_path,
    spec_name = spec_name,
    recording_in = recording_in,
)
