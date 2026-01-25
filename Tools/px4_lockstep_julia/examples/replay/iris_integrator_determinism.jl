# Iris integrator determinism check
#
# Runs the same integrator multiple times on a fixed recording to check that
# replay is bitwise-stable (or at least numerically identical at log resolution).
#
# Typical usage:
#
#   julia --project=Tools/px4_lockstep_julia \
#     Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl \
#     RK4 3 /path/to/spec.toml
#
# Or reuse an existing Tier-0 recording (no PX4 lib needed):
#
#   julia --project=Tools/px4_lockstep_julia \
#     Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl \
#     RK4 3 /path/to/spec.toml out/iris_YYYYMMDD_HHMMSS_tier0.jls

using PX4Lockstep
const Workflows = PX4Lockstep.Workflows
const Integrators = PX4Lockstep.Sim.Integrators

function _arg_or_nothing(i::Int)
    return length(ARGS) >= i ? ARGS[i] : nothing
end

solver_name = Symbol(_arg_or_nothing(1) === nothing ? "RK4" : ARGS[1])
repeat_count = _arg_or_nothing(2) === nothing ? 3 : parse(Int, ARGS[2])
repeat_count >= 1 || error("IRIS_DETERMINISM_N must be >= 1")

function _make_integrator(name::Symbol)
    if name === :Euler
        return Integrators.EulerIntegrator()
    elseif name === :RK4
        return Integrators.RK4Integrator()
    elseif name === :RK23
        return Integrators.RK23Integrator()
    elseif name === :RK45
        return Integrators.RK45Integrator()
    else
        error("Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)")
    end
end

base = _make_integrator(solver_name)
label_root = lowercase(String(solver_name))

solvers = [
    Symbol("$(label_root)_run$(i)") => deepcopy(base)
    for i in 1:repeat_count
]

reference_integrator = deepcopy(base)
spec_path = _arg_or_nothing(3)
recording_in = _arg_or_nothing(4)
out_dir = _arg_or_nothing(5)
log_dir = _arg_or_nothing(6)

spec_path = spec_path === "" ? nothing : spec_path
recording_in = recording_in === "" ? nothing : recording_in
out_dir = out_dir === nothing ? joinpath(@__DIR__, "out") : out_dir
log_dir = log_dir === "" ? nothing : log_dir
spec_name = spec_path === nothing ? :iris_default : nothing

Workflows.compare_integrators_iris_mission(
    solvers = solvers,
    reference_integrator = reference_integrator,
    out_dir = out_dir,
    log_dir = log_dir,
    spec_path = spec_path,
    spec_name = spec_name,
    recording_in = recording_in,
)
