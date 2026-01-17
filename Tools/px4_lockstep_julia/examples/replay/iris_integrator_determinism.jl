# Iris integrator determinism check
#
# Runs the same integrator multiple times on a fixed recording to check that
# replay is bitwise-stable (or at least numerically identical at log resolution).
#
# Typical usage:
#
#   PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
#   IRIS_DETERMINISM_SOLVER=RK4 IRIS_DETERMINISM_N=3 \
#   IRIS_LOG_DIR=Tools/px4_lockstep_julia/examples/replay/out \
#     julia --project=Tools/px4_lockstep_julia \
#       Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl
#
# Or reuse an existing Tier-0 recording:
#
#   IRIS_RECORD_IN=Tools/px4_lockstep_julia/examples/replay/out/iris_YYYYMMDD_HHMMSS_tier0.jls \
#   IRIS_DETERMINISM_SOLVER=RK4 IRIS_DETERMINISM_N=3 \
#   IRIS_LOG_DIR=Tools/px4_lockstep_julia/examples/replay/out \
#     julia --project=Tools/px4_lockstep_julia \
#       Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl

using PX4Lockstep
const Sim = PX4Lockstep.Sim

function _env_int(name::AbstractString, default::Int)
    return parse(Int, get(ENV, name, string(default)))
end

solver_name = Symbol(get(ENV, "IRIS_DETERMINISM_SOLVER", "RK4"))
repeat_count = _env_int("IRIS_DETERMINISM_N", 3)
repeat_count >= 1 || error("IRIS_DETERMINISM_N must be >= 1")

base = Sim.iris_integrator(solver_name)
label_root = lowercase(String(solver_name))

solvers = [
    Symbol("$(label_root)_run$(i)") => deepcopy(base)
    for i in 1:repeat_count
]

reference_integrator = deepcopy(base)
out_dir = get(ENV, "IRIS_OUT_DIR", joinpath(@__DIR__, "out"))
log_dir = get(ENV, "IRIS_LOG_DIR", nothing)

Sim.compare_integrators_iris_mission(
    solvers = solvers,
    reference_integrator = reference_integrator,
    out_dir = out_dir,
    log_dir = log_dir,
)
