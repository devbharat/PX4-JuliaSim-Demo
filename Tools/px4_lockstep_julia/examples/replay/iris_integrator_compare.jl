# Iris integrator compare (clean UX)
#
# This script is intentionally thin: it delegates all logic to the
# first-class workflow wrapper in `Sim.Workflows/CompareIntegrators.jl`.
#
# Typical usage:
#
#   PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
#     IRIS_SWEEP_SOLVERS=RK4,RK23,RK45 \
#     IRIS_T_END_S=20.0 IRIS_OUT_DIR=out \
#     julia --project=Tools/px4_lockstep_julia \
#       Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
#
# Or replay an existing Tier-0 recording:
#
#   IRIS_RECORD_IN=out/iris_20260101_120000_tier0.jls \
#     IRIS_SWEEP_SOLVERS=RK4,RK23,RK45 \
#     julia --project=Tools/px4_lockstep_julia \
#       Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl

using PX4Lockstep
const Sim = PX4Lockstep.Sim

out_dir = joinpath(@__DIR__, "out")
Sim.compare_integrators_iris_mission(out_dir=out_dir)
