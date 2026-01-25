#!/usr/bin/env julia

"""Run an aircraft spec from a TOML file.

Usage
-----
  julia --project=. examples/run_spec.jl <spec.toml> [live|record|replay] [recording_path]

Examples
--------
From `Tools/px4_lockstep_julia`:
  julia --project=. examples/run_spec.jl examples/specs/iris.toml
  julia --project=. examples/run_spec.jl examples/specs/iris.toml record /tmp/iris.tier0
  julia --project=. examples/run_spec.jl examples/specs/iris.toml replay /tmp/iris.tier0

From the PX4 root:
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/run_spec.jl Tools/px4_lockstep_julia/examples/specs/iris.toml
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/run_spec.jl Tools/px4_lockstep_julia/examples/specs/iris.toml record /tmp/iris.tier0
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/run_spec.jl Tools/px4_lockstep_julia/examples/specs/iris.toml replay /tmp/iris.tier0
"""

using PX4Lockstep

if isempty(ARGS)
    error("Missing spec path. See docstring in examples/run_spec.jl")
end

spec_path = ARGS[1]
mode = length(ARGS) >= 2 ? Symbol(lowercase(ARGS[2])) : nothing
rec_path = length(ARGS) >= 3 ? ARGS[3] : nothing

if mode === :record
    rec_path === nothing && (rec_path = "recording.tier0")
    rec = PX4Lockstep.Sim.Aircraft.run_spec(spec_path; mode = :record, recording_out = rec_path)
    println("Recorded Tier0 to: ", rec_path)
elseif mode === :replay
    rec_path === nothing && error("replay requires a recording path")
    PX4Lockstep.Sim.Aircraft.run_spec(spec_path; mode = :replay, recording_in = rec_path)
elseif mode === :live || mode === nothing
    PX4Lockstep.Sim.Aircraft.run_spec(spec_path; mode = mode)
else
    error("Unknown mode '$mode' (expected live|record|replay)")
end
