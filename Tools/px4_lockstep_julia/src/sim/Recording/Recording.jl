"""PX4Lockstep.Sim.Recording

Recording and replay infrastructure.

This module owns:

* trace types (sampled / ZOH / sample-hold)
* recorders and persistence helpers

This module is the canonical home for trace and recorder types.
"""
module Recording

include("Traces.jl")
include("Recorder.jl")
include("Tier0.jl")

end # module Recording
