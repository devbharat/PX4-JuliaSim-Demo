"""PX4Lockstep.Sim.Runtime

Canonical simulation runtime.

Target architecture
-------------------
This module is intended to become the **single authoritative engine** for the simulator.
It owns:

* event timeline traversal (union of all cadences + scenario boundaries)
* deterministic discrete source stepping order
* continuous plant integration between boundaries
* record/replay modes via pluggable Sources + Recorder sinks

Migration note
--------------
This module is the canonical home for the engine API. Source implementations live
under `Sim.Sources` and recorder/trace implementations live under `Sim.Recording`.

See:
* `docs/engine_unification.md`
* `docs/engine_unification_todo.md`
"""
module Runtime

include("Bus.jl")
include("Timeline.jl")

# Canonical boundary semantics and scheduling.
include("BoundaryProtocol.jl")
include("Scheduler.jl")
include("Validation.jl")

# Optional deterministic boundary hooks.
include("Telemetry.jl")

include("Engine.jl")

end # module Runtime
