"""PX4Lockstep.Sim.RecordReplay

First-class record/replay architecture.

This is the **Option A** design: an event-sourced, typed **signal bus** plus
replayable component sources.

Status
------
This module is intentionally introduced in a **top-down skeleton** form:

* Types, interfaces, and documentation are in place.
* Most methods contain explicit `TODO` placeholders.

The goal is to make the next iterations mechanical:
fill in the TODOs while keeping the public API stable.

See also
--------
* `docs/record_replay.md` (design doc)
* `docs/record_replay_todo.md` (living checklist)
"""
module RecordReplay

# Public surface (initial; expected to grow).
export BUS_SCHEMA_VERSION,
    TimeAxis,
    Timeline,
    dt_to_us,
    periodic_axis,
    build_timeline,
    build_timeline_for_run,
    on_axis,
    SimBus,
    reset_bus!,
    AbstractTrace,
    SampledTrace,
    ZOHTrace,
    SampleHoldTrace,
    sample,
    AbstractRecorder,
    NullRecorder,
    InMemoryRecorder,
    Tier0Recording,
    save_recording,
    load_recording,
    record!,
    finalize!,
    stream_times,
    stream_values,
    zoh_trace,
    samplehold_trace,
    sampled_trace,
    tier0_traces,
    scenario_traces,
    estimator_traces,
    AbstractSource,
    AbstractAutopilotSource,
    AbstractWindSource,
    AbstractScenarioSource,
    AbstractEstimatorSource,
    NullScenarioSource,
    NullEstimatorSource,
    ReplayAutopilotSource,
    ReplayWindSource,
    LiveAutopilotSource,
    LiveWindSource,
    LiveScenarioSource,
    ReplayScenarioSource,
    LiveEstimatorSource,
    ReplayEstimatorSource,
    EngineMode,
    EngineConfig,
    BusEngine,
    run!,
    plant_replay_engine,
    plant_record_engine

include("RecordReplay/Timeline.jl")
include("RecordReplay/Bus.jl")
include("RecordReplay/Traces.jl")
include("RecordReplay/Recorder.jl")

# Recording containers + persistence helpers.
include("RecordReplay/Recording.jl")
include("RecordReplay/Sources.jl")
include("RecordReplay/Builders.jl")
include("RecordReplay/Engine.jl")

end # module RecordReplay
