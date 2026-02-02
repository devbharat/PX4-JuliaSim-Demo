"""Public engine build entrypoint."""

"""Build and run an engine from an `AircraftSpec`.

This is the stable build entrypoint used by `Workflows.simulate_iris_mission`.

Modes
-----
* `:live`   -> return `Runtime.Engine`
* `:record` -> return `Recording.Tier0Recording` (and optionally write to `recording_out`)
* `:replay` -> return `Runtime.Engine`
"""
function build_engine(
    spec::AircraftSpec;
    mode::Symbol = :live,
    recording_in = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
    telemetry = spec.telemetry,
    log_sinks = spec.log_sinks,
    report_timing::Bool = false,
)
    validate_spec(spec; mode = mode, recording_in = recording_in)

    # Multirotor specs only. Additional airframe kinds will be enabled
    # in later phases.

    inst = build_aircraft_instance(
        spec;
        mode = mode,
        recording_in = recording_in,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )

    ap = inst.px4_handle

    try
        if mode === :record
            recorder = Recording.InMemoryRecorder()
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :record,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                recorder = recorder,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end

            rec = Recording.Tier0Recording(
                timeline = inst.timeline,
                plant0 = inst.plant0,
                recorder = recorder,
                meta = inst.meta,
            )
            if recording_out !== nothing
                Recording.write_recording(recording_out, rec)
            end
            return rec

        elseif mode === :replay
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :replay,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end
            return eng

        elseif mode === :live
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :live,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end
            return eng

        else
            throw(
                ArgumentError(
                    "build_engine: unknown mode=$(mode) (expected :live|:record|:replay)",
                ),
            )
        end
    finally
        ap === nothing || Autopilots.close!(ap)
    end
end
