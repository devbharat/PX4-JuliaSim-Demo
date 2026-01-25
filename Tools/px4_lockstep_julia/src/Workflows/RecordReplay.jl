"""Workflows: Integrator comparison via record/replay.

This file turns the integrator-comparison examples into **first-class API**.

Motivation
----------
Closed-loop comparisons (running PX4 multiple times with different integrators) are
not isolating the integrator effect because PX4 issues different commands when the
plant diverges.

The canonical workflow is:

1. Record a closed-loop PX4 run once (Tier-0 streams: cmd, wind, scenario/faults).
2. Replay open-loop plant-only, sweeping integrators against a tight reference replay.

This is the only scientifically honest way to compare integrator envelopes in a
closed-loop system.
"""

using Dates
using Printf

# -----------------------------
# Small utilities
# -----------------------------

function _default_run_name(prefix::AbstractString)
    return prefix * "_" * Dates.format(Dates.now(), "yyyymmdd_HHMMSS")
end

function _slugify_label(label::AbstractString)
    s = lowercase(String(label))
    s = replace(s, r"[^a-z0-9]+" => "_")
    s = replace(s, r"^_+|_+$" => "")
    return isempty(s) ? "solver" : s
end

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

function _default_reference_integrator()
    return Integrators.RK45Integrator(
        rtol_pos = 1e-7,
        atol_pos = 1e-7,
        rtol_vel = 1e-7,
        atol_vel = 1e-7,
        rtol_ω = 1e-7,
        atol_ω = 1e-7,
        atol_att_rad = 1e-7,
        h_min = 1e-6,
        h_max = 0.02,
        plant_error_control = true,
        atol_rotor = 1e-2,
        atol_soc = 1e-6,
        atol_v1 = 1e-3,
    )
end

function _with_home(spec::Aircraft.AircraftSpec, home)
    return Aircraft.AircraftSpec(
        name = spec.name,
        px4 = spec.px4,
        timeline = spec.timeline,
        plant = spec.plant,
        airframe = spec.airframe,
        actuation = spec.actuation,
        power = spec.power,
        sensors = spec.sensors,
        seed = spec.seed,
        home = home,
        telemetry = spec.telemetry,
        log_sinks = spec.log_sinks,
    )
end

function _rms(v::AbstractVector{Float64})
    s = 0.0
    @inbounds for x in v
        s += x * x
    end
    return sqrt(s / length(v))
end

function _battery_error(
    ref::Vector{Powertrain.BatteryStatus},
    sol::Vector{Powertrain.BatteryStatus},
)
    @assert length(ref) == length(sol)
    n = length(ref)
    verr = Vector{Float64}(undef, n)
    ierr = Vector{Float64}(undef, n)
    @inbounds for i = 1:n
        verr[i] = abs(sol[i].voltage_v - ref[i].voltage_v)
        ierr[i] = abs(sol[i].current_a - ref[i].current_a)
    end
    return (
        max_v = maximum(verr),
        rms_v = _rms(verr),
        max_i = maximum(ierr),
        rms_i = _rms(ierr),
    )
end

"""Coerce a solver spec to `(label::String, integrator)`.

Accepted forms:
- `:RK23`, `:RK45`, ... (requires `make_integrator`)
- `"RK23"` (requires `make_integrator`)
- an integrator instance
- `"name" => integrator`
- `:name => integrator`
"""
function _coerce_solver(
    s;
    make_integrator::Function = (
        name::Symbol
    )->error("No make_integrator provided for solver name=$name"),
)
    if s isa Integrators.AbstractIntegrator
        return (string(typeof(s)), s)
    elseif s isa Pair
        label = string(first(s))
        integ = last(s)
        integ isa Integrators.AbstractIntegrator ||
            error("solver pair must map to an AbstractIntegrator")
        return (label, integ)
    elseif s isa Symbol
        return (String(s), make_integrator(s))
    elseif s isa AbstractString
        sym = Symbol(strip(String(s)))
        return (String(strip(String(s))), make_integrator(sym))
    else
        error("Unsupported solver spec type: $(typeof(s))")
    end
end

function _dt_from_axis(axis::Vector{UInt64})
    n = length(axis)
    n < 2 && return 0.0
    dt_us = axis[2] - axis[1]
    # Guard uniform axis.
    for i = 3:n
        (axis[i] - axis[i-1]) == dt_us ||
            error("Non-uniform log axis; cannot compare on a uniform grid")
    end
    return Float64(dt_us) * 1e-6
end

function _print_summary(rows)
    println()
    println("Summary (max error vs reference replay)")
    @printf(
        "%-10s  %7s  %10s  %10s  %10s  %11s  %12s  %9s  %9s\n",
        "solver",
        "wall_s",
        "pos_max",
        "vel_max",
        "att_max",
        "ω_max",
        "rotor_max",
        "|ΔV|",
        "|ΔI|",
    )

    for r in rows
        @printf(
            "%-10s  %7.3f  %10.3g  %10.3g  %10.3g  %11.3g  %12.3g  %9.3g  %9.3g\n",
            r.solver,
            r.wall_s,
            r.pos_max_m,
            r.vel_max_mps,
            r.att_max_rad,
            r.bodyrate_max_rad_s,
            r.rotor_omega_max_rad_s,
            r.battV_max_v,
            r.battI_max_a,
        )
    end
    return nothing
end

function _write_summary_csv(path::AbstractString, rows)
    open(path, "w") do io
        println(
            io,
            "solver,wall_s,pos_max_m,vel_max_mps,att_max_rad,bodyrate_max_rad_s,rotor_omega_max_rad_s,battV_max_v,battI_max_a",
        )
        for r in rows
            println(
                io,
                "$(r.solver),$(r.wall_s),$(r.pos_max_m),$(r.vel_max_mps),$(r.att_max_rad),$(r.bodyrate_max_rad_s),$(r.rotor_omega_max_rad_s),$(r.battV_max_v),$(r.battI_max_a)",
            )
        end
    end
    return path
end

# -----------------------------
# Generic compare on a Tier0 recording
# -----------------------------

"""Compare multiple integrators on a Tier-0 recording.

You must provide the plant model (`dynfun`) to use for replay.

The function runs:
- a **reference replay** (tight RK45, or user-provided `reference_integrator`)
- one replay per solver in `solvers`

and returns a vector of summary rows.

Notes
-----
- The recording must contain Tier-0 streams (`:cmd`, `:wind_ned`, and log samples).
- If the recording also contains scenario/fault streams, they are replayed as well.
  For plant-affecting faults, this is strongly recommended.
- If `log_dir` is provided, per-solver CSV logs are written as
  `<log_prefix>_ref_log.csv` and `<log_prefix>_<solver>_log.csv`.
"""
function compare_integrators_recording(;
    recording::Recording.Tier0Recording,
    dynfun,
    solvers,
    reference_integrator::Integrators.AbstractIntegrator,
    make_integrator::Function = (name::Symbol)->error("make_integrator required"),
    require_scenario::Bool = true,
    log_dir::Union{Nothing,AbstractString} = nothing,
    log_prefix::AbstractString = "compare",
    out_csv::Union{Nothing,AbstractString} = nothing,
    print_table::Bool = true,
)
    # Build traces.
    tr = Recording.tier0_traces(recording)

    scenario_src = Sources.NullScenarioSource()
    if require_scenario
        scn = Recording.scenario_traces(recording)
        wind_dist = hasproperty(scn, :wind_dist) ? scn.wind_dist : nothing
        scenario_src = Sources.ReplayScenarioSource(
            scn.ap_cmd,
            scn.landed,
            scn.faults;
            wind_dist = wind_dist,
        )
    else
        try
            scn = Recording.scenario_traces(recording)
            wind_dist = hasproperty(scn, :wind_dist) ? scn.wind_dist : nothing
            scenario_src = Sources.ReplayScenarioSource(
                scn.ap_cmd,
                scn.landed,
                scn.faults;
                wind_dist = wind_dist,
            )
        catch
            scenario_src = Sources.NullScenarioSource()
        end
    end

    ap_src = Sources.ReplayAutopilotSource(tr.cmd)
    wind_src = Sources.ReplayWindSource(tr.wind_ned)

    log_dir = log_dir === nothing ? nothing : String(log_dir)
    log_prefix = String(log_prefix)

    ref_log_sink = nothing
    if log_dir !== nothing
        mkpath(log_dir)
        ref_log_path = joinpath(log_dir, "$(log_prefix)_ref_log.csv")
        ref_log_sink = Logging.CSVLogSink(ref_log_path)
        println("  reference_log: $(ref_log_path)")
    end

    # Reference replay.
    ref_rec = Recording.InMemoryRecorder()
    ref_sim = Runtime.plant_replay_engine(
        timeline = recording.timeline,
        plant0 = recording.plant0,
        dynfun = dynfun,
        integrator = reference_integrator,
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario_src,
        estimator = Sources.NullEstimatorSource(),
        telemetry = Runtime.NullTelemetry(),
        recorder = ref_rec,
        log_sinks = ref_log_sink,
    )
    Runtime.run!(ref_sim)

    dt_log_s = _dt_from_axis(recording.timeline.log.t_us)
    ref_traj = Verification.Trajectory(dt_log_s, ref_rec.values[:plant])
    ref_batt = ref_rec.values[:battery]

    # Coerce solvers.
    solver_list = [_coerce_solver(s; make_integrator = make_integrator) for s in solvers]

    rows = Vector{NamedTuple}()

    for (label, integ) in solver_list
        t0 = time_ns()
        rec = Recording.InMemoryRecorder()

        log_sink = nothing
        if log_dir !== nothing
            slug = _slugify_label(label)
            log_path = joinpath(log_dir, "$(log_prefix)_$(slug)_log.csv")
            log_sink = Logging.CSVLogSink(log_path)
            println("  solver_log[$(label)]: $(log_path)")
        end

        sim = Runtime.plant_replay_engine(
            timeline = recording.timeline,
            plant0 = recording.plant0,
            dynfun = dynfun,
            integrator = integ,
            autopilot = ap_src,
            wind = wind_src,
            scenario = scenario_src,
            estimator = Sources.NullEstimatorSource(),
            telemetry = Runtime.NullTelemetry(),
            recorder = rec,
            log_sinks = log_sink,
        )
        Runtime.run!(sim)

        wall_s = (time_ns() - t0) * 1e-9

        traj = Verification.Trajectory(dt_log_s, rec.values[:plant])
        err = Verification.error_series(ref_traj, traj)
        berr = _battery_error(ref_batt, rec.values[:battery])

        row = (
            solver = label,
            wall_s = wall_s,
            pos_max_m = err.max.pos,
            vel_max_mps = err.max.vel,
            att_max_rad = err.max.att_rad,
            bodyrate_max_rad_s = err.max.ω,
            rotor_omega_max_rad_s = (haskey(err.max, :rotor) ? err.max.rotor : NaN),
            battV_max_v = berr.max_v,
            battI_max_a = berr.max_i,
        )
        push!(rows, row)
    end

    if print_table
        _print_summary(rows)
    end
    if out_csv !== nothing
        _write_summary_csv(out_csv, rows)
    end

    return rows
end

# -----------------------------
# Iris mission: clean UX wrapper
# -----------------------------

"""Compare integrators for an Iris PX4 waypoint mission with clean UX.

This is the primary entrypoint for integrator envelope validation.
You must provide either `spec_path` or `spec_name`.

Defaults:
- If `recording_in` is not provided, it records a Tier-0 run using the spec's integrator.
- Then it replays plant-only using the Tier-0 streams and sweeps `solvers`.

You can either:
- provide `recording_in` to skip recording, OR
- use a spec with `px4.mission_path` + `px4.libpath` to record a new run.

Outputs:
- prints a summary table
- optionally writes a summary CSV if `out_csv` is provided
- optionally writes per-solver CSV logs if `log_dir` is provided
"""
function compare_integrators_iris_mission(;
    spec_path::Union{Nothing,AbstractString} = nothing,
    spec_name::Union{Nothing,Symbol} = nothing,
    # Recording control
    recording_in::Union{Nothing,AbstractString} = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
    # Output
    out_csv::Union{Nothing,AbstractString} = nothing,
    out_dir::Union{Nothing,AbstractString} = nothing,
    run_name::Union{Nothing,AbstractString} = nothing,
    log_dir::Union{Nothing,AbstractString} = nothing,
    log_prefix::Union{Nothing,AbstractString} = nothing,
    # Solver sweep
    solvers = [:RK4, :RK23, :RK45],
    # Integrators
    reference_integrator::Integrators.AbstractIntegrator = _default_reference_integrator(),
    # Behavior
    print_table::Bool = true,
)
    if spec_path === nothing && spec_name === nothing
        error("spec_path or spec_name is required (e.g. spec_name=:iris_default).")
    end
    resolved_spec_path =
        spec_path === nothing ? Workflows.spec_path(spec_name) : String(spec_path)
    spec = Aircraft.load_spec(resolved_spec_path; strict = true)

    t_end_s = spec.timeline.t_end_s
    dt_autopilot_s = spec.timeline.dt_autopilot_s
    dt_wind_s = spec.timeline.dt_wind_s
    dt_log_s = spec.timeline.dt_log_s
    mission_path = spec.px4.mission_path
    libpath = spec.px4.libpath

    # Resolve run_name and output paths.
    run_name = run_name === nothing ? _default_run_name("iris") : String(run_name)
    log_prefix = log_prefix === nothing ? run_name : String(log_prefix)

    if out_dir !== nothing
        mkpath(out_dir)
        if recording_out === nothing
            recording_out = joinpath(out_dir, "$(run_name)_tier0.jls")
        end
        if out_csv === nothing
            out_csv = joinpath(out_dir, "$(run_name)_summary.csv")
        end
    end

    println("Iris integrator compare (record + replay)")
    println(
        "  t_end=$(t_end_s)s, dt_ap=$(dt_autopilot_s)s, dt_wind=$(dt_wind_s)s, dt_log=$(dt_log_s)s",
    )
    println("  solvers: ", join(string.(solvers), ", "))
    log_dir === nothing || println("  log_dir: $(log_dir)")

    # Acquire recording.
    rec = if recording_in !== nothing
        println("[compare_integrators_iris_mission] Using existing recording: $(recording_in)")
        Recording.read_recording(recording_in)
    else
        mission_path === nothing &&
            error("No recording_in provided and spec.px4.mission_path is not set.")
        libpath === nothing &&
            error("px4.libpath is required to record; set it in the TOML.")

        println("[compare_integrators_iris_mission] RECORD (PX4 live)")
        recording = simulate_iris_mission(
            spec_path = resolved_spec_path,
            mode = :record,
            recording_out = recording_out,
        )
        # `simulate_iris_mission(mode=:record)` returns a Tier0Recording.
        recording
    end

    if recording_out !== nothing && recording_in === nothing
        println("  recording_out: $(recording_out)")
    end

    # Reconstruct replay plant model from spec (NoWind env; wind comes from trace).
    home_rec = get(rec.meta, :home, spec.home)
    spec_replay = home_rec === spec.home ? spec : _with_home(spec, home_rec)
    inst, _ =
        Aircraft.build_aircraft_instance(spec_replay; mode = :replay, recording_in = rec)
    dynfun = inst.dynfun

    println("[compare_integrators_iris_mission] REPLAY (plant-only sweep)")

    rows = compare_integrators_recording(
        recording = rec,
        dynfun = dynfun,
        solvers = solvers,
        reference_integrator = reference_integrator,
        make_integrator = _make_integrator,
        require_scenario = true,
        log_dir = log_dir,
        log_prefix = log_prefix,
        out_csv = out_csv,
        print_table = print_table,
    )

    if out_csv !== nothing
        println("Wrote summary CSV → $(out_csv)")
    end

    return rows
end

export compare_integrators_recording, compare_integrators_iris_mission
