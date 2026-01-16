#!/usr/bin/env julia
"""One-shot Iris integrator comparison (record + replay, clean UX).

This is the recommended **PX4-in-the-loop integrator comparison** workflow.

Why this exists
---------------
Closed-loop integrator comparisons are often misleading because small differences
in the plant integration can cause PX4 to issue different commands. The record/replay
architecture lets you isolate the integrator choice:

1) **Record** a closed-loop run (PX4 live) using a high-accuracy integrator.
   This captures deterministic Tier-0 streams:
   * actuator commands on autopilot ticks
   * wind samples on wind ticks
   * scenario outputs (including faults) on scenario/event ticks

2) **Replay** the plant open-loop using the recorded streams, sweeping integrators
   and comparing each to a high-accuracy reference replay.

Usage
-----
Just run it:

```bash
PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib) \
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia -O3 Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
```

If you already have a recording and only want the replay sweep:

```bash
IRIS_RECORD_IN=/path/to/iris_recording.jls \
  julia --project -O3 examples/replay/iris_integrator_compare.jl
```

Environment variables
---------------------
Recording (when `IRIS_RECORD_IN` is NOT set):
* `PX4_LOCKSTEP_MISSION` (required)
* `PX4_LOCKSTEP_LIB` (optional; falls back to PX4Lockstep default search)

Common:
* `IRIS_OUT_DIR` (default: `examples/replay/out`)
* `IRIS_RUN_NAME` (default: auto timestamp)
* `IRIS_T_END_S` (default: 20.0)
* `IRIS_DT_AUTOPILOT_S` (default: 0.004)
* `IRIS_DT_WIND_S` (default: 0.002)
* `IRIS_DT_LOG_S` (default: 0.01)
* `IRIS_SEED` (default: 1)

Replay sweep:
* `IRIS_SWEEP_SOLVERS` (default: "RK4,RK23,RK45")

Outputs
-------
In `IRIS_OUT_DIR` (or `examples/replay/out`):
* `<run>_tier0.jls` : Tier-0 recording (if recorded)
* `<run>_summary.csv` : max error summary vs reference replay
"""

using Dates
using Printf
using Random

using PX4Lockstep
using PX4Lockstep.Sim
using PX4Lockstep.Sim.RecordReplay

const V = Sim.Verification

# Keep Iris defaults centralized.
include(joinpath(@__DIR__, "iris_common.jl"))


################################################################################
# Helpers
################################################################################

_getenv_str(name::AbstractString, default::AbstractString) = get(ENV, name, default)

function _getenv_float(name::AbstractString, default::Float64)
    v = get(ENV, name, nothing)
    v === nothing && return default
    return parse(Float64, v)
end

function _getenv_int(name::AbstractString, default::Int)
    v = get(ENV, name, nothing)
    v === nothing && return default
    return parse(Int, v)
end

function _rms(v::AbstractVector{Float64})
    s = 0.0
    @inbounds for x in v
        s += x * x
    end
    return sqrt(s / length(v))
end

function _battery_error(ref::Vector{Sim.Powertrain.BatteryStatus}, sol::Vector{Sim.Powertrain.BatteryStatus})
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

function _default_run_name()
    return "iris_" * Dates.format(Dates.now(), "yyyymmdd_HHMMSS")
end


################################################################################
# Record (PX4 live)
################################################################################

function _record_tier0!(;
    out_path::AbstractString,
    mission_path::AbstractString,
    lib_path::AbstractString,
    t_end_s::Float64,
    dt_ap_s::Float64,
    dt_wind_s::Float64,
    dt_log_s::Float64,
    seed::Int,
)
    println("[iris_integrator_compare] RECORD (PX4 live)")

    px4_cfg = PX4Lockstep.LockstepConfig(
        dataman_use_ram = 1,
        enable_commander = 0,
        enable_control_allocator = 1,
    )

    ap = Sim.Autopilots.init!(
        config = px4_cfg,
        libpath = (isempty(lib_path) ? nothing : lib_path),
        home = DEFAULT_HOME,
        edge_trigger = false,
    )

    try
        Sim.Autopilots.load_mission!(ap, mission_path)

        env = iris_env_live()
        vehicle = iris_vehicle_defaults()
        battery = iris_battery_defaults()
        scenario = iris_scenario_defaults()
        estimator = iris_estimator_defaults(dt_ap_s)
        contact = Sim.Contacts.FlatGroundContact()

        dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

        plant0 = Sim.Plant.init_plant_state(
            vehicle.state,
            vehicle.motor_actuators,
            vehicle.servo_actuators,
            vehicle.propulsion,
            battery,
        )

        # Sources
        scenario_src = RecordReplay.LiveScenarioSource(scenario; env = env, vehicle = vehicle, battery = battery)
        wind_src = RecordReplay.LiveWindSource(env.wind, Random.Xoshiro(seed), dt_wind_s)
        ap_src = RecordReplay.LiveAutopilotSource(ap)
        est_src = RecordReplay.LiveEstimatorSource(estimator, Random.Xoshiro(seed + 1), dt_ap_s)

        # Timeline
        t0_us = UInt64(0)
        t_end_us = RecordReplay.dt_to_us(t_end_s)
        timeline = RecordReplay.build_timeline_for_run(
            t0_us,
            t_end_us;
            dt_ap_us = RecordReplay.dt_to_us(dt_ap_s),
            dt_wind_us = RecordReplay.dt_to_us(dt_wind_s),
            dt_log_us = RecordReplay.dt_to_us(dt_log_s),
            scenario = scenario_src,
        )

        # Use the same integrator for recording and reference replay by default.
        integrator = iris_reference_integrator()
        recorder = RecordReplay.InMemoryRecorder()
        sim = RecordReplay.plant_record_engine(
            timeline = timeline,
            plant0 = plant0,
            dynfun = dynfun,
            integrator = integrator,
            autopilot = ap_src,
            wind = wind_src,
            scenario = scenario_src,
            estimator = est_src,
            recorder = recorder,
        )

        RecordReplay.run!(sim)

        recording = RecordReplay.Tier0Recording(
            timeline = timeline,
            plant0 = plant0,
            recorder = recorder,
            meta = Dict(
                :mission_path => mission_path,
                :seed => seed,
                :t_end_s => t_end_s,
                :dt_autopilot_s => dt_ap_s,
                :dt_wind_s => dt_wind_s,
                :dt_log_s => dt_log_s,
                :home => DEFAULT_HOME,
                :record_solver => "RK45_ref",
            ),
        )

        RecordReplay.save_recording(out_path, recording)
        println("[iris_integrator_compare] Saved recording → $(out_path)")
        return recording
    finally
        Sim.Autopilots.close!(ap)
    end
end


################################################################################
# Replay sweep (plant-only)
################################################################################

function _replay_sweep(; recording::RecordReplay.Tier0Recording, solvers::Vector{String})
    # Build traces.
    tr = RecordReplay.tier0_traces(recording)
    scenario_src = try
        scn = RecordReplay.scenario_traces(recording)
        RecordReplay.ReplayScenarioSource(scn.ap_cmd, scn.landed, scn.faults)
    catch
        RecordReplay.NullScenarioSource()
    end

    # Reconstruct dynfun parameters (NoWind env; wind comes from trace).
    home = get(recording.meta, :home, DEFAULT_HOME)
    env = iris_env_replay(home = home)
    vehicle = iris_vehicle_defaults()
    battery = iris_battery_defaults()
    contact = Sim.Contacts.FlatGroundContact()
    dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

    # Reference replay.
    ref_int = iris_reference_integrator()
    ref_rec = RecordReplay.InMemoryRecorder()
    ref_sim = RecordReplay.plant_replay_engine(
        timeline = recording.timeline,
        plant0 = recording.plant0,
        dynfun = dynfun,
        integrator = ref_int,
        cmd_trace = tr.cmd,
        wind_trace = tr.wind_ned,
        scenario = scenario_src,
        recorder = ref_rec,
    )
    RecordReplay.run!(ref_sim)

    log_axis = recording.timeline.log.t_us
    dt_log_s = length(log_axis) >= 2 ? Float64(log_axis[2] - log_axis[1]) * 1e-6 : 0.0
    ref_traj = V.Trajectory(dt_log_s, ref_rec.values[:plant])
    ref_batt = ref_rec.values[:battery]

    rows = Vector{NamedTuple}()

    for name in solvers
        int = iris_test_integrator(name)

        t0 = time_ns()
        rec = RecordReplay.InMemoryRecorder()
        sim = RecordReplay.plant_replay_engine(
            timeline = recording.timeline,
            plant0 = recording.plant0,
            dynfun = dynfun,
            integrator = int,
            cmd_trace = tr.cmd,
            wind_trace = tr.wind_ned,
            scenario = scenario_src,
            recorder = rec,
        )
        RecordReplay.run!(sim)
        wall_s = (time_ns() - t0) * 1e-9

        trj = V.Trajectory(dt_log_s, rec.values[:plant])
        err = V.error_series(ref_traj, trj)
        berr = _battery_error(ref_batt, rec.values[:battery])

        row = (
            solver = name,
            wall_s = wall_s,
            pos_max_m = err.max.pos,
            vel_max_mps = err.max.vel,
            att_max_rad = err.max.att_rad,
            bodyrate_max_rad_s = err.max.ω,
            rotor_omega_max_rad_s = err.max.rotor,
            battV_max_v = berr.max_v,
            battI_max_a = berr.max_i,
        )
        push!(rows, row)
    end
    return rows
end


################################################################################
# Main
################################################################################

function main()
    out_dir = _getenv_str("IRIS_OUT_DIR", joinpath(@__DIR__, "out"))
    mkpath(out_dir)

    run_name = _getenv_str("IRIS_RUN_NAME", _default_run_name())
    rec_in = get(ENV, "IRIS_RECORD_IN", "")
    rec_out = _getenv_str("IRIS_RECORD_OUT", joinpath(out_dir, "$(run_name)_tier0.jls"))
    summary_csv = _getenv_str("IRIS_OUT_CSV", joinpath(out_dir, "$(run_name)_summary.csv"))

    t_end_s = _getenv_float("IRIS_T_END_S", 20.0)
    dt_ap_s = _getenv_float("IRIS_DT_AUTOPILOT_S", 0.004)
    dt_wind_s = _getenv_float("IRIS_DT_WIND_S", 0.002)
    dt_log_s = _getenv_float("IRIS_DT_LOG_S", 0.01)
    seed = _getenv_int("IRIS_SEED", 1)

    # Candidate solvers.
    solvers = split(_getenv_str("IRIS_SWEEP_SOLVERS", "RK4,RK23,RK45"), ',')
    solvers = [String(strip(s)) for s in solvers if !isempty(strip(s))]

    println("Iris integrator compare (record + replay)")
    println("  t_end=$(t_end_s)s, dt_ap=$(dt_ap_s)s, dt_wind=$(dt_wind_s)s, dt_log=$(dt_log_s)s")
    println("  solvers: ", join(solvers, ", "))
    println("  out_dir: $(out_dir)")

    recording = if !isempty(rec_in)
        println("[iris_integrator_compare] Using existing recording: $(rec_in)")
        RecordReplay.load_recording(rec_in)
    else
        mission_path = get(ENV, "PX4_LOCKSTEP_MISSION", "")
        isempty(mission_path) && error("Set PX4_LOCKSTEP_MISSION (or set IRIS_RECORD_IN to skip recording)")
        lib_path = get(ENV, PX4Lockstep.LIB_ENV, "")
        _record_tier0!(
            out_path = rec_out,
            mission_path = mission_path,
            lib_path = lib_path,
            t_end_s = t_end_s,
            dt_ap_s = dt_ap_s,
            dt_wind_s = dt_wind_s,
            dt_log_s = dt_log_s,
            seed = seed,
        )
    end

    println("[iris_integrator_compare] REPLAY (plant-only sweep)")
    rows = _replay_sweep(recording = recording, solvers = solvers)

    # Print summary table.
    println()
    println("Summary (max error vs RK45 reference replay)")
    @printf("%-6s  %7s  %10s  %10s  %10s  %11s  %12s  %9s  %9s\n",
        "solver", "wall_s", "pos_max", "vel_max", "att_max", "ω_max", "rotor_max", "|ΔV|", "|ΔI|")
    for r in rows
        @printf(
            "%-6s  %7.3f  %10.3g  %10.3g  %10.3g  %11.3g  %12.3g  %9.3g  %9.3g\n",
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

    # Write CSV.
    open(summary_csv, "w") do io
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
    println()
    println("Wrote summary CSV → $(summary_csv)")
    if isempty(rec_in)
        println("Recording → $(rec_out)")
    end
    return nothing
end

main()
