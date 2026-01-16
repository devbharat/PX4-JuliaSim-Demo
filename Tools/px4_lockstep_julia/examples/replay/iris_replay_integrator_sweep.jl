#!/usr/bin/env julia
"""Replay an Iris Tier-0 recording and sweep plant integrators.

This is the **plant-only** half of the recommended workflow:

1) Record a closed-loop PX4 run using `examples/replay/iris_record_run.jl`.
2) Replay the plant open-loop using the recorded Tier-0 streams:
   * `cmd(t)` (actuator commands) on `timeline.ap`
   * `wind_ned(t)` on `timeline.wind`
   * scenario outputs (`faults`, `ap_cmd`, `landed`) on `timeline.scn` / `timeline.evt`
3) Compare candidate integrators against a high-accuracy reference replay.

PX4 is **not** involved in replay: only the plant is integrated.

Environment variables
---------------------
Required:
* `IRIS_RECORD_IN` : path to a `.jls` Tier-0 recording file

Optional:
* `IRIS_SWEEP_SOLVERS` : comma-separated list (default: "RK4,RK23,RK45")
* `IRIS_OUT_CSV` : write a summary CSV (default: none)

Notes
-----
* This reconstructs the Iris dynfun from shared defaults (`iris_common.jl`).
  If you change vehicle/battery/environment parameters, keep record + replay
  scripts consistent (or start storing richer metadata in the recording).
"""

using Printf

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


################################################################################
# Main
################################################################################

in_path = get(ENV, "IRIS_RECORD_IN", "")
isempty(in_path) && error("Set IRIS_RECORD_IN to a Tier-0 recording (.jls)")

recording = RecordReplay.load_recording(in_path)
tr = RecordReplay.tier0_traces(recording)

# Optional: replay scenario outputs (especially bus.faults) so plant-affecting
# faults are applied identically across integrator sweeps.
scenario_src = try
    scn = RecordReplay.scenario_traces(recording)
    RecordReplay.ReplayScenarioSource(scn.ap_cmd, scn.landed, scn.faults)
catch
    RecordReplay.NullScenarioSource()
end

# Reconstruct dynfun parameters. Wind comes from the trace (NoWind env here).
home = get(recording.meta, :home, DEFAULT_HOME)
env = iris_env_replay(home = home)
vehicle = iris_vehicle_defaults()
battery = iris_battery_defaults()
contact = Sim.Contacts.FlatGroundContact()
dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

# Reference replay
ref_int = iris_reference_integrator()
println("Running reference replay: RK45 (tight, plant-aware) ...")
ref_rec = RecordReplay.InMemoryRecorder()
ref_sim = RecordReplay.plant_replay_engine(
    timeline = recording.timeline,
    plant0 = recording.plant0,
    dynfun = dynfun,
    integrator = ref_int,
    cmd_trace = tr.cmd,
    wind_trace = tr.wind,
    scenario = scenario_src,
    recorder = ref_rec,
)
RecordReplay.run!(ref_sim)

dt_log_s = Float64(recording.timeline.log.dt_us) * 1e-6
ref_traj = V.Trajectory(dt_log_s, ref_rec.values[:plant])
ref_batt = ref_rec.values[:battery]

solvers = split(_getenv_str("IRIS_SWEEP_SOLVERS", "RK4,RK23,RK45"), ',')
solvers = [strip(s) for s in solvers if !isempty(strip(s))]

println("Sweeping solvers: ", join(solvers, ", "))

rows = Vector{NamedTuple}()

for s in solvers
    name = strip(s)
    int = iris_test_integrator(name)
    print("  replay ", rpad(name, 6), " ... ")

    rec = RecordReplay.InMemoryRecorder()
    sim = RecordReplay.plant_replay_engine(
        timeline = recording.timeline,
        plant0 = recording.plant0,
        dynfun = dynfun,
        integrator = int,
        cmd_trace = tr.cmd,
        wind_trace = tr.wind,
        scenario = scenario_src,
        recorder = rec,
    )
    RecordReplay.run!(sim)

    trj = V.Trajectory(dt_log_s, rec.values[:plant])
    err = V.error_series(ref_traj, trj)
    berr = _battery_error(ref_batt, rec.values[:battery])

    row = (
        solver = name,
        pos_max_m = err.max.pos,
        vel_max_mps = err.max.vel,
        att_max_rad = err.max.att_rad,
        bodyrate_max_rad_s = err.max.ω,
        rotor_omega_max_rad_s = err.max.rotor,
        battV_max_v = berr.max_v,
        battI_max_a = berr.max_i,
    )
    push!(rows, row)

    @printf(
        "pos=%.3g m  vel=%.3g m/s  att=%.3g rad  ω=%.3g rad/s  rotor=%.3g rad/s  |ΔV|=%.3g V  |ΔI|=%.3g A\n",
        row.pos_max_m,
        row.vel_max_mps,
        row.att_max_rad,
        row.bodyrate_max_rad_s,
        row.rotor_omega_max_rad_s,
        row.battV_max_v,
        row.battI_max_a,
    )
end

# Optional CSV summary
out_csv = get(ENV, "IRIS_OUT_CSV", "")
if !isempty(out_csv)
    open(out_csv, "w") do io
        println(
            io,
            "solver,pos_max_m,vel_max_mps,att_max_rad,bodyrate_max_rad_s,rotor_omega_max_rad_s,battV_max_v,battI_max_a",
        )
        for r in rows
            println(
                io,
                "$(r.solver),$(r.pos_max_m),$(r.vel_max_mps),$(r.att_max_rad),$(r.bodyrate_max_rad_s),$(r.rotor_omega_max_rad_s),$(r.battV_max_v),$(r.battI_max_a)",
            )
        end
    end
    println("Wrote summary CSV → $(out_csv)")
end
