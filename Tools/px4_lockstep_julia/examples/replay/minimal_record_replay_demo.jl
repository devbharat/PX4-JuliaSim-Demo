"""Minimal record/replay demo (Option A).

This script is intentionally tiny and deterministic. It does **not** run PX4.

It demonstrates the core Option A loop:
- build a multi-axis `Timeline`
- define recorded `cmd(t)` and `wind(t)` traces
- run `Engine` in replay mode
- run again in record mode and rebuild traces from the recorder

Run
---
    julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/replay/minimal_record_replay_demo.jl
"""

using StaticArrays
using PX4Lockstep

const Sim = PX4Lockstep.Sim
const RT = Sim.Runtime
const Rec = Sim.Recording
const Log = Sim.Logging

struct CmdAccelX end

function (f::CmdAccelX)(t::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
    a = u.cmd.motors[1]
    return Sim.RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Sim.Types.vec3(a, 0.0, 0.0),
        q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
end


function main()
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
    )

    # Optional: structured log sink (CSV). This is independent of the Tier-0 recorder.
    out_dir = joinpath(@__DIR__, "out")
    mkpath(out_dir)
    csv_sink_replay = Log.CSVLogSink(joinpath(out_dir, "demo_log_replay.csv"))
    csv_sink_record = Log.CSVLogSink(joinpath(out_dir, "demo_log_record.csv"))

    # Command: 0 for t < 0.05, 1 for t >= 0.05.
    cmd_data = Vector{Sim.Vehicles.ActuatorCommand}(undef, length(timeline.ap.t_us))
    for (i, t_us) in pairs(timeline.ap.t_us)
        a = (t_us < UInt64(50_000)) ? 0.0 : 1.0
        motors = SVector{12,Float64}(ntuple(j -> j == 1 ? a : 0.0, 12))
        cmd_data[i] = Sim.Vehicles.ActuatorCommand(motors = motors)
    end
    cmd_tr = Rec.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = Rec.SampleHoldTrace(timeline.wind, wind_data)

    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    # Replay run.
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        log_sinks = csv_sink_replay,
    )
    RT.run!(sim)

    println("Replay final pos_x = ", sim.plant.pos_ned[1], "  vel_x = ", sim.plant.vel_ned[1])

    # Record run (using the same replay sources to demonstrate trace capture + rebuild).
    rec = Rec.InMemoryRecorder()
    sim2 = RT.plant_record_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = ap_src,
        wind = wind_src,
        recorder = rec,
        log_sinks = csv_sink_record,
    )
    RT.run!(sim2)

    tr2 = Rec.tier0_traces(rec, timeline)
    println("Recorded cmd samples: ", length(tr2.cmd.data), ", wind samples: ", length(tr2.wind_ned.data))
end

main()
