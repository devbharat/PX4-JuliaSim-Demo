using Test
using StaticArrays
using PX4Lockstep

const SIM = PX4Lockstep.Sim
const RT = SIM.Runtime
const INTEG = SIM.Integrators
const SRC = SIM.Sources
const REC = SIM.Recording
const WORK = SIM.Workflows

"""A tiny deterministic rigid-body model: X-acceleration equals motor[1] command."""
struct CmdAccelX end

function (f::CmdAccelX)(t, x::SIM.RigidBody.RigidBodyState, u::SIM.Plant.PlantInput)
    a = u.cmd.motors[1]
    pos_dot = SIM.Types.vec3(x.vel_ned[1], 0.0, 0.0)
    vel_dot = SIM.Types.vec3(a, 0.0, 0.0)
    q_dot = SIM.Types.Quat(0.0, 0.0, 0.0, 0.0)
    ω_dot = SIM.Types.vec3(0.0, 0.0, 0.0)
    return SIM.RigidBody.RigidBodyDeriv(
        pos_dot = pos_dot,
        vel_dot = vel_dot,
        q_dot = q_dot,
        ω_dot = ω_dot,
    )
end

@testset "Workflows.compare_integrators_recording" begin
    # Timeline: short + cheap, but enough to show Euler vs RK4.
    t_end_us = UInt64(200_000) # 0.2s
    timeline = RT.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(10_000),
        dt_phys_us = UInt64(10_000),
    )

    # Deterministic sources (replay sources) for a record run.
    half_us = t_end_us ÷ 2
    cmds = [
        SIM.Vehicles.ActuatorCommand(
            motors = SVector{12,Float64}(ntuple(_ -> (t >= half_us ? 1.0 : 0.0), 12)),
            servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8)),
        )
        for t in timeline.ap.t_us
    ]
    winds = [SIM.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]

    cmd_trace = REC.ZOHTrace(timeline.ap, cmds)
    wind_trace = REC.SampleHoldTrace(timeline.wind, winds)

    ap_src = SRC.ReplayAutopilotSource(cmd_trace)
    wind_src = SRC.ReplayWindSource(wind_trace)

    # Plant initial condition.
    x0 = SIM.RigidBody.RigidBodyState(
        pos_ned = SIM.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = SIM.Types.vec3(0.0, 0.0, 0.0),
        q_bn = SIM.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = SIM.Types.vec3(0.0, 0.0, 0.0),
    )

    # Record: captures the Tier-0 streams (cmd/wind/plant/battery + scenario defaults).
    rec = REC.InMemoryRecorder()
    rec_sim = RT.plant_record_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = INTEG.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = SRC.NullScenarioSource(),
        estimator = SRC.NullEstimatorSource(),
        telemetry = RT.NullTelemetry(),
        recorder = rec,
    )
    RT.run!(rec_sim)

    recording = REC.Tier0Recording(timeline = timeline, plant0 = x0, recorder = rec)

    solvers = [
        "euler" => INTEG.EulerIntegrator(),
        "rk4" => INTEG.RK4Integrator(),
    ]

    out1 = mktempdir()
    out_csv1 = joinpath(out1, "summary.csv")
    rows1 = WORK.compare_integrators_recording(
        recording = recording,
        dynfun = CmdAccelX(),
        solvers = solvers,
        reference_integrator = INTEG.RK4Integrator(),
        out_csv = out_csv1,
        print_table = false,
    )

    @test length(rows1) == 2
    @test rows1[1].solver == "euler"
    @test rows1[2].solver == "rk4"
    @test isfinite(rows1[1].pos_max_m)
    @test isfinite(rows1[2].pos_max_m)

    # Euler should be (much) worse than RK4 on position for this piecewise-constant acceleration.
    @test rows1[1].pos_max_m > rows1[2].pos_max_m

    # Determinism sanity (ignore wall_time_ms, which depends on machine load).
    out2 = mktempdir()
    out_csv2 = joinpath(out2, "summary.csv")
    rows2 = WORK.compare_integrators_recording(
        recording = recording,
        dynfun = CmdAccelX(),
        solvers = solvers,
        reference_integrator = INTEG.RK4Integrator(),
        out_csv = out_csv2,
        print_table = false,
    )

    @test rows2[1].solver == rows1[1].solver
    @test rows2[2].solver == rows1[2].solver
    @test isapprox(rows2[1].pos_max_m, rows1[1].pos_max_m; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[2].pos_max_m, rows1[2].pos_max_m; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[1].vel_max_mps, rows1[1].vel_max_mps; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[2].vel_max_mps, rows1[2].vel_max_mps; atol = 1e-12, rtol = 0.0)
end
