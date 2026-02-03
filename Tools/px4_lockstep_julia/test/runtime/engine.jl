
@testset "Runtime.Engine time_us is exact and log samples pre-step state" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(2_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1.0, 2.0, 3.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    log = Sim.Logging.SimLog()
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        log_sinks = log,
    )
    RT.run!(sim)

    # Log is pre-step state at t=0.
    @test length(log.t) >= 1
    @test log.t[1] == 0.0
    @test log.time_us[1] == UInt64(0)
    @test log.pos_ned[1] == (1.0, 2.0, 3.0)
end

@testset "Runtime.Engine respects t_end (no overshoot)" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(5_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(2_000),
        dt_wind_us = UInt64(2_000),
        dt_log_us = UInt64(2_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK45Integrator(h_min = 1e-6, h_max = 0.01)
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.ZeroRB(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)
    @test sim.bus.time_us == t_end_us
end

@testset "Runtime.Engine: AtTime scenario boundaries are true event boundaries" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    # Minimal scenario source that declares a one-off event boundary at 5 ms and
    # toggles a motor disable fault at that boundary.
    mutable struct MarkScenario
        times::Vector{UInt64}
    end

    function Sim.Runtime.event_times_us(src::MarkScenario, t0_us::UInt64, t_end_us::UInt64)
        t_evt = UInt64(5_000)
        return (t_evt >= t0_us && t_evt <= t_end_us) ? UInt64[t0_us, t_evt] : UInt64[t0_us]
    end

    function Sim.Runtime.update!(src::MarkScenario, bus::RT.SimBus, plant_state, t_us::UInt64)
        push!(src.times, t_us)
        if t_us >= UInt64(5_000)
            bus.faults = Sim.Faults.disable_motor(bus.faults, 1)
        end
        return nothing
    end

    scenario = MarkScenario(UInt64[])
    t0_us = UInt64(0)
    t_end_us = UInt64(12_000)
    timeline = RT.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(20_000),
        scenario = scenario,
    )

    # The evt axis must contain the scenario boundary (5 ms) even though AP ticks are 0/10 ms.
    @test UInt64(5_000) in timeline.evt.t_us

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState()
    integ = Sim.Integrators.RK4Integrator()

    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = integ,
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = scenario,
    )

    evt = timeline.evt.t_us
    @test evt[1] == t0_us
    # Process the first boundary, step to the 5 ms event, and process it.
    RT.process_events_at!(sim)
    RT.step_to_next_event!(sim)
    RT.process_events_at!(sim)

    @test sim.t_us == UInt64(5_000)
    @test Sim.Faults.is_motor_disabled(sim.bus.faults, 1)
    @test UInt64(5_000) in scenario.times
end

@testset "Runtime.Engine holds wind constant between wind ticks" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    struct TimeWind <: Sim.Environment.AbstractWind end
    Sim.Environment.step_wind!(::TimeWind, ::Sim.Types.Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing
    Sim.Environment.sample_wind!(::TimeWind, ::Sim.Types.Vec3, ::Float64) = nothing
    Sim.Environment.wind_velocity(::TimeWind, ::Sim.Types.Vec3, t::Float64) =
        Sim.Types.vec3(t, 0.0, 0.0)

    env = Sim.Environment.EnvironmentModel(wind = TimeWind())

    t0_us = UInt64(0)
    t_end_us = UInt64(300_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(100_000),
        dt_wind_us = UInt64(200_000),
        dt_log_us = UInt64(100_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_src = Sim.Sources.LiveWindSource(env.wind, Random.Xoshiro(1), 0.2)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    log = Sim.Logging.SimLog()
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.ZeroRB(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        log_sinks = log,
    )
    RT.run!(sim)

    @test length(log.wind_ned) == length(timeline.log.t_us)
    @test log.wind_ned[1][1] == 0.0
    @test log.wind_ned[2][1] == 0.0
    @test isapprox(log.wind_ned[3][1], 0.2; atol = 1e-12)
    @test isapprox(log.wind_ned[4][1], 0.2; atol = 1e-12)
end