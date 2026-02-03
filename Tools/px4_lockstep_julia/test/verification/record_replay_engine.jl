using Test
using Random
using StaticArrays

const Sim = PX4Lockstep.Sim
const RT = Sim.Runtime
const REC = Sim.Recording


@testset "Runtime.Engine: replay integration matches analytic" begin
    # 0.1s run, autopilot is the densest axis so evt == ap (piecewise-constant inputs).
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
    )

    # Command: 0 for t < 0.05, 1 for t >= 0.05.
    cmd_data = Vector{Sim.Vehicles.ActuatorCommand}(undef, length(timeline.ap.t_us))
    for (i, t_us) in pairs(timeline.ap.t_us)
        a = (t_us < UInt64(50_000)) ? 0.0 : 1.0
        motors = SVector{12,Float64}(ntuple(j -> j == 1 ? a : 0.0, 12))
        cmd_data[i] = Sim.Vehicles.ActuatorCommand(motors = motors)
    end
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK4Integrator()
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)

    # Analytic: acceleration 0 for 0.05s, then 1 for 0.05s.
    @test isapprox(sim.plant.vel_ned[1], 0.05; atol = 1e-12)
    @test isapprox(sim.plant.pos_ned[1], 0.5 * (0.05^2); atol = 1e-12)
end


@testset "Runtime.Engine: record mode captures axis-aligned traces" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
    )

    cmd_data = Vector{Sim.Vehicles.ActuatorCommand}(undef, length(timeline.ap.t_us))
    for (i, t_us) in pairs(timeline.ap.t_us)
        a = (t_us < UInt64(50_000)) ? 0.0 : 1.0
        motors = SVector{12,Float64}(ntuple(j -> j == 1 ? a : 0.0, 12))
        cmd_data[i] = Sim.Vehicles.ActuatorCommand(motors = motors)
    end
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    rec = REC.InMemoryRecorder()
    integ = Sim.Integrators.EulerIntegrator()

    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    sim = RT.plant_record_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
        recorder = rec,
    )
    RT.run!(sim)

    traces = REC.tier0_traces(rec, timeline)

    @test traces.cmd.axis.t_us == timeline.ap.t_us
    @test traces.wind_base_ned.axis.t_us == timeline.wind.t_us
    @test traces.plant.axis.t_us == timeline.log.t_us

    @test traces.cmd.data == cmd_tr.data
    @test traces.wind_base_ned.data == wind_tr.data
end


@testset "Runtime.Engine: record_estimator captures estimator stream" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(20_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
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

    rec = REC.InMemoryRecorder()
    Sim.simulate(
        mode = :record,
        timeline = timeline,
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        recorder = rec,
        record_estimator = true,
    )

    traces = REC.estimator_traces(rec, timeline)
    @test traces.est.axis.t_us == timeline.ap.t_us
    @test traces.est isa REC.ZOHTrace
    @test traces.est.data[1].pos_ned == x0.pos_ned
    @test traces.est.data[end].pos_ned == x0.pos_ned
end


@testset "Runtime.Engine: strict_cmd validates even without sanitize" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_src = PX4Lockstep.Tests.Fixtures.ConstantMotorAutopilotSource(cmd)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    scenario = Sim.Sources.NullScenarioSource()
    estimator = Sim.Sources.NullEstimatorSource()

    x0 = Sim.RigidBody.RigidBodyState()

    sim = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true, sanitize_cmd = false);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario,
        estimator = estimator,
    )

    @test_throws ErrorException RT.run!(sim)
end


@testset "Runtime.Engine: sanitize replaces NaN actuator commands" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(NaN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_src = PX4Lockstep.Tests.Fixtures.ConstantMotorAutopilotSource(cmd)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    scenario = Sim.Sources.NullScenarioSource()
    estimator = Sim.Sources.NullEstimatorSource()

    x0 = Sim.RigidBody.RigidBodyState()
    rec = REC.InMemoryRecorder()

    sim = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true, sanitize_cmd = true);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = PX4Lockstep.Tests.Fixtures.CmdAccelX(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario,
        estimator = estimator,
        recorder = rec,
    )

    RT.run!(sim)
    REC.finalize!(rec)

    traces = REC.tier0_traces(rec, timeline)
    @test traces.cmd.data[1].motors[1] == 0.0
end


@testset "Runtime.Engine: DirectActuators snap at autopilot ticks" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(ntuple(i -> i <= 4 ? 0.5 : 0.0, 12))
    cmd_data = [Sim.Vehicles.ActuatorCommand(motors = motors) for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    env = Sim.Environment.EnvironmentModel(wind = Sim.Environment.NoWind())
    model = PX4Lockstep.Tests.Fixtures.iris_vehicle_for_tests().model
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()
    propulsion = Sim.Propulsion.default_multirotor_set()
    battery = Sim.Aircraft.build_battery(
        Sim.Aircraft.BatterySpec(model = :ideal, voltage_v = 12.0),
    )

    rb0 = Sim.RigidBody.RigidBodyState()
    plant0 = Sim.Plant.init_plant_state(rb0, motor_act, servo_act, propulsion, battery)

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        model,
        env,
        Sim.Contacts.NoContact(),
        motor_act,
        servo_act,
        propulsion,
        battery,
    )

    integ = Sim.Integrators.RK4Integrator()
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)

    @test sim.plant.rotor_ω[1] > 0.0
end



@testset "Record/replay equivalence at log ticks (Tier0)" begin
    res = PX4Lockstep.Tests.Fixtures.record_replay_equivalence_log_ticks()
    @test res.count > 0
    @test res.max_pos <= 1e-12
    @test res.max_vel <= 1e-12
    @test res.max_att <= 1e-12
    @test res.max_ω <= 1e-12
    @test res.max_rotor <= 1e-12
    @test res.max_soc <= 1e-15
    @test res.max_v1 <= 1e-12
end

@testset "Record/replay equivalence with wind disturbance (Tier0)" begin
    res = PX4Lockstep.Tests.Fixtures.record_replay_equivalence_wind_disturbance()
    @test res.count > 0
    @test res.max_pos <= 1e-12
    @test res.max_vel <= 1e-12
    @test res.max_q <= 1e-12
    @test res.max_ω <= 1e-12
end


@testset "Stage ordering: scenario -> wind -> derived outputs -> autopilot" begin
    # The canonical engine should guarantee that a scenario mutation at a boundary
    # is visible to the wind source at the same boundary, and that derived outputs
    # are computed before the autopilot runs.

    # Scenario: disconnect the battery and inject a step gust at t=0.
    mutable struct OrderScenario{W}
        wind::W
        dv_ned::Sim.Types.Vec3
        duration_s::Float64
        fired::Bool
    end

    function RT.update!(src::OrderScenario, bus::RT.SimBus, plant_state, t_us::UInt64)
        if !src.fired && t_us == 0
            src.fired = true
            bus.faults = Sim.Faults.set_battery_connected(bus.faults, false)
            Sim.Environment.add_step_gust!(src.wind, src.dv_ned, t_us, src.duration_s)
        end
        return nothing
    end

    mutable struct CaptureAutopilot
        seen_t_us::Vector{UInt64}
        seen_wind::Vector{Sim.Types.Vec3}
        seen_batt_connected::Vector{Bool}
    end

    function RT.update!(src::CaptureAutopilot, bus::RT.SimBus, plant, t_us::UInt64)
        push!(src.seen_t_us, t_us)
        push!(src.seen_wind, bus.wind_ned)
        push!(src.seen_batt_connected, bus.batteries[1].connected)
        bus.cmd = Sim.Vehicles.ActuatorCommand() # don't care
        return nothing
    end

    veh = PX4Lockstep.Tests.Fixtures.iris_vehicle_for_tests()
    batt = PX4Lockstep.Tests.Fixtures.iris_battery_for_tests()
    env0 = PX4Lockstep.Tests.Fixtures.iris_env_replay_for_tests()
    wind_model = Sim.Environment.OUWind(
        mean = Sim.Types.vec3(0.0, 0.0, 0.0),
        σ = Sim.Types.vec3(0.0, 0.0, 0.0), # deterministic
        τ_s = 1.0,
    )
    env = Sim.Environment.EnvironmentModel(
        atmosphere = env0.atmosphere,
        wind = wind_model,
        gravity = env0.gravity,
        origin = env0.origin,
    )

    contact = Sim.Contacts.NoContact()
    model = Sim.PlantModels.CoupledMultirotorModel(
        veh.model,
        env,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )
    rb0 = Sim.RigidBody.RigidBodyState()
    plant0 = Sim.Plant.init_plant_state(
        rb0,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )

    tl = RT.build_timeline(UInt64(0), UInt64(10_000);
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    scenario = OrderScenario(wind_model, Sim.Types.vec3(3.0, 0.0, 0.0), 1.0, false)
    wind_src = Sim.Sources.LiveWindSource(wind_model, Random.Xoshiro(1), 0.01)
    autopilot = CaptureAutopilot(UInt64[], Sim.Types.Vec3[], Bool[])

    eng = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_LIVE, enable_derived_outputs = true);
        timeline = tl,
        plant0 = plant0,
        dynfun = model,
        integrator = Sim.Integrators.RK4Integrator(),
        scenario = scenario,
        wind = wind_src,
        autopilot = autopilot,
        estimator = Sim.Sources.NullEstimatorSource(),
        telemetry = RT.NullTelemetry(),
    )

    RT.run!(eng)

    # Autopilot should have run at t=0 and should observe the scenario+wind effects.
    @test autopilot.seen_t_us[1] == 0
    @test autopilot.seen_wind[1] == Sim.Types.vec3(3.0, 0.0, 0.0)
    @test autopilot.seen_batt_connected[1] == false
end

@testset "Recording.read_recording enforces schema version" begin
    timeline = RT.build_timeline(UInt64(0), UInt64(1_000);
        dt_ap_us = UInt64(1_000),
        dt_wind_us = UInt64(1_000),
        dt_log_us = UInt64(1_000),
    )
    rec = REC.Tier0Recording(
        bus_schema_version = RT.BUS_SCHEMA_VERSION - 1,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        recorder = REC.InMemoryRecorder(),
    )
    path = joinpath(mktempdir(), "bad_recording.jls")
    REC.write_recording(path, rec)
    @test_throws ErrorException REC.read_recording(path)
end