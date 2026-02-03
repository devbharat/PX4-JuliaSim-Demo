using Test
using StaticArrays

const Sim = PX4Lockstep.Sim
const RT = Sim.Runtime
const REC = Sim.Recording
const T = Sim.Types

function _battery_equal(a::Sim.Powertrain.BatteryStatus, b::Sim.Powertrain.BatteryStatus)
    @test a.connected == b.connected
    @test isapprox(a.voltage_v, b.voltage_v; atol = 1e-9, rtol = 0)
    @test isapprox(a.current_a, b.current_a; atol = 1e-9, rtol = 0)
    @test isapprox(a.temperature_c, b.temperature_c; atol = 1e-9, rtol = 0)
    @test isapprox(a.remaining, b.remaining; atol = 1e-9, rtol = 0)
    @test a.warning == b.warning
end

function _compare_stream(rec1, rec2, name::Symbol, axis::RT.TimeAxis, cmp)
    @test REC.stream_times(rec1, name) == axis.t_us
    @test REC.stream_times(rec2, name) == axis.t_us
    v1 = REC.stream_values(rec1, name)
    v2 = REC.stream_values(rec2, name)
    @test length(v1) == length(v2)
    for i in eachindex(v1, v2)
        cmp(v1[i], v2[i])
    end
end

function _build_drop_setup()
    env = PX4Lockstep.Tests.Fixtures.iris_env_replay_for_tests()
    contact = PX4Lockstep.Tests.Fixtures.iris_contact_for_tests()
    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = T.vec3(0.0, 0.0, -1.0),
        vel_ned = T.vec3(0.0, 0.0, 2.0),
        q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = T.vec3(0.0, 0.0, 0.0),
    )
    vehicle = PX4Lockstep.Tests.Fixtures.iris_vehicle_for_tests(x0 = rb0)
    battery = PX4Lockstep.Tests.Fixtures.iris_battery_for_tests()
    dynfun = PX4Lockstep.Tests.Fixtures.iris_dynfun_for_tests(env, vehicle, battery; contact = contact)
    plant0 = Sim.Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )
    return (dynfun = dynfun, plant0 = plant0)
end

@testset "Record/replay bus diagnostics (landed/impact/battery)" begin
    timeline = PX4Lockstep.Tests.Fixtures.iris_timeline_for_tests(
        t_end_s = 0.4,
        dt_autopilot_s = 0.01,
        dt_wind_s = 0.02,
        dt_log_s = 0.05,
        dt_phys_s = 0.005,
        scenario_source = Sim.Sources.NullScenarioSource(),
    )

    cmd = Sim.Vehicles.ActuatorCommand()
    cmd_data = fill(cmd, length(timeline.ap.t_us))
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [T.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    rec1 = REC.InMemoryRecorder()
    setup1 = _build_drop_setup()
    sim1 = RT.plant_record_engine(
        timeline = timeline,
        plant0 = setup1.plant0,
        dynfun = setup1.dynfun,
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        recorder = rec1,
    )
    RT.run!(sim1)

    # Ensure the run produced at least one impact to exercise the diagnostics path.
    impact_counts = REC.stream_values(rec1, :impact_count)
    @test any(x -> x > 0, impact_counts)

    traces = REC.tier0_traces(rec1, timeline)
    scn = REC.scenario_traces(rec1, timeline)

    ap_replay = Sim.Sources.ReplayAutopilotSource(traces.cmd)
    wind_replay = Sim.Sources.ReplayWindSource(traces.wind_base_ned)
    wind_dist = hasproperty(scn, :wind_dist) ? scn.wind_dist : nothing
    scenario_replay = Sim.Sources.ReplayScenarioSource(
        scn.ap_cmd,
        scn.landed,
        scn.faults;
        wind_dist = wind_dist,
    )

    rec2 = REC.InMemoryRecorder()
    setup2 = _build_drop_setup()
    sim2 = RT.plant_record_engine(
        timeline = timeline,
        plant0 = setup2.plant0,
        dynfun = setup2.dynfun,
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_replay,
        wind = wind_replay,
        scenario = scenario_replay,
        estimator = Sim.Sources.NullEstimatorSource(),
        recorder = rec2,
    )
    RT.run!(sim2)

    axis = timeline.log

    _compare_stream(rec1, rec2, :landed_phy, axis, (a, b) -> (@test a == b))
    _compare_stream(rec1, rec2, :impact_count, axis, (a, b) -> (@test a == b))
    _compare_stream(rec1, rec2, :impact_time_us, axis, (a, b) -> (@test a == b))
    _compare_stream(
        rec1,
        rec2,
        :impact_dv_ned,
        axis,
        (a, b) -> (@test isapprox(a, b; atol = 1e-9, rtol = 0)),
    )
    _compare_stream(
        rec1,
        rec2,
        :impact_accel_est_ned,
        axis,
        (a, b) -> (@test isapprox(a, b; atol = 1e-9, rtol = 0)),
    )
    _compare_stream(rec1, rec2, :battery, axis, (a, b) -> _battery_equal(a, b))
    _compare_stream(
        rec1,
        rec2,
        :batteries,
        axis,
        (a, b) -> begin
            @test length(a) == length(b)
            for i in eachindex(a, b)
                _battery_equal(a[i], b[i])
            end
        end,
    )
end