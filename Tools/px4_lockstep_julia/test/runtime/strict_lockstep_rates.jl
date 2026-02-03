
@testset "Simulation strict_lockstep_rates errors on autopilot rate mismatch" begin
    # Minimal autopilot that advertises a fast internal loop rate.
    Base.@kwdef mutable struct RateOutputs
        actuator_motors::NTuple{12,Float32} = ntuple(_ -> 0f0, 12)
        actuator_servos::NTuple{8,Float32} = ntuple(_ -> 0f0, 8)
        trajectory_setpoint_position::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_velocity::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_acceleration::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_yaw::Float32 = 0f0
        trajectory_setpoint_yawspeed::Float32 = 0f0
        nav_state::Int32 = Int32(0)
        arming_state::Int32 = Int32(0)
        mission_seq::Int32 = Int32(0)
        mission_count::Int32 = Int32(0)
        mission_finished::Int32 = Int32(0)
    end

    mutable struct RateAutopilot <: Sim.Autopilots.AbstractAutopilot
        out::RateOutputs
    end
    Sim.Autopilots.autopilot_output_type(::RateAutopilot) = RateOutputs
    Sim.Autopilots.max_internal_rate_hz(::RateAutopilot) = 250

    function Sim.Autopilots.autopilot_step(
        ap::RateAutopilot,
        time_us::UInt64,
        pos::Sim.Types.Vec3,
        vel::Sim.Types.Vec3,
        q::Sim.Types.Quat,
        ω::Sim.Types.Vec3,
        cmd::Sim.Autopilots.AutopilotCommand;
        landed::Bool = false,
        battery::Sim.Powertrain.BatteryStatus = Sim.Powertrain.BatteryStatus(),
        batteries::Vector{Sim.Powertrain.BatteryStatus} = Sim.Powertrain.BatteryStatus[battery],
    )::RateOutputs
        return ap.out
    end

    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
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

    ap = RateAutopilot(RateOutputs())
    autopilot_src = Sim.Sources.LiveAutopilotSource(ap)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    # dt_ap=0.01 (100 Hz) is slower than required for a 250 Hz loop.
    @test_throws ArgumentError Sim.simulate(
        mode = :live,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        dynfun = PX4Lockstep.Tests.Fixtures.ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        strict_lockstep_rates = true,
    )

    # Opt-out should only warn (and succeed).
    sim = Sim.simulate(
        mode = :live,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        dynfun = PX4Lockstep.Tests.Fixtures.ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        strict_lockstep_rates = false,
    )
    @test sim.cfg.mode == RT.MODE_LIVE
end