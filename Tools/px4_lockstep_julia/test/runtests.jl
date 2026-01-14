using Test
using Random
using StaticArrays
using PX4Lockstep

const Sim = PX4Lockstep.Sim

@testset "Scheduling.StepTrigger" begin
    trig = Sim.Scheduling.StepTrigger(5)
    @test Sim.Scheduling.due(trig, 0)
    @test !Sim.Scheduling.due(trig, 1)
    @test Sim.Scheduling.due(trig, 5)
    @test Sim.Scheduling.due(trig, 10)

    trig2 = Sim.Scheduling.StepTrigger(4; offset_steps = 2)
    @test !Sim.Scheduling.due(trig2, 0)
    @test !Sim.Scheduling.due(trig2, 1)
    @test Sim.Scheduling.due(trig2, 2)
    @test !Sim.Scheduling.due(trig2, 3)
    @test Sim.Scheduling.due(trig2, 6)
end

@testset "Noise.AR1 deterministic" begin
    rng = MersenneTwister(1234)
    a = Sim.Noise.AR1(1.0, 0.5)
    x1 = Sim.Noise.step!(a, rng, 0.01)
    rng2 = MersenneTwister(1234)
    a2 = Sim.Noise.AR1(1.0, 0.5)
    x2 = Sim.Noise.step!(a2, rng2, 0.01)
    @test x1 == x2
end

@testset "Environment.GustStep delegates stepping" begin
    # If mean wind is stateful (OU), stepping a GustStep wrapper should still advance it.
    rng = MersenneTwister(42)
    ou = Sim.Environment.OUWind(mean = Sim.Types.vec3(0, 0, 0), σ = Sim.Types.vec3(1, 0, 0), τ_s = 1.0)
    w = Sim.Environment.GustStep(ou, Sim.Types.vec3(0, 0, 0), 0.0, 1.0)
    pos = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Environment.step_wind!(w, pos, 0.0, 0.1, rng)
    @test ou.v_gust[1] != 0.0
end

@testset "Vehicles.FirstOrderActuators exact discretization" begin
    a = Sim.Vehicles.FirstOrderActuators{1}(τ = 0.1, y0 = SVector{1,Float64}(0.0))
    u = SVector{1,Float64}(1.0)
    y = Sim.Vehicles.step_actuators!(a, u, 0.05)
    @test isapprox(y[1], 1.0 - exp(-0.05 / 0.1); atol = 1e-12)
end

@testset "Simulation time_us is exact and log samples pre-step state" begin
    # Minimal dummy autopilot so we can run the engine without libpx4_lockstep.
    Base.@kwdef mutable struct DummyOutputs
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

    mutable struct DummyAutopilot <: Sim.Autopilots.AbstractAutopilot
        out::DummyOutputs
    end
    Sim.Autopilots.autopilot_output_type(::DummyAutopilot) = DummyOutputs

    function Sim.Autopilots.autopilot_step(
        ap::DummyAutopilot,
        time_us::UInt64,
        pos::Sim.Types.Vec3,
        vel::Sim.Types.Vec3,
        q::Sim.Types.Quat,
        ω::Sim.Types.Vec3,
        cmd::Sim.Autopilots.AutopilotCommand;
        landed::Bool = false,
        battery::Sim.Powertrain.BatteryStatus = Sim.Powertrain.BatteryStatus(),
    )::DummyOutputs
        # Put a recognizable pattern into motors when armed.
        m = cmd.armed ? 0.4f0 : 0f0
        ap.out = DummyOutputs(actuator_motors = ntuple(_ -> m, 12))
        return ap.out
    end

    # Build a small sim instance.
    env = Sim.Environment.EnvironmentModel(wind = Sim.Environment.ConstantWind(Sim.Types.vec3(0, 0, 0)))
    model = Sim.Vehicles.IrisQuadrotor()
    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1.0, 2.0, 3.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    hover_T = model.params.mass * 9.80665 / 4.0
    propulsion = Sim.Propulsion.default_iris_quadrotor_set(km_m = model.params.km, thrust_hover_per_rotor_n = hover_T)
    vehicle = Sim.Simulation.VehicleInstance(model, Sim.Vehicles.DirectActuators(), Sim.Vehicles.DirectActuators(), propulsion, x0)

    cfg = Sim.Simulation.SimulationConfig(dt = 0.002, t0 = 0.0, t_end = 0.01, dt_autopilot = 0.01, dt_log = 0.002, seed = 1)
    ap = DummyAutopilot(DummyOutputs())
    scenario = Sim.Scenario.ScriptedScenario(arm_time_s = 1e9, mission_time_s = 1e9)
    log = Sim.Logging.SimLog()

    sim = Sim.Simulation.SimulationInstance(
        cfg = cfg,
        env = env,
        vehicle = vehicle,
        autopilot = ap,
        estimator = Sim.Estimators.TruthEstimator(),
        integrator = Sim.Integrators.EulerIntegrator(),
        scenario = scenario,
        battery = Sim.Powertrain.IdealBattery(voltage_v = 12.0),
        log = log,
        contact = Sim.Contacts.NoContact(),
    )

    @test Sim.Simulation.time_us(sim) == 0
    Sim.Simulation.step!(sim)
    @test Sim.Simulation.time_us(sim) == 2000

    # Log is pre-step state at t=0.
    @test length(sim.log.t) == 1
    @test sim.log.t[1] == 0.0
    @test sim.log.pos_ned[1] == (1.0, 2.0, 3.0)
end

@testset "Logging.SimLog push" begin
    log = Sim.Logging.SimLog()
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1, 2, 3),
        vel_ned = Sim.Types.vec3(0.1, 0.2, 0.3),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    cmd = Sim.Vehicles.ActuatorCommand(
        motors = SVector{12,Float64}(fill(0.1, 12)),
        servos = SVector{8,Float64}(fill(0.0, 8)),
    )
    wind = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Logging.log!(log, 0.0, x, cmd; wind_ned = wind, rho = 1.2)
    @test length(log.t) == 1
    @test log.pos_ned[1] == (1.0, 2.0, 3.0)
end
