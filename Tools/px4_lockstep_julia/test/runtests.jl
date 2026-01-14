using Test
using Random
using StaticArrays
using LinearAlgebra
using PX4Lockstep

const Sim = PX4Lockstep.Sim

"""Return the geodesic rotation error (rad) between two quaternions.

Quaternions are treated as equivalent up to sign (q == -q).
"""
function quat_angle_error(q::Sim.Types.Quat, q_ref::Sim.Types.Quat)
    d = abs(sum(q .* q_ref))
    d = clamp(d, 0.0, 1.0)
    return 2.0 * acos(d)
end

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

@testset "PX4Lockstep ABI handshake helper" begin
    # Should pass with self-reported expectations.
    PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(sizeof(PX4Lockstep.LockstepInputs)),
        UInt32(sizeof(PX4Lockstep.LockstepOutputs)),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION + UInt32(1),
        UInt32(sizeof(PX4Lockstep.LockstepInputs)),
        UInt32(sizeof(PX4Lockstep.LockstepOutputs)),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(sizeof(PX4Lockstep.LockstepInputs)) + UInt32(4),
        UInt32(sizeof(PX4Lockstep.LockstepOutputs)),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )
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

@testset "Environment ISA altitude uses origin_alt_msl_m" begin
    atm = Sim.Environment.ISA1976()
    env = Sim.Environment.EnvironmentModel(atmosphere = atm, origin_alt_msl_m = 1000.0)
    # At z=0 (at home origin), MSL altitude should be origin_alt.
    rho0 = Sim.Environment.air_density(env.atmosphere, env.origin_alt_msl_m - 0.0)
    @test isapprox(rho0, Sim.Environment.air_density(atm, 1000.0); rtol = 1e-12)

    # At z=-100 (100 m above home), MSL altitude should be 1100 m.
    rho1 = Sim.Environment.air_density(env.atmosphere, env.origin_alt_msl_m - (-100.0))
    @test isapprox(rho1, Sim.Environment.air_density(atm, 1100.0); rtol = 1e-12)
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
    propulsion = Sim.Propulsion.default_iris_quadrotor_set(km_m = 0.05, thrust_hover_per_rotor_n = hover_T)
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

@testset "Propulsion owns rotor_dir yaw-torque sign" begin
    model = Sim.Vehicles.IrisQuadrotor()
    hover_T = model.params.mass * 9.80665 / 4.0
    prop = Sim.Propulsion.default_iris_quadrotor_set(km_m = 0.05, thrust_hover_per_rotor_n = hover_T)

    # Spin only rotor 3 (which has rotor_dir = -1 in the default set).
    duties = SVector{4,Float64}(0.0, 0.0, 0.5, 0.0)
    out = Sim.Propulsion.step_propulsion!(prop, duties, 16.0, 1.225, Sim.Types.vec3(0.0, 0.0, 0.0), 0.002)

    @test out.shaft_torque_nm[3] < 0.0

    env = Sim.Environment.EnvironmentModel()
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    d = Sim.Vehicles.dynamics(model, env, 0.0, x, out)
    @test d.ω_dot[3] < 0.0
end

@testset "Analytic: free-fall under gravity matches closed form" begin
    g = 9.80665
    dt = 0.01
    T = 1.0
    n = Int(round(T / dt))

    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.Types.Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Closed form in NED (z is down):
    #   v_z(t) = g t
    #   z(t)   = 0.5 g t^2
    z_exp = 0.5 * g * T^2
    vz_exp = g * T

    @test isapprox(x.pos_ned[3], z_exp; atol = 1e-9)
    @test isapprox(x.vel_ned[3], vz_exp; atol = 1e-9)
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)
end

@testset "Analytic: constant body-rate quaternion integration" begin
    dt = 0.001
    T = 1.0
    n = Int(round(T / dt))

    ω = π  # rad/s about body Z (down)
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, ω),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            vel_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Exact solution: q(t) = [cos(ωt/2), 0, 0, sin(ωt/2)]
    q_ref = Sim.Types.quat_from_axis_angle(Sim.Types.vec3(0.0, 0.0, 1.0), ω*T)
    θ_err = quat_angle_error(x.q_bn, q_ref)

    @test θ_err < 1e-6
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)

    yaw = Sim.Types.yaw_from_quat(x.q_bn)
    @test isapprox(Sim.Types.wrap_pi(yaw), Sim.Types.wrap_pi(ω*T); atol = 1e-6)
end

@testset "Analytic: DelayedEstimator ring-buffer exactness" begin
    rng = MersenneTwister(0)
    dt_est = 0.01
    delay_s = 0.03
    delay_steps = 3

    inner = Sim.Estimators.TruthEstimator()
    est = Sim.Estimators.DelayedEstimator(inner; delay_s = delay_s, dt_est = dt_est)
    Sim.Estimators.reset!(est)

    outs = Float64[]
    for k = 0:10
        t = k * dt_est
        x = Sim.RigidBody.RigidBodyState(
            pos_ned = Sim.Types.vec3(Float64(k), 0.0, 0.0),
            vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        y = Sim.Estimators.estimate!(est, rng, t, x, dt_est)
        push!(outs, y.pos_ned[1])
    end

    # During fill, the estimator returns current.
    @test outs[1] == 0.0
    @test outs[2] == 1.0
    @test outs[3] == 2.0

    # After warm-up, output is delayed by exactly delay_steps samples.
    for k = delay_steps+1:11
        # outs is 1-indexed; k corresponds to truth sample k-1.
        truth_k = k - 1
        @test outs[k] == Float64(truth_k - delay_steps)
    end

    # Reset should restore the warm-up behavior.
    Sim.Estimators.reset!(est)
    x = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(42.0, 0.0, 0.0))
    y = Sim.Estimators.estimate!(est, rng, 0.0, x, dt_est)
    @test y.pos_ned[1] == 42.0

    # Delay must be an exact multiple of dt_est (no silent rounding).
    @test_throws ErrorException Sim.Estimators.DelayedEstimator(
        inner;
        delay_s = 0.015,
        dt_est = dt_est,
    )
end

@testset "Cadence: 10-minute schedule has exact hits and constant time_us deltas" begin
    # This is intentionally a *scheduler-level* test: it proves no drift over long horizons
    # without running full vehicle dynamics.
    dt = 0.002
    dt_us = Int(round(dt * 1e6))
    ap_dt = 0.01
    ap_steps = Int(round(ap_dt / dt))

    total_time_s = 600.0  # 10 minutes
    total_steps = Int(round(total_time_s / dt))

    trig = Sim.Scheduling.StepTrigger(ap_steps)
    expected_hits = (total_steps - 1) ÷ ap_steps + 1

    hits = 0
    last_us = nothing
    for step = 0:(total_steps-1)
        if Sim.Scheduling.due(trig, step)
            t_us = UInt64(step * dt_us)
            if last_us !== nothing
                @test t_us - last_us == UInt64(ap_steps * dt_us)
            end
            last_us = t_us
            hits += 1
        end
    end

    @test hits == expected_hits

    last_trigger_step = ((total_steps - 1) ÷ ap_steps) * ap_steps
    @test last_us == UInt64(last_trigger_step * dt_us)
end
