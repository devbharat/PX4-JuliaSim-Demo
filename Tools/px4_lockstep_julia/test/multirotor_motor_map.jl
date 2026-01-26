using Test
using StaticArrays
using LinearAlgebra
using PX4Lockstep

const Sim = PX4Lockstep.Sim

@testset "Phase 2: MotorMap maps PX4 channels -> physical propulsors" begin
    env = iris_env_replay_for_tests()

    # Use a small 4-motor multirotor and inspect the instantaneous rotor acceleration.
    N = 4
    rotor_pos = SVector(
        Sim.Types.vec3(0.1, 0.1, 0.0),
        Sim.Types.vec3(-0.1, -0.1, 0.0),
        Sim.Types.vec3(0.1, -0.1, 0.0),
        Sim.Types.vec3(-0.1, 0.1, 0.0),
    )
    rotor_axis = SVector{N,Sim.Types.Vec3}(ntuple(_ -> Sim.Types.vec3(0.0, 0.0, 1.0), N))
    I = Sim.Types.Mat3([0.01 0.0 0.0; 0.0 0.01 0.0; 0.0 0.0 0.02])

    params = Sim.Vehicles.QuadrotorParams{N}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = inv(I),
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{N,Float64}(ntuple(_ -> 0.0, N)),
        rotor_dir = SVector{N,Float64}(ntuple(_ -> 1.0, N)),
        linear_drag = 0.0,
        angular_damping = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    model = Sim.Vehicles.GenericMultirotor{N}(params)
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()

    hover_T = params.mass * 9.80665 / Float64(N)
    prop = Sim.Propulsion.default_multirotor_set(N = N, thrust_hover_per_rotor_n = hover_T)
    battery = Sim.Powertrain.IdealBattery()

    # Swap channels: physical motor #1 reads channel 2, physical motor #2 reads channel 1.
    motor_map = Sim.Vehicles.MotorMap{N}(SVector{N,Int}(2, 1, 3, 4))

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        model,
        env,
        Sim.Contacts.NoContact(),
        motor_act,
        servo_act,
        prop,
        battery;
        motor_map = motor_map,
    )

    plant0 = Sim.Plant.init_plant_state(
        Sim.RigidBody.RigidBodyState(),
        motor_act,
        servo_act,
        prop,
        battery,
    )

    # Command: channel1 low, channel2 high. With the mapping swap, ω̇[1] should exceed ω̇[2].
    motors = SVector{12,Float64}(ntuple(i -> (i == 1 ? 0.1 : (i == 2 ? 0.9 : 0.0)), 12))
    cmd = Sim.Vehicles.ActuatorCommand(
        motors = motors,
        servos = zero(SVector{8,Float64}),
    )
    u = Sim.Plant.PlantInput(
        cmd = cmd,
        wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        faults = Sim.Faults.FaultState(),
    )

    plant0_mapped =
        applicable(Sim.plant_on_autopilot_tick, dynfun, plant0, cmd) ?
        Sim.plant_on_autopilot_tick(dynfun, plant0, cmd) :
        plant0

    k1 = dynfun(0.0, plant0_mapped, u)
    @test all(isfinite, k1.rotor_ω_dot)
    @test k1.rotor_ω_dot[1] > k1.rotor_ω_dot[2]
end


@testset "Phase 2: Generic multirotor N=8 runs without PX4" begin
    env = iris_env_replay_for_tests()

    N = 8
    r = 0.25
    rotor_pos = SVector{N,Sim.Types.Vec3}(
        ntuple(i -> begin
            θ = 2.0 * pi * (Float64(i - 1) / Float64(N))
            Sim.Types.vec3(r * cos(θ), r * sin(θ), 0.0)
        end, N),
    )
    rotor_axis = SVector{N,Sim.Types.Vec3}(ntuple(_ -> Sim.Types.vec3(0.0, 0.0, 1.0), N))
    I = Sim.Types.Mat3([0.05 0.0 0.0; 0.0 0.05 0.0; 0.0 0.0 0.10])

    params = Sim.Vehicles.QuadrotorParams{N}(
        mass = 2.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = inv(I),
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{N,Float64}(ntuple(_ -> 0.0, N)),
        rotor_dir = SVector{N,Float64}(ntuple(_ -> 1.0, N)),
        linear_drag = 0.05,
        angular_damping = Sim.Types.vec3(0.02, 0.02, 0.01),
    )

    model = Sim.Vehicles.GenericMultirotor{N}(params)
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()

    hover_T = params.mass * 9.80665 / Float64(N)
    prop = Sim.Propulsion.default_multirotor_set(N = N, thrust_hover_per_rotor_n = hover_T)
    battery = Sim.Powertrain.IdealBattery()

    motor_map = Sim.Vehicles.MotorMap{N}(SVector{N,Int}(ntuple(i -> i, N)))

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        model,
        env,
        Sim.Contacts.NoContact(),
        motor_act,
        servo_act,
        prop,
        battery;
        motor_map = motor_map,
    )

    plant0 = Sim.Plant.init_plant_state(
        Sim.RigidBody.RigidBodyState(),
        motor_act,
        servo_act,
        prop,
        battery,
    )

    timeline = iris_timeline_for_tests(
        t_end_s = 0.2,
        dt_autopilot_s = 0.01,
        dt_wind_s = 0.01,
        dt_log_s = 0.2,
        dt_phys_s = nothing,
    )

    # Rough hover-ish command. This is only a smoke test; we care about determinism and
    # numerical stability, not perfect trim.
    hover_duty = 0.65
    motors = SVector{12,Float64}(ntuple(i -> (i <= N ? hover_duty : 0.0), 12))
    cmd = Sim.Vehicles.ActuatorCommand(
        motors = motors,
        servos = zero(SVector{8,Float64}),
    )

    cmd_trace = Sim.Recording.ZOHTrace(timeline.ap, fill(cmd, length(timeline.ap.t_us)))
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_trace)

    wind_trace = Sim.Recording.SampleHoldTrace(
        timeline.wind,
        fill(Sim.Types.vec3(0.0, 0.0, 0.0), length(timeline.wind.t_us)),
    )
    wind_src = Sim.Sources.ReplayWindSource(wind_trace)

    eng = Sim.simulate(
        mode = :live,
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        strict_lockstep_rates = false,
    )

    # Numerical sanity: no NaNs/Infs.
    @test all(isfinite, eng.plant.rotor_ω)
    @test all(isfinite, eng.plant.rb.pos_ned)
    @test all(isfinite, eng.plant.rb.vel_ned)
end
