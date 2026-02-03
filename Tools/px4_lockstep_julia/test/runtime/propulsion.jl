@testset "Propulsion owns rotor_dir yaw-torque sign" begin
    env = PX4Lockstep.Tests.Fixtures.iris_env_replay_for_tests()
    vehicle = PX4Lockstep.Tests.Fixtures.iris_vehicle_for_tests()
    battery = PX4Lockstep.Tests.Fixtures.iris_battery_for_tests()

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        Sim.Contacts.NoContact(),
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    plant0 = Sim.Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Spin only rotor 3 (which has rotor_dir = -1 in the default set).
    motors = SVector{12,Float64}(ntuple(i -> (i == 3 ? 0.5 : 0.0), 12))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = zero(SVector{8,Float64}))
    u = Sim.Plant.PlantInput(
        cmd = cmd,
        wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        faults = Sim.Faults.FaultState(),
    )

    plant0_mapped =
        applicable(Sim.plant_on_autopilot_tick, dynfun, plant0, cmd) ?
        Sim.plant_on_autopilot_tick(dynfun, plant0, cmd) :
        plant0
    plant0_spin = Sim.Plant.PlantState{4,1}(
        rb = plant0_mapped.rb,
        motors_y = plant0_mapped.motors_y,
        motors_ydot = plant0_mapped.motors_ydot,
        servos_y = plant0_mapped.servos_y,
        servos_ydot = plant0_mapped.servos_ydot,
        rotor_ω = SVector{4,Float64}(0.0, 0.0, 300.0, 0.0),
        power = plant0_mapped.power,
    )

    y = Sim.plant_outputs(dynfun, 0.0, plant0_spin, u)
    out = y.rotors
    @test out.shaft_torque_nm[3] < 0.0

    d = Sim.Vehicles.dynamics(
        vehicle.model,
        env,
        0.0,
        plant0_spin.rb,
        out,
        Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    @test d.ω_dot[3] < 0.0
end