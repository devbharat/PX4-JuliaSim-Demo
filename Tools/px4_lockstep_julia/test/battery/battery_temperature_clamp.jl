using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: temperature clamp in plant_project" begin
    env = iris_env_replay_for_tests()
    veh = iris_vehicle_for_tests()
    batt = iris_battery_for_tests()
    model = iris_dynfun_for_tests(env, veh, batt)

    x0 = Sim.Plant.init_plant_state(
        veh.state,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )
    N = typeof(x0).parameters[1]
    B = typeof(x0).parameters[2]

    temp_hi = Sim.Types.SVector{B,Float64}(ntuple(_ -> 200.0, B))
    power = Sim.Plant.PowerState{B}(soc = x0.power.soc, v1 = x0.power.v1, temp_c = temp_hi)
    x_bad = Sim.Plant.PlantState{N,B}(
        rb = x0.rb,
        motors_y = x0.motors_y,
        motors_ydot = x0.motors_ydot,
        servos_y = x0.servos_y,
        servos_ydot = x0.servos_ydot,
        rotor_ω = x0.rotor_ω,
        power = power,
    )

    x_proj = Sim.PlantModels.plant_project(model, x_bad)
    @test all(T -> T <= 120.0, x_proj.power.temp_c)
end
