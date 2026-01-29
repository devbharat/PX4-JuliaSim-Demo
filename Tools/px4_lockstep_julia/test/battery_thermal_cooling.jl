using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: thermal cooling behavior" begin
    AC = Sim.Aircraft
    PM = Sim.PlantModels

    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            thermal = Sim.Aircraft.BatteryThermalSpec(
                enabled = true,
                ambient_temp_c = 20.0,
                initial_temp_c = 30.0,
                c_th_j_per_k = 500.0,
                k_to_ambient_w_per_k = 2.0,
            ),
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    # With zero current and T > T_amb, temperature should decay toward ambient.
    temp_dot = PM._battery_temp_dot(b, 0.5, 30.0, 0.0, 0.0)
    @test temp_dot < 0.0
end
