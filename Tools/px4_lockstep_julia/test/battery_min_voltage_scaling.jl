using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: min_voltage_is_cell scaling" begin
    AC = Sim.Aircraft

    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            series = 4,
            parallel = 1,
            min_voltage_v = 3.1,
            min_voltage_is_cell = true,
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    @test isapprox(b.min_voltage_v, 12.4; atol = 1e-12)
end
