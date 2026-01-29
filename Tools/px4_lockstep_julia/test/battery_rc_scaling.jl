using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: thevenin_is_cell RC scaling" begin
    AC = Sim.Aircraft

    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            series = 4,
            parallel = 2,
            thevenin_is_cell = true,
            r0 = 0.02,
            r1 = 0.01,
            c1 = 100.0,
            v1_0 = 0.3,
        ),
    )

    # scale_r = 4/2 = 2, scale_c = 2/4 = 0.5
    @test isapprox(b.r1, 0.02; atol = 1e-12)
    @test isapprox(b.c1, 50.0; atol = 1e-12)
    @test isapprox(b.v1_0, 1.2; atol = 1e-12)
end
