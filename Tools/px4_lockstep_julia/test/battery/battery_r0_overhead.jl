using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: r0_overhead_ohm additivity" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain

    # Constant R0 path.
    b_const = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            r0 = 0.02,
            r0_overhead_ohm = 0.005,
            r1 = 0.0,
            c1 = 0.0,
        ),
    )
    @test isapprox(PT.r0_ohm(b_const.r0_model, 0.5, 25.0), 0.025; atol = 1e-12)

    # Surface R0 path.
    mktempdir() do dir
        csv = joinpath(dir, "surface.csv")
        open(csv, "w") do io
            write(io, "temp_c,soc,r0_ohm\n")
            write(io, "25.0,0.5,0.01\n")
            write(io, "25.0,1.0,0.01\n")
            write(io, "30.0,0.5,0.01\n")
            write(io, "30.0,1.0,0.01\n")
        end

        b_surf = AC.build_battery(
            Sim.Aircraft.BatterySpec(
                model = :thevenin,
                r0_surface_csv_path = csv,
                r0_surface_is_cell = false,
                r0_overhead_ohm = 0.004,
                r1 = 0.0,
                c1 = 0.0,
            ),
        )
        @test isapprox(PT.r0_ohm(b_surf.r0_model, 0.5, 25.0), 0.014; atol = 1e-12)
    end
end
