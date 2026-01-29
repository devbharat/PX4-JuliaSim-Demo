using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: r0_surface SOC units/convention" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain

    mktempdir() do dir
        csv = joinpath(dir, "surface.csv")
        open(csv, "w") do io
            write(io, "temp_c,soc,r0_ohm\n")
            # soc is in PERCENT and uses "used" convention (DoD)
            write(io, "20.0,0,0.010\n")
            write(io, "20.0,50,0.012\n")
            write(io, "20.0,100,0.014\n")
            write(io, "30.0,0,0.020\n")
            write(io, "30.0,50,0.022\n")
            write(io, "30.0,100,0.024\n")
        end

        b = AC.build_battery(
            Sim.Aircraft.BatterySpec(
                model = :thevenin,
                r0_surface_csv_path = csv,
                r0_surface_soc_units = :percent,
                r0_surface_soc_convention = :used,
                r1 = 0.0,
                c1 = 0.0,
            ),
        )

        # used=50% corresponds to soc_remaining=0.5
        r0 = PT.r0_ohm(b.r0_model, 0.5, 20.0)
        @test isapprox(r0, 0.012; atol = 1e-12)
    end
end
