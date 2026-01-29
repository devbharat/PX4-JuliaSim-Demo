using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: r0_surface_is_cell scaling" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain

    mktempdir() do dir
        csv = joinpath(dir, "surface.csv")
        open(csv, "w") do io
            write(io, "temp_c,soc,r0_ohm\n")
            write(io, "20.0,0.5,0.01\n")
            write(io, "20.0,1.0,0.01\n")
            write(io, "30.0,0.5,0.01\n")
            write(io, "30.0,1.0,0.01\n")
        end

        b_pack = AC.build_battery(
            Sim.Aircraft.BatterySpec(
                model = :thevenin,
                series = 4,
                parallel = 2,
                r0_surface_csv_path = csv,
                r0_surface_is_cell = true,
                r1 = 0.0,
                c1 = 0.0,
            ),
        )

        # Ns/Np scaling for cell-level surface: 4/2 = 2x
        r0 = PT.r0_ohm(b_pack.r0_model, 0.5, 20.0)
        @test isapprox(r0, 0.02; atol = 1e-12)
    end
end
