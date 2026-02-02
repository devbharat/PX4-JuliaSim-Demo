using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: ocv_csv_step resampling" begin
    AC = Sim.Aircraft

    mktempdir() do dir
        csv = joinpath(dir, "ocv.csv")
        open(csv, "w") do io
            write(io, "soc,ocv_v\n")
            write(io, "0.0,10.0\n")
            write(io, "0.25,10.5\n")
            write(io, "0.5,11.0\n")
            write(io, "0.75,11.5\n")
            write(io, "1.0,12.0\n")
        end

        b = AC.build_battery(
            Sim.Aircraft.BatterySpec(
                model = :thevenin,
                ocv_csv_path = csv,
                ocv_csv_soc_col = "soc",
                ocv_csv_v_col = "ocv_v",
                ocv_csv_step = 0.5,
                r1 = 0.0,
                c1 = 0.0,
            ),
        )

        @test length(b.ocv.soc) == 3
        @test isapprox(b.ocv.soc[1], 0.0; atol = 1e-12)
        @test isapprox(b.ocv.soc[2], 0.5; atol = 1e-12)
        @test isapprox(b.ocv.soc[3], 1.0; atol = 1e-12)
    end
end
