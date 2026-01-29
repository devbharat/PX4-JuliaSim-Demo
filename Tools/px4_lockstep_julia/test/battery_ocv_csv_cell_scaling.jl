using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: ocv_csv_path with ocv_is_cell scaling" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain

    mktempdir() do dir
        csv = joinpath(dir, "ocv.csv")
        open(csv, "w") do io
            write(io, "used_soc,cell_ocv\n")
            write(io, "0,4.2\n")
            write(io, "100,3.0\n")
        end

        b = AC.build_battery(
            Sim.Aircraft.BatterySpec(
                model = :thevenin,
                series = 4,
                parallel = 1,
                ocv_csv_path = csv,
                ocv_csv_soc_col = "used_soc",
                ocv_csv_v_col = "cell_ocv",
                ocv_csv_soc_units = :percent,
                ocv_csv_soc_convention = :used,
                ocv_is_cell = true,
                r1 = 0.0,
                c1 = 0.0,
            ),
        )

        v_full = PT._interp_ocv(b.ocv, 1.0)
        v_empty = PT._interp_ocv(b.ocv, 0.0)
        @test isapprox(v_full, 4.0 * 4.2; atol = 1e-12)
        @test isapprox(v_empty, 4.0 * 3.0; atol = 1e-12)
    end
end
