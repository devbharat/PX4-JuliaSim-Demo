using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: thermal hook basics (Phase 7)" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain
    PM = Sim.PlantModels
    assets_root = normpath(joinpath(@__DIR__, "..", "src", "Workflows", "assets", "battery"))
    pack_meta = joinpath(assets_root, "packs", "example_8s1p_3290mah", "meta.toml")

    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            pack_asset = pack_meta,
            temp_c = 25.0,
            thermal = Sim.Aircraft.BatteryThermalSpec(
                enabled = true,
                ambient_temp_c = 25.0,
                initial_temp_c = 25.0,
                c_th_j_per_k = 600.0,
                k_to_ambient_w_per_k = 1.0,
            ),
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    soc = 0.50
    r0_cold = PT.r0_ohm(b.r0_model, soc, 10.0)
    r0_warm = PT.r0_ohm(b.r0_model, soc, 30.0)
    @test r0_cold > r0_warm

    # At ambient temperature (T == T_amb), P_loss should heat the pack.
    temp_dot = PM._battery_temp_dot(b, soc, 25.0, 0.0, 20.0)
    @test temp_dot > 0.0
end
