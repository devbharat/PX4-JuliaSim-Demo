@testset "Aircraft/Powertrain: battery pack scaling from a cell asset" begin
    # Phase 4 acceptance: the *same* cell asset should be usable to build
    # different pack configurations, with correct scaling laws.

    BA = Sim.Aircraft.BatteryAssets
    assets_root = normpath(joinpath(@__DIR__, "..", "..", "src", "Workflows", "assets", "battery"))
    cell_meta = joinpath(assets_root, "cells", "example_3290mah_hv", "meta.toml")

    cell = BA.load_cell_asset(cell_meta)

    # Build two packs from the same cell asset.
    b_8s1p = Sim.Aircraft.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            cell_asset = cell_meta,
            series = 8,
            parallel = 1,
            temp_c = 20.0,
        ),
    )

    b_4s2p = Sim.Aircraft.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            cell_asset = cell_meta,
            series = 4,
            parallel = 2,
            temp_c = 20.0,
        ),
    )

    # Capacity scales with parallel count only: Q_pack = Np * Q_cell.
    @test isapprox(b_4s2p.capacity_c, 2.0 * b_8s1p.capacity_c; rtol = 0, atol = 1e-9)

    # OCV scales with series count only: V_pack = Ns * V_cell.
    soc_q = 0.5
    ocv_cell_soc, ocv_cell_v = Sim.Powertrain.load_ocv_curve_csv(
        cell.ocv_csv;
        soc_col = cell.ocv_soc_col,
        v_col = cell.ocv_v_col,
        soc_units = cell.ocv_soc_units,
        soc_convention = cell.ocv_soc_convention,
    )
    ocv_cell = Sim.Powertrain._interp_ocv(
        Sim.Powertrain.normalize_ocv_curve(ocv_cell_soc, ocv_cell_v)...,
        soc_q,
    )

    @test isapprox(
        Sim.Powertrain._interp_ocv(b_8s1p.ocv, soc_q),
        8.0 * ocv_cell;
        rtol = 0,
        atol = 1e-6,
    )
    @test isapprox(
        Sim.Powertrain._interp_ocv(b_4s2p.ocv, soc_q),
        4.0 * ocv_cell;
        rtol = 0,
        atol = 1e-6,
    )

    # R0 (cell-dominated component) scales as R_pack = (Ns/Np) * R_cell.
    surf_cell = Sim.Powertrain.load_bilinear_surface_csv(
        cell.r0_surface_csv;
        x_col = cell.r0_temp_col,
        y_col = cell.r0_soc_col,
        z_col = cell.r0_col,
        y_units = cell.r0_soc_units,
        y_convention = cell.r0_soc_convention,
    )

    temp_q = 20.0
    soc_r = 0.525
    r0_cell = Sim.Powertrain._interp_bilinear(surf_cell, temp_q, soc_r)

    r0_8s1p = Sim.Powertrain.r0_ohm(b_8s1p.r0_model, soc_r, temp_q)
    r0_4s2p = Sim.Powertrain.r0_ohm(b_4s2p.r0_model, soc_r, temp_q)

    @test isapprox(r0_8s1p, (8.0 / 1.0) * r0_cell; rtol = 0, atol = 1e-9)
    @test isapprox(r0_4s2p, (4.0 / 2.0) * r0_cell; rtol = 0, atol = 1e-9)
end
