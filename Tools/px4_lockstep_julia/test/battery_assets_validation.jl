using Test

const Sim = PX4Lockstep.Sim

@testset "Battery assets: Example pack/cell invariants (Phase 6)" begin
    BA = Sim.Aircraft.BatteryAssets
    AC = Sim.Aircraft
    PT = Sim.Powertrain
    assets_root = normpath(joinpath(@__DIR__, "..", "src", "Workflows", "assets", "battery"))
    cell_meta = joinpath(assets_root, "cells", "example_3290mah_hv", "meta.toml")
    pack_meta = joinpath(assets_root, "packs", "example_8s1p_3290mah", "meta.toml")

    # -------------------------------------------------------------------------
    # 1) Asset loader sanity (Phase 1/3): meta.toml parses and paths resolve.
    # -------------------------------------------------------------------------
    cell = BA.load_cell_asset(cell_meta)
    pack = BA.load_pack_asset(pack_meta)

    @test pack.series == 8
    @test pack.parallel == 1
    @test pack.cell.id == cell.id
    @test isfile(cell.ocv_csv)
    @test pack.r0_surface_csv !== nothing
    @test isfile(pack.r0_surface_csv)
    @test cell.r0_surface_csv !== nothing
    @test isfile(cell.r0_surface_csv)

    # -------------------------------------------------------------------------
    # 2) SOC convention + OCV endpoints (Phase 0/2):
    #    - raw Example CSV uses used_soc (DoD)
    #    - processed assets use soc remaining.
    # -------------------------------------------------------------------------
    # Raw OCV artifact in the repo (kept for traceability).
    raw_ocv = joinpath(
        assets_root,
        "raw",
        "example_3290mah_8s1p",
        "example_cell_ocv_soc.csv",
    )
    @test isfile(raw_ocv)

    # Load raw curve directly with conversion (used_soc -> soc remaining).
    soc_raw, v_raw = PT.load_ocv_curve_csv(
        raw_ocv;
        soc_col = "used_soc",
        v_col = "cell_ocv",
        soc_units = :percent,
        soc_convention = :used,
        step = 0.005, # match processed asset grid (0.5% steps)
    )
    @test soc_raw[1] == 0.0
    @test soc_raw[end] == 1.0

    # Endpoints should match expected HV-ish cell range.
    @test isapprox(v_raw[1], 3.0; atol = 0.05)   # empty
    @test isapprox(v_raw[end], 4.4; atol = 0.05) # full

    # Load the processed asset curve.
    soc_proc, v_proc = PT.load_ocv_curve_csv(
        cell.ocv_csv;
        soc_col = cell.ocv_soc_col,
        v_col = cell.ocv_v_col,
        soc_units = cell.ocv_soc_units,
        soc_convention = cell.ocv_soc_convention,
        step = nothing,
    )

    # The processed asset should be consistent with the raw conversion.
    @test length(soc_raw) == length(soc_proc)
    @test all(soc_raw .== soc_proc)
    # Small tolerance because the processed CSV is formatted/rounded.
    @test maximum(abs.(v_raw .- v_proc)) < 1e-3

    # -------------------------------------------------------------------------
    # 3) OCV scaling (Phase 4): cell curve scaled by series count equals pack OCV.
    # -------------------------------------------------------------------------
    b_pack = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            pack_asset = pack_meta,
            temp_c = 25.0,
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    # Build a pack from the cell asset directly (same series/parallel) so we can
    # compare scaling behavior across asset types.
    b_from_cell = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            cell_asset = cell_meta,
            series = 8,
            parallel = 1,
            temp_c = 25.0,
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    # Compare pack OCV to cell OCV * 8 at a few key SOC points.
    cell_tbl = PT.OCVTable(soc_proc, v_proc)
    for s in (0.0, 0.1, 0.5, 0.9, 1.0)
        v_cell = PT._interp_ocv(cell_tbl, s)
        v_pack = PT._interp_ocv(b_pack.ocv, s)
        @test isapprox(v_pack, 8.0 * v_cell; atol = 1e-9)
        # And the pack built from the cell asset should match too.
        v_pack2 = PT._interp_ocv(b_from_cell.ocv, s)
        @test isapprox(v_pack2, v_pack; atol = 1e-9)
    end

    # Pack endpoints implied by the curve.
    @test isapprox(PT._interp_ocv(b_pack.ocv, 1.0), 35.2; atol = 0.5)
    @test isapprox(PT._interp_ocv(b_pack.ocv, 0.0), 24.0; atol = 0.5)

    # -------------------------------------------------------------------------
    # 4) Resistance surface table-point exactness (Phase 4/5):
    #    interpolating at grid points returns the stored values.
    # -------------------------------------------------------------------------
    surf_pack_rdc = PT.load_bilinear_surface_csv(pack.r0_surface_csv; z_col = "rdc_ohm")
    surf_cell_rdc = PT.load_bilinear_surface_csv(cell.r0_surface_csv; z_col = "rdc_ohm")

    # Also validate that the derived cell surface is exactly pack/8 at the grid points.
    # (This is an internal consistency check for the current Example asset generation.)
    open(pack.r0_surface_csv, "r") do io
        hdr = split(chomp(readline(io)), ',')
        idx = Dict{String,Int}(name => i for (i, name) in enumerate(hdr))
        for name in ("temp_c", "soc", "rdc_ohm")
            @test haskey(idx, name)
        end
        for line in eachline(io)
            s = strip(line)
            isempty(s) && continue
            cols = split(s, ',')
            temp_c = parse(Float64, cols[idx["temp_c"]])
            soc = parse(Float64, cols[idx["soc"]])
            rdc_pack = parse(Float64, cols[idx["rdc_ohm"]])

            rdc_lookup = PT._interp_bilinear(surf_pack_rdc, temp_c, soc)
            @test isapprox(rdc_lookup, rdc_pack; atol = 1e-12)

            rdc_cell_lookup = PT._interp_bilinear(surf_cell_rdc, temp_c, soc)
            @test isapprox(rdc_cell_lookup * 8.0, rdc_pack; atol = 5e-6)
        end
    end

    # Scaling check at the model level: a pack built from a cell surface should
    # yield the same pack-level resistance.
    soc_q = 0.50
    temp_q = 20.0
    r_pack = PT.r0_ohm(b_pack.r0_model, soc_q, temp_q)
    r_pack2 = PT.r0_ohm(b_from_cell.r0_model, soc_q, temp_q)
    @test isapprox(r_pack2, r_pack; atol = 5e-6)

    # -------------------------------------------------------------------------
    # 5) End-to-end discharge sanity (Phase 6): constant-current discharge should
    #    deliver plausible energy and be internally consistent.
    # -------------------------------------------------------------------------
    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            pack_asset = pack_meta,
            temp_c = 25.0,
            # Keep Phase-5 policy: DCIR as effective droop resistance.
            r1 = 0.0,
            c1 = 0.0,
            min_voltage_v = 0.0,
        ),
    )
    st = PT.battery_state(b)

    I = 20.0
    dt = 1.0
    e_deliv_j = 0.0
    e_ocv_j = 0.0

    # Simulate until SOC is depleted.
    for _ = 1:2000
        st.soc <= 0.0 && break

        soc = st.soc
        ocv = PT._interp_ocv(b.ocv, soc)
        r0 = PT.r0_ohm(b.r0_model, soc, b.temp_c)
        v_term = ocv - I * r0

        # Accumulate energy (J).
        e_deliv_j += max(0.0, v_term) * I * dt
        e_ocv_j += ocv * I * dt

        PT.step!(b, st, I, dt)
    end

    @test st.soc == 0.0

    wh_deliv = e_deliv_j / 3600.0
    wh_ocv = e_ocv_j / 3600.0

    # Sanity envelopes.
    @test wh_ocv > 80.0
    @test wh_ocv < 130.0
    @test wh_deliv > 70.0
    @test wh_deliv < wh_ocv
end

@testset "Battery assets: TOML overrides asset (warns)" begin
    assets_root = normpath(joinpath(@__DIR__, "..", "src", "Workflows", "assets", "battery"))
    cell_meta = joinpath(assets_root, "cells", "example_3290mah_hv", "meta.toml")
    pack_meta = joinpath(assets_root, "packs", "example_8s1p_3290mah", "meta.toml")
    mktempdir() do dir
        toml = """
        schema_version = 1
        [power]
        [[power.batteries]]
        id = "bat1"
        model = "thevenin"
        pack_asset = "$(pack_meta)"
        series = 4
        r0_surface_r0_col = "r0_ohm"
        """
        path = joinpath(dir, "spec.toml")
        write(path, toml)

        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        bspec = spec.power.batteries[1]

        cell = Sim.Aircraft.BatteryAssets.load_cell_asset(cell_meta)
        soc_raw, v_raw = Sim.Powertrain.load_ocv_curve_csv(
            cell.ocv_csv;
            soc_col = cell.ocv_soc_col,
            v_col = cell.ocv_v_col,
            soc_units = cell.ocv_soc_units,
            soc_convention = cell.ocv_soc_convention,
        )
        soc_norm, v_norm = Sim.Powertrain.normalize_ocv_curve(soc_raw, v_raw)
        cell_ocv = Sim.Powertrain._interp_ocv(soc_norm, v_norm, 1.0)

        batt = Test.@test_logs(
            (:warn, r"series=4.*overrides asset value 8"),
            (:warn, r"r0_surface_r0_col=r0_ohm.*overrides asset value rdc_ohm"),
            Sim.Aircraft.build_battery(bspec),
        )

        pack_ocv = Sim.Powertrain._interp_ocv(batt.ocv, 1.0)
        @test isapprox(pack_ocv, 4.0 * cell_ocv; atol = 1e-6, rtol = 0)
    end
end
