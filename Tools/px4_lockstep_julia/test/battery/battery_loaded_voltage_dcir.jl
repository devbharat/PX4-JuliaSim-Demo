using Test
using Statistics

const Sim = PX4Lockstep.Sim

@testset "Battery: Example DCIR matches loaded voltage sag (Phase 5)" begin
    AC = Sim.Aircraft
    PT = Sim.Powertrain
    assets_root = normpath(joinpath(@__DIR__, "..", "..", "src", "Workflows", "assets", "battery"))
    pack_meta = joinpath(assets_root, "packs", "example_8s1p_3290mah", "meta.toml")

    # Build a battery directly from the pack asset. We disable the RC branch here
    # because this test focuses on the *effective* droop resistance used by the bus solver
    # to match loaded voltage under thrust.
    b = AC.build_battery(
        Sim.Aircraft.BatterySpec(
            model = :thevenin,
            pack_asset = pack_meta,
            temp_c = 25.0,
            r1 = 0.0,
            c1 = 0.0,
        ),
    )

    # --- 1) Sanity: pack asset defaults to DCIR (rdc_ohm) as the series droop term.
    surf_csv = joinpath(dirname(pack_meta), "resistance_surface_pack.csv")

    surf_rdc = PT.load_bilinear_surface_csv(surf_csv; z_col = "rdc_ohm")
    surf_r0 = PT.load_bilinear_surface_csv(surf_csv; z_col = "r0_ohm")

    # Pick an interior query point so interpolation is well-defined.
    soc_q = 0.50
    temp_q = 10.0

    r_model = PT.r0_ohm(b.r0_model, soc_q, temp_q)
    r_expected = PT._interp_bilinear(surf_rdc, temp_q, soc_q)
    r_wrong = PT._interp_bilinear(surf_r0, temp_q, soc_q)

    @test isapprox(r_model, r_expected; atol = 1e-12)
    @test abs(r_model - r_wrong) > 1e-3

    # --- 2) Pulse sag regression: V_min ≈ V_pre - I * R_droop
    # The extracted pulse CSV uses V_pre and the end-of-pulse "Min Voltage" marker
    # (i.e., a DCIR-style sag definition). Using rdc_ohm as the droop term should
    # match this reasonably well.
    pulses_csv = joinpath(dirname(pack_meta), "dcir_extracted_pulses_pack.csv")

    errs = Float64[]
    open(pulses_csv, "r") do io
        hdr = split(chomp(readline(io)), ',')
        idx = Dict{String,Int}(name => i for (i, name) in enumerate(hdr))

        for name in ("temp_c", "soc_est", "current_a", "v_pre_v", "v_min_v")
            @test haskey(idx, name)
        end

        for line in eachline(io)
            s = strip(line)
            isempty(s) && continue
            cols = split(s, ',')

            temp_c = parse(Float64, cols[idx["temp_c"]])
            soc = parse(Float64, cols[idx["soc_est"]])
            I = parse(Float64, cols[idx["current_a"]])
            v_pre = parse(Float64, cols[idx["v_pre_v"]])
            v_min = parse(Float64, cols[idx["v_min_v"]])

            r = PT.r0_ohm(b.r0_model, soc, temp_c)
            v_pred = v_pre - I * r
            push!(errs, abs(v_pred - v_min))
        end
    end

    @test !isempty(errs)

    med = Statistics.median(errs)
    srt = sort(errs)
    p95_idx = clamp(Int(ceil(0.95 * length(srt))), 1, length(srt))
    p95 = srt[p95_idx]

    # These bounds are intentionally loose; the goal is to catch regressions
    # (wrong column, wrong SOC convention, broken interpolation), not to enforce
    # a perfect fit.
    @test med < 0.5
    @test p95 < 5.0
end
