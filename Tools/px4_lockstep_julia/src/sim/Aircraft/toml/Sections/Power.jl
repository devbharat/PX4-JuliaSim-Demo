function _parse_power(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::PowerSpec,
    base_dir::AbstractString,
)
    tbl = _as_table(tbl_any, ctx)
    strict && _known_keys!(tbl, Set(["batteries", "buses", "share_mode"]), ctx)

    bats = base.batteries
    if haskey(tbl, "batteries")
        arr = _as_array(tbl["batteries"], "$ctx.batteries")
        bats = BatterySpec[]
        for (i, b_any) in enumerate(arr)
            b = _as_table(b_any, "$ctx.batteries[$i]")
            explicit_keys = Set{Symbol}(Symbol(k) for k in keys(b))
            allowed = Set([
                "id",
                "model",
                "pack_asset",
                "cell_asset",
                "temp_c",
                "thermal",
                "series",
                "parallel",
                "capacity_ah",
                "cell_capacity_ah",
                "soc0",
                "voltage_v",
                "ocv_soc",
                "ocv_v",
                "ocv_csv_path",
                "ocv_csv_soc_col",
                "ocv_csv_v_col",
                "ocv_csv_soc_units",
                "ocv_csv_soc_convention",
                "ocv_csv_step",
                "ocv_is_cell",
                "r0_surface_csv_path",
                "r0_surface_temp_col",
                "r0_surface_soc_col",
                "r0_surface_r0_col",
                "r0_surface_soc_units",
                "r0_surface_soc_convention",
                "r0_surface_is_cell",
                "r0",
                "r0_overhead_ohm",
                "r1",
                "c1",
                "v1_0",
                "min_voltage_v",
                "thevenin_is_cell",
                "min_voltage_is_cell",
                "low_thr",
                "crit_thr",
                "emerg_thr",
            ])
            strict && _known_keys!(b, allowed, "$ctx.batteries[$i]")
            id =
                haskey(b, "id") ? _sym(_as_string(b["id"], "$ctx.batteries[$i].id")) : :bat1
            model =
                haskey(b, "model") ?
                _sym(_as_string(b["model"], "$ctx.batteries[$i].model")) : :thevenin

            # Asset references are explicit meta.toml paths (absolute or spec-relative).
            pack_asset = nothing
            if haskey(b, "pack_asset")
                pa = _as_string(b["pack_asset"], "$ctx.batteries[$i].pack_asset")
                endswith(lowercase(pa), ".toml") ||
                    error("$ctx.batteries[$i].pack_asset must be a path to meta.toml")
                pack_asset = _resolve_path(base_dir, pa; must_exist = true)
            end
            cell_asset = nothing
            if haskey(b, "cell_asset")
                ca = _as_string(b["cell_asset"], "$ctx.batteries[$i].cell_asset")
                endswith(lowercase(ca), ".toml") ||
                    error("$ctx.batteries[$i].cell_asset must be a path to meta.toml")
                cell_asset = _resolve_path(base_dir, ca; must_exist = true)
            end
            if pack_asset !== nothing && cell_asset !== nothing
                error("$ctx.batteries[$i]: cannot set both pack_asset and cell_asset")
            end

            temp_c =
                haskey(b, "temp_c") ? _as_f64(b["temp_c"], "$ctx.batteries[$i].temp_c") :
                25.0

            # Optional thermal hook. If enabled, the plant integrates a
            # per-battery temperature state and uses it as the temperature input for
            # resistance models (R0 surfaces / temperature fits).
            thermal = BatteryThermalSpec()
            if haskey(b, "thermal")
                th = _as_table(b["thermal"], "$ctx.batteries[$i].thermal")
                thermal_explicit_keys = Set{Symbol}(Symbol(k) for k in keys(th))
                if strict
                    _known_keys!(
                        th,
                        Set([
                            "enabled",
                            "ambient_temp_c",
                            "initial_temp_c",
                            "c_th_j_per_k",
                            "k_to_ambient_w_per_k",
                        ]),
                        "$ctx.batteries[$i].thermal",
                    )
                end

                enabled =
                    haskey(th, "enabled") ?
                    _as_bool(th["enabled"], "$ctx.batteries[$i].thermal.enabled") : false

                ambient_temp_c =
                    haskey(th, "ambient_temp_c") ?
                    _as_f64(
                        th["ambient_temp_c"],
                        "$ctx.batteries[$i].thermal.ambient_temp_c",
                    ) : nothing
                initial_temp_c =
                    haskey(th, "initial_temp_c") ?
                    _as_f64(
                        th["initial_temp_c"],
                        "$ctx.batteries[$i].thermal.initial_temp_c",
                    ) : nothing
                c_th_j_per_k =
                    haskey(th, "c_th_j_per_k") ?
                    _as_f64(th["c_th_j_per_k"], "$ctx.batteries[$i].thermal.c_th_j_per_k") :
                    nothing
                k_to_ambient_w_per_k =
                    haskey(th, "k_to_ambient_w_per_k") ?
                    _as_f64(
                        th["k_to_ambient_w_per_k"],
                        "$ctx.batteries[$i].thermal.k_to_ambient_w_per_k",
                    ) : nothing

                thermal = BatteryThermalSpec(
                    enabled = enabled,
                    ambient_temp_c = ambient_temp_c,
                    initial_temp_c = initial_temp_c,
                    c_th_j_per_k = c_th_j_per_k,
                    k_to_ambient_w_per_k = k_to_ambient_w_per_k,
                    explicit_keys = thermal_explicit_keys,
                )
            end
            series =
                haskey(b, "series") ? _as_int(b["series"], "$ctx.batteries[$i].series") : 1
            parallel =
                haskey(b, "parallel") ?
                _as_int(b["parallel"], "$ctx.batteries[$i].parallel") : 1
            ocv_soc =
                haskey(b, "ocv_soc") ?
                [
                    Float64(v) for
                    v in _as_array(b["ocv_soc"], "$ctx.batteries[$i].ocv_soc")
                ] : [0.0, 1.0]
            ocv_v =
                haskey(b, "ocv_v") ?
                [Float64(v) for v in _as_array(b["ocv_v"], "$ctx.batteries[$i].ocv_v")] :
                [10.8, 12.6]
            ocv_csv_path =
                haskey(b, "ocv_csv_path") ?
                _resolve_path(
                    base_dir,
                    _as_string(b["ocv_csv_path"], "$ctx.batteries[$i].ocv_csv_path");
                    must_exist = true,
                ) : nothing
            ocv_csv_soc_col =
                haskey(b, "ocv_csv_soc_col") ?
                _as_string(b["ocv_csv_soc_col"], "$ctx.batteries[$i].ocv_csv_soc_col") :
                "soc"
            ocv_csv_v_col =
                haskey(b, "ocv_csv_v_col") ?
                _as_string(b["ocv_csv_v_col"], "$ctx.batteries[$i].ocv_csv_v_col") : "ocv_v"
            ocv_csv_soc_units =
                haskey(b, "ocv_csv_soc_units") ?
                _norm_kind(b["ocv_csv_soc_units"], "$ctx.batteries[$i].ocv_csv_soc_units") :
                :fraction
            ocv_csv_soc_units =
                ocv_csv_soc_units in (:fraction, :frac) ? :fraction :
                ocv_csv_soc_units in (:percent, :pct, :percentage) ? :percent :
                error(
                    "$ctx.batteries[$i].ocv_csv_soc_units must be one of 'fraction'|'percent' (got $(ocv_csv_soc_units))",
                )
            ocv_csv_soc_convention =
                haskey(b, "ocv_csv_soc_convention") ?
                _norm_kind(
                    b["ocv_csv_soc_convention"],
                    "$ctx.batteries[$i].ocv_csv_soc_convention",
                ) : :remaining
            ocv_csv_soc_convention =
                ocv_csv_soc_convention in (:remaining, :soc, :soc_remaining) ? :remaining :
                ocv_csv_soc_convention in (:used, :dod, :depth_of_discharge, :used_soc) ?
                :used :
                error(
                    "$ctx.batteries[$i].ocv_csv_soc_convention must be one of 'remaining'|'used'|'dod' (got $(ocv_csv_soc_convention))",
                )
            ocv_csv_step =
                haskey(b, "ocv_csv_step") ?
                _as_f64(b["ocv_csv_step"], "$ctx.batteries[$i].ocv_csv_step") : nothing
            ocv_is_cell =
                haskey(b, "ocv_is_cell") ?
                _as_bool(b["ocv_is_cell"], "$ctx.batteries[$i].ocv_is_cell") : false

            # Optional resistance surface (used by model=:thevenin when provided).
            r0_surface_csv_path =
                haskey(b, "r0_surface_csv_path") ?
                _resolve_path(
                    base_dir,
                    _as_string(
                        b["r0_surface_csv_path"],
                        "$ctx.batteries[$i].r0_surface_csv_path",
                    );
                    must_exist = true,
                ) : nothing
            r0_surface_temp_col =
                haskey(b, "r0_surface_temp_col") ?
                _as_string(
                    b["r0_surface_temp_col"],
                    "$ctx.batteries[$i].r0_surface_temp_col",
                ) : nothing
            r0_surface_soc_col =
                haskey(b, "r0_surface_soc_col") ?
                _as_string(
                    b["r0_surface_soc_col"],
                    "$ctx.batteries[$i].r0_surface_soc_col",
                ) : nothing
            r0_surface_r0_col =
                haskey(b, "r0_surface_r0_col") ?
                _as_string(b["r0_surface_r0_col"], "$ctx.batteries[$i].r0_surface_r0_col") :
                nothing
            r0_surface_soc_units =
                haskey(b, "r0_surface_soc_units") ?
                _norm_kind(
                    b["r0_surface_soc_units"],
                    "$ctx.batteries[$i].r0_surface_soc_units",
                ) : nothing
            if r0_surface_soc_units !== nothing
                r0_surface_soc_units =
                    r0_surface_soc_units in (:fraction, :frac) ? :fraction :
                    r0_surface_soc_units in (:percent, :pct, :percentage) ? :percent :
                    error(
                        "$ctx.batteries[$i].r0_surface_soc_units must be one of 'fraction'|'percent' (got $(r0_surface_soc_units))",
                    )
            end
            r0_surface_soc_convention =
                haskey(b, "r0_surface_soc_convention") ?
                _norm_kind(
                    b["r0_surface_soc_convention"],
                    "$ctx.batteries[$i].r0_surface_soc_convention",
                ) : nothing
            if r0_surface_soc_convention !== nothing
                r0_surface_soc_convention =
                    r0_surface_soc_convention in (:remaining, :soc, :soc_remaining) ?
                    :remaining :
                    r0_surface_soc_convention in
                    (:used, :dod, :depth_of_discharge, :used_soc) ? :used :
                    error(
                        "$ctx.batteries[$i].r0_surface_soc_convention must be one of 'remaining'|'used'|'dod' (got $(r0_surface_soc_convention))",
                    )
            end
            r0_surface_is_cell =
                haskey(b, "r0_surface_is_cell") ?
                _as_bool(b["r0_surface_is_cell"], "$ctx.batteries[$i].r0_surface_is_cell") :
                nothing
            cell_capacity_ah =
                haskey(b, "cell_capacity_ah") ?
                _as_f64(b["cell_capacity_ah"], "$ctx.batteries[$i].cell_capacity_ah") :
                nothing
            thevenin_is_cell =
                haskey(b, "thevenin_is_cell") ?
                _as_bool(b["thevenin_is_cell"], "$ctx.batteries[$i].thevenin_is_cell") :
                false
            min_voltage_is_cell =
                haskey(b, "min_voltage_is_cell") ?
                _as_bool(
                    b["min_voltage_is_cell"],
                    "$ctx.batteries[$i].min_voltage_is_cell",
                ) : false
            push!(
                bats,
                BatterySpec(
                    id = id,
                    model = model,
                    pack_asset = pack_asset,
                    cell_asset = cell_asset,
                    temp_c = temp_c,
                    thermal = thermal,
                    series = series,
                    parallel = parallel,
                    capacity_ah = haskey(b, "capacity_ah") ?
                                  _as_f64(
                        b["capacity_ah"],
                        "$ctx.batteries[$i].capacity_ah",
                    ) : 5.0,
                    cell_capacity_ah = cell_capacity_ah,
                    soc0 = haskey(b, "soc0") ?
                           _as_f64(b["soc0"], "$ctx.batteries[$i].soc0") : 1.0,
                    voltage_v = haskey(b, "voltage_v") ?
                                _as_f64(b["voltage_v"], "$ctx.batteries[$i].voltage_v") :
                                nothing,
                    ocv_soc = ocv_soc,
                    ocv_v = ocv_v,
                    ocv_csv_path = ocv_csv_path,
                    ocv_csv_soc_col = ocv_csv_soc_col,
                    ocv_csv_v_col = ocv_csv_v_col,
                    ocv_csv_soc_units = ocv_csv_soc_units,
                    ocv_csv_soc_convention = ocv_csv_soc_convention,
                    ocv_csv_step = ocv_csv_step,
                    ocv_is_cell = ocv_is_cell,
                    r0_surface_csv_path = r0_surface_csv_path,
                    r0_surface_temp_col = r0_surface_temp_col,
                    r0_surface_soc_col = r0_surface_soc_col,
                    r0_surface_r0_col = r0_surface_r0_col,
                    r0_surface_soc_units = r0_surface_soc_units,
                    r0_surface_soc_convention = r0_surface_soc_convention,
                    r0_surface_is_cell = r0_surface_is_cell,
                    r0 = haskey(b, "r0") ? _as_f64(b["r0"], "$ctx.batteries[$i].r0") :
                         0.020,
                    r0_overhead_ohm = haskey(b, "r0_overhead_ohm") ?
                                      _as_f64(
                        b["r0_overhead_ohm"],
                        "$ctx.batteries[$i].r0_overhead_ohm",
                    ) : 0.0,
                    r1 = haskey(b, "r1") ? _as_f64(b["r1"], "$ctx.batteries[$i].r1") :
                         0.010,
                    c1 = haskey(b, "c1") ? _as_f64(b["c1"], "$ctx.batteries[$i].c1") :
                         2000.0,
                    v1_0 = haskey(b, "v1_0") ?
                           _as_f64(b["v1_0"], "$ctx.batteries[$i].v1_0") : 0.0,
                    min_voltage_v = haskey(b, "min_voltage_v") ?
                                    _as_f64(
                        b["min_voltage_v"],
                        "$ctx.batteries[$i].min_voltage_v",
                    ) : 9.9,
                    thevenin_is_cell = thevenin_is_cell,
                    min_voltage_is_cell = min_voltage_is_cell,
                    low_thr = haskey(b, "low_thr") ?
                              _as_f64(b["low_thr"], "$ctx.batteries[$i].low_thr") : 0.15,
                    crit_thr = haskey(b, "crit_thr") ?
                               _as_f64(b["crit_thr"], "$ctx.batteries[$i].crit_thr") : 0.10,
                    emerg_thr = haskey(b, "emerg_thr") ?
                                _as_f64(b["emerg_thr"], "$ctx.batteries[$i].emerg_thr") :
                                0.05,
                    explicit_keys = explicit_keys,
                ),
            )
        end
    end

    buses = base.buses
    if haskey(tbl, "buses")
        arr = _as_array(tbl["buses"], "$ctx.buses")
        buses = PowerBusSpec[]
        for (i, bus_any) in enumerate(arr)
            bus = _as_table(bus_any, "$ctx.buses[$i]")
            strict && _known_keys!(
                bus,
                Set(["id", "battery_ids", "motor_ids", "servo_ids", "avionics_load_w"]),
                "$ctx.buses[$i]",
            )
            id = _sym(_as_string(get(bus, "id", nothing), "$ctx.buses[$i].id"))
            b_ids = [
                _sym(_as_string(v, "$ctx.buses[$i].battery_ids")) for v in _as_array(
                    get(bus, "battery_ids", BatteryId[]),
                    "$ctx.buses[$i].battery_ids",
                )
            ]
            m_ids = [
                _sym(_as_string(v, "$ctx.buses[$i].motor_ids")) for v in
                _as_array(get(bus, "motor_ids", MotorId[]), "$ctx.buses[$i].motor_ids")
            ]
            s_ids = [
                _sym(_as_string(v, "$ctx.buses[$i].servo_ids")) for v in
                _as_array(get(bus, "servo_ids", ServoId[]), "$ctx.buses[$i].servo_ids")
            ]
            avionics =
                haskey(bus, "avionics_load_w") ?
                _as_f64(bus["avionics_load_w"], "$ctx.buses[$i].avionics_load_w") : 0.0
            push!(
                buses,
                PowerBusSpec(
                    id = id,
                    battery_ids = b_ids,
                    motor_ids = m_ids,
                    servo_ids = s_ids,
                    avionics_load_w = avionics,
                ),
            )
        end
    end

    share_mode = base.share_mode
    if haskey(tbl, "share_mode")
        share_mode = _norm_kind(tbl["share_mode"], "$ctx.share_mode")
        share_mode =
            share_mode in (:inv_r0, :equal) ? share_mode :
            error("$ctx.share_mode must be 'inv_r0' or 'equal'")
    end

    return PowerSpec(batteries = bats, buses = buses, share_mode = share_mode)
end
