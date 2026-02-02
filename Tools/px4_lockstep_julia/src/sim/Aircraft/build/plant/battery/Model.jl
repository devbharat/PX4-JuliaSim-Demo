"""Battery model construction helpers."""

function _build_battery(bs::BatterySpec)
    resolved = _resolve_battery_assets(bs)

    series = resolved.series
    parallel = resolved.parallel
    capacity_ah = resolved.capacity_ah

    ocv_soc = resolved.ocv_soc
    ocv_v = resolved.ocv_v
    ocv_csv_path = resolved.ocv_csv_path
    ocv_csv_soc_col = resolved.ocv_csv_soc_col
    ocv_csv_v_col = resolved.ocv_csv_v_col
    ocv_csv_soc_units = resolved.ocv_csv_soc_units
    ocv_csv_soc_convention = resolved.ocv_csv_soc_convention
    ocv_csv_step = resolved.ocv_csv_step
    ocv_is_cell = resolved.ocv_is_cell

    r0_surface_csv_path = resolved.r0_surface_csv_path
    r0_surface_temp_col = resolved.r0_surface_temp_col
    r0_surface_soc_col = resolved.r0_surface_soc_col
    r0_surface_r0_col = resolved.r0_surface_r0_col
    r0_surface_soc_units = resolved.r0_surface_soc_units
    r0_surface_soc_convention = resolved.r0_surface_soc_convention
    r0_surface_is_cell = resolved.r0_surface_is_cell

    thermal = _build_battery_thermal_params(bs, resolved.pack_asset_loaded)
    thermal_params = thermal.thermal_params
    temp_c = thermal.temp_c

    # Load/normalize OCV curve and apply pack scaling if needed.
    function _effective_ocv_curve()
        soc_pts, v_pts = if ocv_csv_path === nothing
            (ocv_soc, ocv_v)
        else
            Powertrain.load_ocv_curve_csv(
                ocv_csv_path;
                soc_col = ocv_csv_soc_col,
                v_col = ocv_csv_v_col,
                soc_units = ocv_csv_soc_units,
                soc_convention = ocv_csv_soc_convention,
                step = ocv_csv_step,
            )
        end

        soc_n, v_n = Powertrain.normalize_ocv_curve(soc_pts, v_pts)

        if ocv_is_cell
            v_n = [v * series for v in v_n]
        end

        return soc_n, v_n
    end

    # -----------------------------
    # Build battery model object
    # -----------------------------
    if bs.model === :thevenin_surface
        throw(
            ArgumentError(
                "Battery $(bs.id) model=:thevenin_surface is deprecated; use model=:thevenin with r0_surface_csv_path instead",
            ),
        )
    end
    if bs.model === :thevenin
        ocv_soc_n, ocv_v_n = _effective_ocv_curve()
        ocv_tbl = Powertrain.OCVTable(ocv_soc_n, ocv_v_n)

        # R0: constant (default) or data-driven surface.
        use_surface = r0_surface_csv_path !== nothing

        r0_model = if use_surface
            surf = Powertrain.load_bilinear_surface_csv(
                r0_surface_csv_path;
                x_col = r0_surface_temp_col,
                y_col = r0_surface_soc_col,
                z_col = r0_surface_r0_col,
                y_units = r0_surface_soc_units,
                y_convention = r0_surface_soc_convention,
            )
            r0_scale = r0_surface_is_cell ? (series / parallel) : 1.0
            Powertrain.R0BilinearSurface(surf, r0_scale)
        else
            # Back-compat scalar R0. If the user marks it as cell-level, scale to pack.
            r0 = bs.r0
            if bs.thevenin_is_cell
                r0 *= (series / parallel)
            end
            Powertrain.R0Constant(r0)
        end

        # Optional pack-level overhead term (connectors / wiring / BMS). This is
        # always interpreted as *pack-level* and therefore does not participate
        # in series/parallel cell scaling.
        if bs.r0_overhead_ohm > 0
            r0_model = Powertrain.R0Sum(r0_model, Powertrain.R0Constant(bs.r0_overhead_ohm))
        end

        # RC branch (still constant parameters for now).
        r1 = bs.r1
        c1 = bs.c1
        v1_0 = bs.v1_0
        if bs.thevenin_is_cell
            # Note: this scaling applies to the RC branch (r1,c1,v1_0). For surface R0,
            # use r0_surface_is_cell to scale the surface itself.
            scale_r = series / parallel
            scale_c = parallel / series
            r1 *= scale_r
            c1 *= scale_c
            v1_0 *= series
        end

        min_v = bs.min_voltage_is_cell ? bs.min_voltage_v * series : bs.min_voltage_v

        return Powertrain._with_direct_constructors() do
            Powertrain._thevenin_from_params(
                capacity_ah = capacity_ah,
                soc0 = bs.soc0,
                ocv = ocv_tbl,
                r0_model = r0_model,
                temp_c = temp_c,
                thermal = thermal_params,
                r1 = r1,
                c1 = c1,
                v1_0 = v1_0,
                min_voltage_v = min_v,
                low_thr = bs.low_thr,
                crit_thr = bs.crit_thr,
                emerg_thr = bs.emerg_thr,
            )
        end
    elseif bs.model === :ideal
        # Useful for tests and some prototyping.
        bs.voltage_v === nothing &&
            throw(ArgumentError("Battery $(bs.id) model=:ideal requires voltage_v"))
        V = bs.voltage_v
        return Powertrain._with_direct_constructors() do
            Powertrain.IdealBattery(
                capacity_ah = capacity_ah,
                soc0 = bs.soc0,
                voltage_v = V,
                temp_c = temp_c,
                thermal = thermal_params,
                low_thr = bs.low_thr,
                crit_thr = bs.crit_thr,
                emerg_thr = bs.emerg_thr,
            )
        end
    else
        throw(
            ArgumentError(
                "Unsupported battery model=$(bs.model) (expected :thevenin|:ideal)",
            ),
        )
    end
end
