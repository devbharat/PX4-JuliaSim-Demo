"""Battery asset resolution helpers."""

function _resolve_battery_assets(bs::BatterySpec)
    # -----------------------------
    # Resolve optional battery/cell assets
    # -----------------------------
    explicit = bs.explicit_keys
    series = bs.series
    parallel = bs.parallel

    # Capacity can be provided either as pack-level capacity_ah or as a per-cell capacity.
    capacity_ah = bs.capacity_ah
    cell_capacity_ah = bs.cell_capacity_ah

    # OCV curve configuration (can be overridden by assets)
    ocv_soc = bs.ocv_soc
    ocv_v = bs.ocv_v
    ocv_csv_path = bs.ocv_csv_path
    ocv_csv_soc_col = bs.ocv_csv_soc_col
    ocv_csv_v_col = bs.ocv_csv_v_col
    ocv_csv_soc_units = bs.ocv_csv_soc_units
    ocv_csv_soc_convention = bs.ocv_csv_soc_convention
    ocv_csv_step = bs.ocv_csv_step
    ocv_is_cell = bs.ocv_is_cell

    # Optional resistance surface configuration (used by model=:thevenin when provided)
    r0_surface_csv_path = bs.r0_surface_csv_path
    r0_surface_temp_col = bs.r0_surface_temp_col
    r0_surface_soc_col = bs.r0_surface_soc_col
    r0_surface_r0_col = bs.r0_surface_r0_col
    r0_surface_soc_units = bs.r0_surface_soc_units
    r0_surface_soc_convention = bs.r0_surface_soc_convention
    r0_surface_is_cell = bs.r0_surface_is_cell

    surface_from_asset = false
    ocv_from_asset = false

    # Load optional pack asset early so we can also reuse its thermal defaults.
    pack_asset_loaded = nothing

    function _warn_override(field::Symbol, asset_val, toml_val)
        if field in explicit && asset_val !== nothing && toml_val != asset_val
            @warn "Battery $(bs.id): TOML $(field)=$(toml_val) overrides asset value $(asset_val)"
        end
    end

    # Asset resolution:
    # - pack_asset defines series/parallel/capacity and provides cell OCV curve.
    # - cell_asset provides cell OCV curve and (optionally) cell R0 surface.
    if bs.pack_asset !== nothing
        pack = BatteryAssets.load_pack_asset(bs.pack_asset)
        pack_asset_loaded = pack
        if :series in explicit
            _warn_override(:series, pack.series, series)
        else
            series = pack.series
        end
        if :parallel in explicit
            _warn_override(:parallel, pack.parallel, parallel)
        else
            parallel = pack.parallel
        end
        cap =
            pack.qualified_capacity_ah === nothing ? pack.nominal_capacity_ah :
            pack.qualified_capacity_ah
        if cap !== nothing
            if (:capacity_ah in explicit) || (:cell_capacity_ah in explicit)
                if :cell_capacity_ah in explicit && bs.cell_capacity_ah !== nothing
                    toml_cap = bs.cell_capacity_ah * parallel
                    @warn "Battery $(bs.id): TOML cell_capacity_ah=$(bs.cell_capacity_ah) (parallel=$(parallel)) overrides asset capacity_ah=$(cap)"
                elseif :capacity_ah in explicit
                    @warn "Battery $(bs.id): TOML capacity_ah=$(bs.capacity_ah) overrides asset capacity_ah=$(cap)"
                end
            else
                capacity_ah = cap
                cell_capacity_ah = nothing
            end
        end

        cell = pack.cell
        ocv_path_explicit = :ocv_csv_path in explicit
        ocv_inline_explicit = (:ocv_soc in explicit) || (:ocv_v in explicit)
        if ocv_path_explicit
            _warn_override(:ocv_csv_path, cell.ocv_csv, ocv_csv_path)
        elseif ocv_inline_explicit
            @warn "Battery $(bs.id): TOML ocv_soc/ocv_v overrides asset OCV CSV $(cell.ocv_csv)"
        else
            ocv_csv_path = cell.ocv_csv
            ocv_from_asset = true
        end
        if :ocv_csv_soc_col in explicit
            _warn_override(:ocv_csv_soc_col, cell.ocv_soc_col, ocv_csv_soc_col)
        elseif ocv_from_asset
            ocv_csv_soc_col = cell.ocv_soc_col
        end
        if :ocv_csv_v_col in explicit
            _warn_override(:ocv_csv_v_col, cell.ocv_v_col, ocv_csv_v_col)
        elseif ocv_from_asset
            ocv_csv_v_col = cell.ocv_v_col
        end
        if :ocv_csv_soc_units in explicit
            _warn_override(:ocv_csv_soc_units, cell.ocv_soc_units, ocv_csv_soc_units)
        elseif ocv_from_asset
            ocv_csv_soc_units = cell.ocv_soc_units
        end
        if :ocv_csv_soc_convention in explicit
            _warn_override(
                :ocv_csv_soc_convention,
                cell.ocv_soc_convention,
                ocv_csv_soc_convention,
            )
        elseif ocv_from_asset
            ocv_csv_soc_convention = cell.ocv_soc_convention
        end
        if :ocv_is_cell in explicit
            _warn_override(:ocv_is_cell, true, ocv_is_cell)
        else
            ocv_is_cell = true
        end

        if pack.r0_surface_csv !== nothing
            if r0_surface_csv_path === nothing
                r0_surface_csv_path = pack.r0_surface_csv
                surface_from_asset = true
            elseif :r0_surface_csv_path in explicit
                _warn_override(
                    :r0_surface_csv_path,
                    pack.r0_surface_csv,
                    r0_surface_csv_path,
                )
            end
            if surface_from_asset
                r0_surface_temp_col =
                    r0_surface_temp_col === nothing ? pack.r0_temp_col : r0_surface_temp_col
                r0_surface_soc_col =
                    r0_surface_soc_col === nothing ? pack.r0_soc_col : r0_surface_soc_col
                r0_surface_r0_col =
                    r0_surface_r0_col === nothing ? pack.r0_col : r0_surface_r0_col
                r0_surface_soc_units =
                    r0_surface_soc_units === nothing ? pack.r0_soc_units :
                    r0_surface_soc_units
                r0_surface_soc_convention =
                    r0_surface_soc_convention === nothing ? pack.r0_soc_convention :
                    r0_surface_soc_convention
                r0_surface_is_cell =
                    r0_surface_is_cell === nothing ? false : r0_surface_is_cell
                if :r0_surface_temp_col in explicit
                    _warn_override(
                        :r0_surface_temp_col,
                        pack.r0_temp_col,
                        r0_surface_temp_col,
                    )
                end
                if :r0_surface_soc_col in explicit
                    _warn_override(:r0_surface_soc_col, pack.r0_soc_col, r0_surface_soc_col)
                end
                if :r0_surface_r0_col in explicit
                    _warn_override(:r0_surface_r0_col, pack.r0_col, r0_surface_r0_col)
                end
                if :r0_surface_soc_units in explicit
                    _warn_override(
                        :r0_surface_soc_units,
                        pack.r0_soc_units,
                        r0_surface_soc_units,
                    )
                end
                if :r0_surface_soc_convention in explicit
                    _warn_override(
                        :r0_surface_soc_convention,
                        pack.r0_soc_convention,
                        r0_surface_soc_convention,
                    )
                end
                if :r0_surface_is_cell in explicit
                    _warn_override(:r0_surface_is_cell, false, r0_surface_is_cell)
                end
            end
        elseif pack.cell.r0_surface_csv !== nothing
            # Fallback: use the cell resistance surface if the pack does not provide one.
            if r0_surface_csv_path === nothing
                r0_surface_csv_path = pack.cell.r0_surface_csv
                surface_from_asset = true
            elseif :r0_surface_csv_path in explicit
                _warn_override(
                    :r0_surface_csv_path,
                    pack.cell.r0_surface_csv,
                    r0_surface_csv_path,
                )
            end
            if surface_from_asset
                r0_surface_temp_col =
                    r0_surface_temp_col === nothing ? pack.cell.r0_temp_col :
                    r0_surface_temp_col
                r0_surface_soc_col =
                    r0_surface_soc_col === nothing ? pack.cell.r0_soc_col :
                    r0_surface_soc_col
                r0_surface_r0_col =
                    r0_surface_r0_col === nothing ? pack.cell.r0_col : r0_surface_r0_col
                r0_surface_soc_units =
                    r0_surface_soc_units === nothing ? pack.cell.r0_soc_units :
                    r0_surface_soc_units
                r0_surface_soc_convention =
                    r0_surface_soc_convention === nothing ? pack.cell.r0_soc_convention :
                    r0_surface_soc_convention
                r0_surface_is_cell =
                    r0_surface_is_cell === nothing ? true : r0_surface_is_cell
                if :r0_surface_temp_col in explicit
                    _warn_override(
                        :r0_surface_temp_col,
                        pack.cell.r0_temp_col,
                        r0_surface_temp_col,
                    )
                end
                if :r0_surface_soc_col in explicit
                    _warn_override(
                        :r0_surface_soc_col,
                        pack.cell.r0_soc_col,
                        r0_surface_soc_col,
                    )
                end
                if :r0_surface_r0_col in explicit
                    _warn_override(:r0_surface_r0_col, pack.cell.r0_col, r0_surface_r0_col)
                end
                if :r0_surface_soc_units in explicit
                    _warn_override(
                        :r0_surface_soc_units,
                        pack.cell.r0_soc_units,
                        r0_surface_soc_units,
                    )
                end
                if :r0_surface_soc_convention in explicit
                    _warn_override(
                        :r0_surface_soc_convention,
                        pack.cell.r0_soc_convention,
                        r0_surface_soc_convention,
                    )
                end
                if :r0_surface_is_cell in explicit
                    _warn_override(:r0_surface_is_cell, true, r0_surface_is_cell)
                end
            end
        end
    elseif bs.cell_asset !== nothing
        cell = BatteryAssets.load_cell_asset(bs.cell_asset)
        cap_cell =
            cell.qualified_capacity_ah === nothing ? cell.nominal_capacity_ah :
            cell.qualified_capacity_ah
        if cap_cell !== nothing
            if (:capacity_ah in explicit) || (:cell_capacity_ah in explicit)
                if :cell_capacity_ah in explicit && bs.cell_capacity_ah !== nothing
                    toml_cap = bs.cell_capacity_ah * parallel
                    @warn "Battery $(bs.id): TOML cell_capacity_ah=$(bs.cell_capacity_ah) (parallel=$(parallel)) overrides asset capacity_ah=$(cap_cell * parallel)"
                elseif :capacity_ah in explicit
                    @warn "Battery $(bs.id): TOML capacity_ah=$(bs.capacity_ah) overrides asset capacity_ah=$(cap_cell * parallel)"
                end
            else
                capacity_ah = cap_cell * parallel
                cell_capacity_ah = nothing
            end
        end

        ocv_path_explicit = :ocv_csv_path in explicit
        ocv_inline_explicit = (:ocv_soc in explicit) || (:ocv_v in explicit)
        if ocv_path_explicit
            _warn_override(:ocv_csv_path, cell.ocv_csv, ocv_csv_path)
        elseif ocv_inline_explicit
            @warn "Battery $(bs.id): TOML ocv_soc/ocv_v overrides asset OCV CSV $(cell.ocv_csv)"
        else
            ocv_csv_path = cell.ocv_csv
            ocv_from_asset = true
        end
        if :ocv_csv_soc_col in explicit
            _warn_override(:ocv_csv_soc_col, cell.ocv_soc_col, ocv_csv_soc_col)
        elseif ocv_from_asset
            ocv_csv_soc_col = cell.ocv_soc_col
        end
        if :ocv_csv_v_col in explicit
            _warn_override(:ocv_csv_v_col, cell.ocv_v_col, ocv_csv_v_col)
        elseif ocv_from_asset
            ocv_csv_v_col = cell.ocv_v_col
        end
        if :ocv_csv_soc_units in explicit
            _warn_override(:ocv_csv_soc_units, cell.ocv_soc_units, ocv_csv_soc_units)
        elseif ocv_from_asset
            ocv_csv_soc_units = cell.ocv_soc_units
        end
        if :ocv_csv_soc_convention in explicit
            _warn_override(
                :ocv_csv_soc_convention,
                cell.ocv_soc_convention,
                ocv_csv_soc_convention,
            )
        elseif ocv_from_asset
            ocv_csv_soc_convention = cell.ocv_soc_convention
        end
        if :ocv_is_cell in explicit
            _warn_override(:ocv_is_cell, true, ocv_is_cell)
        else
            ocv_is_cell = true
        end

        if cell.r0_surface_csv !== nothing
            if r0_surface_csv_path === nothing
                r0_surface_csv_path = cell.r0_surface_csv
                surface_from_asset = true
            elseif :r0_surface_csv_path in explicit
                _warn_override(
                    :r0_surface_csv_path,
                    cell.r0_surface_csv,
                    r0_surface_csv_path,
                )
            end
            if surface_from_asset
                r0_surface_temp_col =
                    r0_surface_temp_col === nothing ? cell.r0_temp_col : r0_surface_temp_col
                r0_surface_soc_col =
                    r0_surface_soc_col === nothing ? cell.r0_soc_col : r0_surface_soc_col
                r0_surface_r0_col =
                    r0_surface_r0_col === nothing ? cell.r0_col : r0_surface_r0_col
                r0_surface_soc_units =
                    r0_surface_soc_units === nothing ? cell.r0_soc_units :
                    r0_surface_soc_units
                r0_surface_soc_convention =
                    r0_surface_soc_convention === nothing ? cell.r0_soc_convention :
                    r0_surface_soc_convention
                r0_surface_is_cell =
                    r0_surface_is_cell === nothing ? true : r0_surface_is_cell
                if :r0_surface_temp_col in explicit
                    _warn_override(
                        :r0_surface_temp_col,
                        cell.r0_temp_col,
                        r0_surface_temp_col,
                    )
                end
                if :r0_surface_soc_col in explicit
                    _warn_override(:r0_surface_soc_col, cell.r0_soc_col, r0_surface_soc_col)
                end
                if :r0_surface_r0_col in explicit
                    _warn_override(:r0_surface_r0_col, cell.r0_col, r0_surface_r0_col)
                end
                if :r0_surface_soc_units in explicit
                    _warn_override(
                        :r0_surface_soc_units,
                        cell.r0_soc_units,
                        r0_surface_soc_units,
                    )
                end
                if :r0_surface_soc_convention in explicit
                    _warn_override(
                        :r0_surface_soc_convention,
                        cell.r0_soc_convention,
                        r0_surface_soc_convention,
                    )
                end
                if :r0_surface_is_cell in explicit
                    _warn_override(:r0_surface_is_cell, true, r0_surface_is_cell)
                end
            end
        end
    end

    # Finalize capacity from cell_capacity_ah if provided.
    if cell_capacity_ah !== nothing
        capacity_ah = cell_capacity_ah * parallel
    end

    # If a surface is configured, fill any remaining defaults.
    if r0_surface_csv_path !== nothing
        r0_surface_temp_col =
            r0_surface_temp_col === nothing ? "temp_c" : r0_surface_temp_col
        r0_surface_soc_col = r0_surface_soc_col === nothing ? "soc" : r0_surface_soc_col
        r0_surface_r0_col = r0_surface_r0_col === nothing ? "r0_ohm" : r0_surface_r0_col
        r0_surface_soc_units =
            r0_surface_soc_units === nothing ? :fraction : r0_surface_soc_units
        r0_surface_soc_convention =
            r0_surface_soc_convention === nothing ? :remaining : r0_surface_soc_convention
        r0_surface_is_cell = r0_surface_is_cell === nothing ? false : r0_surface_is_cell
    end

    return (
        series = series,
        parallel = parallel,
        capacity_ah = capacity_ah,
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
        pack_asset_loaded = pack_asset_loaded,
    )
end
