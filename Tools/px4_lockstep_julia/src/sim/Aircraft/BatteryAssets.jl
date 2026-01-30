"""Battery asset loader.

This is a lightweight helper that lets TOML specs reference cell/pack assets
via explicit `meta.toml` paths.

Goals:
* Keep the simulation core battery model independent of where data comes from.
* Standardize the on-disk schema so new cells/packs can be added without code changes.
* Resolve asset references at build-time (never inside the plant RHS).

This file intentionally only covers the *data plumbing* part:
- parsing `meta.toml`
- resolving relative CSV paths

The actual numeric use of curves/surfaces is implemented in `Sim.Powertrain`.
"""

module BatteryAssets

using TOML


############################
# Types
############################

Base.@kwdef struct CellAsset
    id::String
    manufacturer::Union{Nothing,String} = nothing
    chemistry::Union{Nothing,String} = nothing
    description::Union{Nothing,String} = nothing

    nominal_capacity_ah::Union{Nothing,Float64} = nothing
    qualified_capacity_ah::Union{Nothing,Float64} = nothing

    # OCV curve CSV (cell-level)
    ocv_csv::String
    ocv_soc_col::String = "soc"
    ocv_v_col::String = "ocv_v"
    ocv_soc_units::Symbol = :fraction          # :fraction|:percent
    ocv_soc_convention::Symbol = :remaining    # :remaining|:used

    # Optional resistance surface (cell-level)
    r0_surface_csv::Union{Nothing,String} = nothing
    r0_temp_col::String = "temp_c"
    r0_soc_col::String = "soc"
    r0_col::String = "r0_ohm"
    r0_soc_units::Symbol = :fraction
    r0_soc_convention::Symbol = :remaining
end

Base.@kwdef struct PackAsset
    id::String
    manufacturer::Union{Nothing,String} = nothing
    description::Union{Nothing,String} = nothing

    series::Int
    parallel::Int

    nominal_capacity_ah::Union{Nothing,Float64} = nothing
    qualified_capacity_ah::Union{Nothing,Float64} = nothing

    cell::CellAsset

    # Optional resistance surface (pack-level)
    r0_surface_csv::Union{Nothing,String} = nothing
    r0_temp_col::String = "temp_c"
    r0_soc_col::String = "soc"
    r0_col::String = "r0_ohm"
    r0_soc_units::Symbol = :fraction
    r0_soc_convention::Symbol = :remaining

    # Optional simple thermal parameters (pack-level).
    thermal_c_th_j_per_k::Union{Nothing,Float64} = nothing
    thermal_k_to_ambient_w_per_k::Union{Nothing,Float64} = nothing
end


############################
# Helpers
############################

@inline function _normsym(x)::Symbol
    x isa Symbol && return x
    x isa AbstractString && return Symbol(lowercase(strip(String(x))))
    error("expected symbol or string, got $(typeof(x))")
end

@inline function _norm_soc_units(x)::Symbol
    s = _normsym(x)
    return s in (:fraction, :frac) ? :fraction :
           s in (:percent, :pct, :percentage) ? :percent :
           error("soc_units must be fraction|percent (got $(s))")
end

@inline function _norm_soc_convention(x)::Symbol
    s = _normsym(x)
    return s in (:remaining, :soc, :soc_remaining) ? :remaining :
           s in (:used, :dod, :depth_of_discharge, :used_soc) ? :used :
           error("soc_convention must be remaining|used (got $(s))")
end

@inline function _resolve_rel(base_dir::AbstractString, rel::AbstractString)
    s = Base.expanduser(String(rel))
    return normpath(isabspath(s) ? s : joinpath(String(base_dir), s))
end

@inline function _get_str(cfg::AbstractDict, key::AbstractString; default = nothing)
    v = get(cfg, key, default)
    v === nothing && return nothing
    v isa AbstractString || error("Expected '$key' to be a string")
    return String(v)
end

@inline function _get_f64(cfg::AbstractDict, key::AbstractString; default = nothing)
    v = get(cfg, key, default)
    v === nothing && return nothing
    v isa Real || error("Expected '$key' to be a number")
    return Float64(v)
end

@inline function _get_int(cfg::AbstractDict, key::AbstractString; default = nothing)
    v = get(cfg, key, default)
    v === nothing && return nothing
    v isa Integer || error("Expected '$key' to be an integer")
    return Int(v)
end


############################
# Loaders
############################

function load_cell_asset_from_meta(meta_path::AbstractString)::CellAsset
    meta_path = String(meta_path)
    cfg = TOML.parsefile(meta_path)
    base_dir = dirname(meta_path)

    id = _get_str(cfg, "id")
    id === nothing && error("Cell meta missing 'id': $(meta_path)")

    ocv_tbl = get(cfg, "ocv_curve", nothing)
    ocv_tbl isa AbstractDict || error("Cell meta missing [ocv_curve] table: $(meta_path)")
    ocv_csv_rel = _get_str(ocv_tbl, "csv")
    ocv_csv_rel === nothing && error("Cell meta [ocv_curve] missing 'csv': $(meta_path)")
    ocv_csv = _resolve_rel(base_dir, ocv_csv_rel)

    soc_col = _get_str(ocv_tbl, "soc_col"; default = "soc")
    v_col = _get_str(ocv_tbl, "v_col"; default = "ocv_v")
    soc_units = _norm_soc_units(get(ocv_tbl, "soc_units", :fraction))
    soc_conv = _norm_soc_convention(get(ocv_tbl, "soc_convention", :remaining))

    # Optional resistance surface
    r0_tbl = get(cfg, "resistance_surface_cell", nothing)
    r0_csv = nothing
    r0_temp_col = "temp_c"
    r0_soc_col = "soc"
    r0_col = "r0_ohm"
    r0_soc_units = :fraction
    r0_soc_conv = :remaining
    if r0_tbl !== nothing
        r0_tbl isa AbstractDict ||
            error("Cell meta [resistance_surface_cell] must be a table")
        r0_csv_rel = _get_str(r0_tbl, "csv")
        r0_csv_rel === nothing &&
            error("Cell meta [resistance_surface_cell] missing 'csv': $(meta_path)")
        r0_csv = _resolve_rel(base_dir, r0_csv_rel)
        r0_temp_col = _get_str(r0_tbl, "temp_col"; default = r0_temp_col)
        r0_soc_col = _get_str(r0_tbl, "soc_col"; default = r0_soc_col)
        r0_col = _get_str(r0_tbl, "r0_col"; default = r0_col)
        r0_soc_units = _norm_soc_units(get(r0_tbl, "soc_units", r0_soc_units))
        r0_soc_conv = _norm_soc_convention(get(r0_tbl, "soc_convention", r0_soc_conv))
    end

    return CellAsset(
        id = id,
        manufacturer = _get_str(cfg, "manufacturer"),
        chemistry = _get_str(cfg, "chemistry"),
        description = _get_str(cfg, "description"),
        nominal_capacity_ah = _get_f64(cfg, "nominal_capacity_ah"),
        qualified_capacity_ah = _get_f64(cfg, "qualified_capacity_ah"),
        ocv_csv = ocv_csv,
        ocv_soc_col = soc_col,
        ocv_v_col = v_col,
        ocv_soc_units = soc_units,
        ocv_soc_convention = soc_conv,
        r0_surface_csv = r0_csv,
        r0_temp_col = r0_temp_col,
        r0_soc_col = r0_soc_col,
        r0_col = r0_col,
        r0_soc_units = r0_soc_units,
        r0_soc_convention = r0_soc_conv,
    )
end

"""Load a cell asset from a `meta.toml` path."""
function load_cell_asset(meta_path::AbstractString)::CellAsset
    s = String(meta_path)
    endswith(lowercase(s), ".toml") ||
        error("Cell asset must be a path to meta.toml (got: $(s))")
    isfile(s) || error("Cell asset meta.toml not found: $(s)")
    return load_cell_asset_from_meta(s)
end


function load_pack_asset_from_meta(meta_path::AbstractString)::PackAsset
    meta_path = String(meta_path)
    cfg = TOML.parsefile(meta_path)
    base_dir = dirname(meta_path)

    id = _get_str(cfg, "id")
    id === nothing && error("Pack meta missing 'id': $(meta_path)")
    series = _get_int(cfg, "series")
    parallel = _get_int(cfg, "parallel")
    (series !== nothing && series >= 1) ||
        error("Pack meta missing/invalid 'series': $(meta_path)")
    (parallel !== nothing && parallel >= 1) ||
        error("Pack meta missing/invalid 'parallel': $(meta_path)")

    cell_tbl = get(cfg, "cell", nothing)
    cell_tbl isa AbstractDict || error("Pack meta missing [cell] table: $(meta_path)")
    cell_ref = _get_str(cell_tbl, "ref")
    cell_ref === nothing && error("Pack meta [cell] missing 'ref': $(meta_path)")
    cell_meta = _resolve_rel(base_dir, cell_ref)
    isfile(cell_meta) ||
        error("Referenced cell meta.toml not found: $(cell_meta) (from $(meta_path))")
    cell = load_cell_asset_from_meta(cell_meta)

    # Optional resistance surface
    r0_tbl = get(cfg, "resistance_surface_pack", nothing)
    r0_csv = nothing
    r0_temp_col = "temp_c"
    r0_soc_col = "soc"
    r0_col = "r0_ohm"
    r0_soc_units = :fraction
    r0_soc_conv = :remaining
    if r0_tbl !== nothing
        r0_tbl isa AbstractDict ||
            error("Pack meta [resistance_surface_pack] must be a table")
        r0_csv_rel = _get_str(r0_tbl, "csv")
        r0_csv_rel === nothing &&
            error("Pack meta [resistance_surface_pack] missing 'csv': $(meta_path)")
        r0_csv = _resolve_rel(base_dir, r0_csv_rel)
        r0_temp_col = _get_str(r0_tbl, "temp_col"; default = r0_temp_col)
        r0_soc_col = _get_str(r0_tbl, "soc_col"; default = r0_soc_col)
        r0_col = _get_str(r0_tbl, "r0_col"; default = r0_col)
        r0_soc_units = _norm_soc_units(get(r0_tbl, "soc_units", r0_soc_units))
        r0_soc_conv = _norm_soc_convention(get(r0_tbl, "soc_convention", r0_soc_conv))
    end

    # Optional thermal parameters
    thermal_tbl = get(cfg, "thermal", nothing)
    thermal_c_th_j_per_k = nothing
    thermal_k_to_ambient_w_per_k = nothing
    if thermal_tbl !== nothing
        thermal_tbl isa AbstractDict || error("Pack meta [thermal] must be a table")
        thermal_c_th_j_per_k = _get_f64(thermal_tbl, "c_th_j_per_k")
        thermal_k_to_ambient_w_per_k = _get_f64(thermal_tbl, "k_to_ambient_w_per_k")
    end

    return PackAsset(
        id = id,
        manufacturer = _get_str(cfg, "manufacturer"),
        description = _get_str(cfg, "description"),
        series = series,
        parallel = parallel,
        nominal_capacity_ah = _get_f64(cfg, "nominal_capacity_ah"),
        qualified_capacity_ah = _get_f64(cfg, "qualified_capacity_ah"),
        cell = cell,
        r0_surface_csv = r0_csv,
        r0_temp_col = r0_temp_col,
        r0_soc_col = r0_soc_col,
        r0_col = r0_col,
        r0_soc_units = r0_soc_units,
        r0_soc_convention = r0_soc_conv,
        thermal_c_th_j_per_k = thermal_c_th_j_per_k,
        thermal_k_to_ambient_w_per_k = thermal_k_to_ambient_w_per_k,
    )
end

"""Load a pack asset from a `meta.toml` path."""
function load_pack_asset(meta_path::AbstractString)::PackAsset
    s = String(meta_path)
    endswith(lowercase(s), ".toml") ||
        error("Pack asset must be a path to meta.toml (got: $(s))")
    isfile(s) || error("Pack asset meta.toml not found: $(s)")
    return load_pack_asset_from_meta(s)
end

export CellAsset, PackAsset
export load_cell_asset, load_pack_asset

end # module BatteryAssets
