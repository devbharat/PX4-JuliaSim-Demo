"""PX4Lockstep.Sim.Powertrain

Battery / powertrain models.

Battery models for sim-side powertrain and PX4 `battery_status` injection.
This module provides a small interface so models can be swapped without touching:
- the sim engine
- the PX4 lockstep bridge

Included models:

- `IdealBattery`: deterministic coulomb counting + constant voltage (baseline)
- `TheveninBattery`: 1st-order Thevenin equivalent (OCV + R0 + optional 1-RC polarization)
  - R0 can be constant or data-driven (e.g., lookup surfaces vs SOC/temp)

The Thevenin model is intentionally lightweight, but it now supports an optional
lumped thermal hook (temperature state integrated in the plant). It is still a
useful starting point for battery-related RTL and energy studies.
"""
module Powertrain

export BatteryStatus,
    AbstractBatteryModel,
    ThermalParams,
    thermal_params,
    OCVTable,
    AbstractR0Model,
    R0Constant,
    R0PiecewiseTemp,
    R0PolynomialTemp,
    R0BilinearSurface,
    R0Sum,
    IdealBattery,
    TheveninBattery,
    step!,
    status

# NOTE
# ----
# In the canonical runtime engine, battery *state* (SOC/V1) is integrated as part of
# `Plant.PlantState`. Battery model objects in this module are therefore intended to
# be **parameter-only**.
#
# For stand-alone stepping (outside the integrated plant), use the
# `BatteryState` helper below and call `step!(model, state, I_bus, dt)`.

############################
# Types
############################

"""Battery status injected into PX4.

Fields match what the lockstep C ABI expects.
"""
Base.@kwdef struct BatteryStatus
    connected::Bool = true
    voltage_v::Float64 = 12.0
    current_a::Float64 = 0.0
    temperature_c::Float64 = 25.0
    remaining::Float64 = 1.0   # [0,1]
    warning::Int32 = 0         # PX4 battery_status warning enum
end

abstract type AbstractBatteryModel end

"""Optional lumped thermal parameters for a battery pack.

This is a deliberately simple, 1-node thermal hook intended to support the future
thermal phase without rewriting the electrical model.

Thermal dynamics are modeled in the plant RHS (not inside the battery model object)
using the following form:

`dT/dt = (P_loss - k*(T - T_amb)) / C_th`

Where:
* `T` is the battery temperature state (°C)
* `T_amb` is a fixed ambient temperature (°C)
* `C_th` is an effective thermal capacitance (J/K)
* `k` is an effective heat transfer coefficient to ambient (W/K)

Notes
-----
- If `enabled=false`, the thermal state (if present) is held constant.
- The simulator currently treats `ambient_temp_c` as a *constant parameter*.
  Coupling to atmosphere/airflow can be added later.
"""
Base.@kwdef struct ThermalParams
    enabled::Bool = false
    c_th_j_per_k::Float64 = Inf
    k_to_ambient_w_per_k::Float64 = 0.0
    ambient_temp_c::Float64 = 25.0
end

"""Return thermal parameters for a battery model (defaults to disabled)."""
@inline thermal_params(::AbstractBatteryModel) = ThermalParams()

"""Open-circuit voltage lookup table.

SOC convention
-------------
`soc` is always **fraction remaining** in `[0,1]` where:

- `0.0` = empty
- `1.0` = full

Tables are assumed monotone in `soc` (increasing) and are interpolated linearly.
"""
struct OCVTable
    soc::Vector{Float64}
    v::Vector{Float64}
end

"""Abstract ohmic resistance model `R0(soc, temp_c)`.

This is deliberately a *typed* model family (structs + numeric tables) instead of
runtime `Function`s so the plant RHS remains deterministic and fast.

All resistance values are in **ohms**.
"""
abstract type AbstractR0Model end

"""Constant ohmic resistance (no SOC/temperature dependence)."""
struct R0Constant <: AbstractR0Model
    r0_ohm::Float64
end

"""Piecewise-linear temperature-dependent ohmic resistance.

This model ignores SOC and interpolates linearly in temperature:

`r0(temp) = interp(temp_pts_c, r0_pts_ohm, temp)`

`scale` is applied multiplicatively (useful for series/parallel scaling).
"""
struct R0PiecewiseTemp <: AbstractR0Model
    temp_pts_c::Vector{Float64}
    r0_pts_ohm::Vector{Float64}
    scale::Float64
end

"""Polynomial temperature-dependent ohmic resistance.

`r0(temp) = scale * (c0 + c1*T + c2*T^2 + ...)`
"""
struct R0PolynomialTemp <: AbstractR0Model
    coeffs::Vector{Float64}
    scale::Float64
end

"""Bilinear SOC/temperature-dependent ohmic resistance surface.

The underlying `BilinearSurface` stores a rectangular grid in (temp, soc).
`scale` is applied multiplicatively (useful for series/parallel scaling).
"""
struct R0BilinearSurface{S} <: AbstractR0Model
    surface::S
    scale::Float64
end

"""Sum of two resistance models.

Useful to represent `(Ns/Np)*R_cell(soc,temp) + R_overhead(temp)`.
"""
struct R0Sum{A<:AbstractR0Model,B<:AbstractR0Model} <: AbstractR0Model
    a::A
    b::B
end

"""Battery state for stand-alone battery stepping.

The canonical multirotor plant integrates battery state (SOC/V1) inside
`Plant.PlantState`, so the battery model objects in this module are parameter-only.

This helper exists for any workflows that still want to step a battery model outside
the integrated plant.
"""
Base.@kwdef mutable struct BatteryState
    soc::Float64 = 1.0
    v1::Float64 = 0.0
    last_current_a::Float64 = 0.0
end

############################
# Helpers
############################

const _ALLOW_DIRECT_CONSTRUCTORS = Ref(false)

@inline function _deprecate_direct_constructor(kind::Symbol)
    _ALLOW_DIRECT_CONSTRUCTORS[] || Base.depwarn(
        "Direct Powertrain.$(kind) construction is deprecated; use Aircraft.BatterySpec/TOML via Aircraft.build_battery.",
        kind,
    )
end

function _with_direct_constructors(f::Function)
    prev = _ALLOW_DIRECT_CONSTRUCTORS[]
    _ALLOW_DIRECT_CONSTRUCTORS[] = true
    try
        return f()
    finally
        _ALLOW_DIRECT_CONSTRUCTORS[] = prev
    end
end

@inline function _warning_from_remaining(
    rem::Float64,
    low::Float64,
    crit::Float64,
    emerg::Float64,
)::Int32
    if rem <= emerg
        return Int32(3)  # EMERGENCY
    elseif rem <= crit
        return Int32(2)  # CRITICAL
    elseif rem <= low
        return Int32(1)  # LOW
    else
        return Int32(0)  # NONE
    end
end

@inline function _clamp01(x::Float64)
    x < 0.0 && return 0.0
    x > 1.0 && return 1.0
    return x
end

@inline _clamp01(x::Real) = _clamp01(Float64(x))

############################
# Ideal battery (baseline)
############################

"""Ideal coulomb-counted battery model.

- SOC integrates from a fixed capacity.
- Voltage is constant (or can be set externally).
- Current draw comes from the modeled bus current.

This is intentionally simple and deterministic.
"""
struct IdealBattery <: AbstractBatteryModel
    capacity_c::Float64          # coulombs
    soc0::Float64                # initial SOC used by `Plant.init_plant_state`
    voltage_v::Float64
    temp_c::Float64
    thermal::ThermalParams
    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
end

function IdealBattery(;
    capacity_ah::Float64 = 5.0,
    soc0::Float64 = 1.0,
    voltage_v::Float64 = 12.0,
    temp_c::Float64 = 25.0,
    thermal::ThermalParams = ThermalParams(),
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
)
    _deprecate_direct_constructor(:IdealBattery)
    capacity_c = capacity_ah * 3600.0
    return IdealBattery(
        capacity_c,
        _clamp01(soc0),
        Float64(voltage_v),
        Float64(temp_c),
        thermal,
        Float64(low_thr),
        Float64(crit_thr),
        Float64(emerg_thr),
    )
end

@inline thermal_params(b::IdealBattery) = b.thermal

"""Construct a battery state from model parameters."""
@inline function battery_state(b::IdealBattery)
    return BatteryState(soc = _clamp01(b.soc0), v1 = 0.0, last_current_a = 0.0)
end

"""Advance the ideal battery state by `dt` seconds given bus current draw."""
function step!(b::IdealBattery, st::BatteryState, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    st.last_current_a = I
    st.soc = _clamp01(st.soc - (I * dt) / b.capacity_c)
    # Ideal battery has no polarization state.
    st.v1 = 0.0
    return nothing
end

function status(b::IdealBattery, st::BatteryState)::BatteryStatus
    rem = _clamp01(st.soc)
    return BatteryStatus(
        connected = true,
        voltage_v = b.voltage_v,
        current_a = st.last_current_a,
        temperature_c = b.temp_c,
        remaining = rem,
        warning = _warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr),
    )
end

############################
# Thevenin battery (OCV + R0 + RC)
############################

"""Piecewise-linear OCV curve helper.

Given two equal-length vectors `(soc_pts, v_pts)`, returns `v(soc)` via clamped linear
interpolation.

Notes
-----
* `soc_pts` must be sorted ascending and contain at least 2 points.
* Uses a binary search (`searchsortedlast`) so dense curves are fine.
"""
function _interp_ocv(soc_pts::Vector{Float64}, v_pts::Vector{Float64}, soc::Float64)
    n = length(soc_pts)
    n == length(v_pts) || throw(ArgumentError("ocv_soc and ocv_v must have same length"))
    n >= 2 || throw(ArgumentError("ocv curve must have at least 2 points"))

    s = _clamp01(soc)

    # Clamp to endpoints.
    if s <= soc_pts[1]
        return v_pts[1]
    elseif s >= soc_pts[end]
        return v_pts[end]
    end

    # Segment index i such that soc_pts[i] <= s < soc_pts[i+1].
    i = searchsortedlast(soc_pts, s)
    @inbounds begin
        s0 = soc_pts[i]
        s1 = soc_pts[i+1]
        v0 = v_pts[i]
        v1 = v_pts[i+1]
        if s1 <= s0
            return v1
        end
        α = (s - s0) / (s1 - s0)
        return (1 - α) * v0 + α * v1
    end
end

"""Normalize an OCV curve into a monotone SOC grid (build-time helper)."""
function normalize_ocv_curve(soc_pts::AbstractVector{<:Real}, v_pts::AbstractVector{<:Real})
    length(soc_pts) == length(v_pts) ||
        throw(ArgumentError("soc_pts and v_pts must have same length"))
    length(soc_pts) >= 2 || throw(ArgumentError("ocv curve must have at least 2 points"))

    soc = [Float64(_clamp01(s)) for s in soc_pts]
    v = [Float64(x) for x in v_pts]

    perm = sortperm(soc)
    soc_s = soc[perm]
    v_s = v[perm]

    soc_u = Float64[]
    v_u = Float64[]
    sizehint!(soc_u, length(soc_s))
    sizehint!(v_u, length(v_s))

    for i = 1:length(soc_s)
        s = soc_s[i]
        vi = v_s[i]
        if !isempty(soc_u) && s == soc_u[end]
            v_u[end] = vi
        else
            push!(soc_u, s)
            push!(v_u, vi)
        end
    end

    length(soc_u) >= 2 ||
        throw(ArgumentError("ocv curve collapsed to <2 unique points after normalization"))

    return soc_u, v_u
end

"""Construct an `OCVTable` from raw SOC/voltage vectors.

This is a build-time convenience that:
- clamps SOC to `[0,1]`
- sorts the SOC axis
- removes duplicate SOC points (keeping the last)
"""
function OCVTable(soc_pts::AbstractVector{<:Real}, v_pts::AbstractVector{<:Real})
    soc_u, v_u = normalize_ocv_curve(soc_pts, v_pts)
    return OCVTable(soc_u, v_u)
end

@inline _interp_ocv(tbl::OCVTable, soc::Float64) = _interp_ocv(tbl.soc, tbl.v, soc)

"""Piecewise-linear interpolation helper for a monotone `x` grid.

The query is clamped to the table endpoints.
"""
function _interp_pwl(x_pts::Vector{Float64}, y_pts::Vector{Float64}, xq::Float64)
    n = length(x_pts)
    n == length(y_pts) || throw(ArgumentError("x_pts and y_pts must have same length"))
    n >= 2 || throw(ArgumentError("pwl table must have at least 2 points"))

    if xq <= x_pts[1]
        return y_pts[1]
    elseif xq >= x_pts[end]
        return y_pts[end]
    end

    i = searchsortedlast(x_pts, xq)
    @inbounds begin
        x0 = x_pts[i]
        x1 = x_pts[i+1]
        y0 = y_pts[i]
        y1 = y_pts[i+1]
        if x1 <= x0
            return y1
        end
        α = (xq - x0) / (x1 - x0)
        return (1 - α) * y0 + α * y1
    end
end

"""Load an OCV curve from a CSV file (header + numeric rows)."""
function load_ocv_curve_csv(
    path::AbstractString;
    soc_col::AbstractString = "soc",
    v_col::AbstractString = "ocv_v",
    soc_units::Symbol = :fraction,
    soc_convention::Symbol = :remaining,
    step::Union{Nothing,Float64} = nothing,
)
    soc_units_n =
        soc_units in (:fraction, :frac) ? :fraction :
        soc_units in (:percent, :pct, :percentage) ? :percent :
        throw(
            ArgumentError("soc_units must be one of :fraction|:percent (got $(soc_units))"),
        )
    soc_convention_n =
        soc_convention in (:remaining, :soc, :soc_remaining) ? :remaining :
        soc_convention in (:used, :dod, :depth_of_discharge, :used_soc) ? :used :
        throw(
            ArgumentError(
                "soc_convention must be one of :remaining|:used|:dod (got $(soc_convention))",
            ),
        )
    if step !== nothing
        (step > 0.0 && step <= 1.0) ||
            throw(ArgumentError("step must be in (0,1] when provided (got $(step))"))
    end

    soc_raw = Float64[]
    v_raw = Float64[]

    open(String(path), "r") do io
        eof(io) && throw(ArgumentError("empty OCV CSV: $(path)"))
        hdr = split(chomp(readline(io)), ',')
        idx_soc = findfirst(==(String(soc_col)), hdr)
        idx_v = findfirst(==(String(v_col)), hdr)
        idx_soc === nothing &&
            throw(ArgumentError("SOC column '$(soc_col)' not found in CSV header"))
        idx_v === nothing &&
            throw(ArgumentError("voltage column '$(v_col)' not found in CSV header"))

        for (ln0, line) in enumerate(eachline(io))
            line_s = strip(line)
            isempty(line_s) && continue
            cols = split(line_s, ',')
            length(cols) >= max(idx_soc, idx_v) || throw(
                ArgumentError(
                    "CSV row $(ln0+1) has too few columns (expected >= $(max(idx_soc, idx_v)))",
                ),
            )

            s = parse(Float64, strip(cols[idx_soc]))
            v = parse(Float64, strip(cols[idx_v]))

            if soc_units_n === :percent
                s /= 100.0
            end
            if soc_convention_n === :used
                s = 1.0 - s
            end

            push!(soc_raw, _clamp01(s))
            push!(v_raw, v)
        end
    end

    length(soc_raw) >= 2 || throw(ArgumentError("OCV CSV contained < 2 data rows"))

    soc_u, v_u = normalize_ocv_curve(soc_raw, v_raw)

    if step === nothing
        return soc_u, v_u
    end

    soc_grid = collect(0.0:step:1.0)
    if isempty(soc_grid) || soc_grid[1] != 0.0
        soc_grid = vcat([0.0], soc_grid)
    end
    if soc_grid[end] < 1.0 - 1e-12
        push!(soc_grid, 1.0)
    elseif soc_grid[end] > 1.0
        soc_grid[end] = 1.0
    end

    v_grid = [_interp_ocv(soc_u, v_u, soc_grid[i]) for i = 1:length(soc_grid)]
    return soc_grid, v_grid
end

############################
# 2D lookup surfaces (temp/SOC tables)
############################

"""Rectilinear 2D surface for bilinear interpolation.

`z[i,j]` corresponds to `(x[i], y[j])`.

This is intended for build-time-loaded tables like DCIR/Ohmic resistance
parameterized by (temperature, SOC).
"""
struct BilinearSurface
    x::Vector{Float64}          # e.g., temperature grid (ascending)
    y::Vector{Float64}          # e.g., SOC grid (ascending)
    z::Matrix{Float64}          # size (length(x), length(y))
end

@inline function _interp_bilinear(surf::BilinearSurface, xq::Float64, yq::Float64)::Float64
    xs = surf.x
    ys = surf.y
    z = surf.z

    nx = length(xs)
    ny = length(ys)
    nx >= 2 || throw(ArgumentError("surface x grid must have at least 2 points"))
    ny >= 2 || throw(ArgumentError("surface y grid must have at least 2 points"))

    # Clamp query to bounds.
    xqc = xq < xs[1] ? xs[1] : (xq > xs[end] ? xs[end] : xq)
    yqc = yq < ys[1] ? ys[1] : (yq > ys[end] ? ys[end] : yq)

    # Find bracketing indices.
    i = searchsortedlast(xs, xqc)
    i = i < 1 ? 1 : (i >= nx ? nx - 1 : i)
    j = searchsortedlast(ys, yqc)
    j = j < 1 ? 1 : (j >= ny ? ny - 1 : j)

    @inbounds begin
        x0 = xs[i]
        x1 = xs[i+1]
        y0 = ys[j]
        y1 = ys[j+1]

        z00 = z[i, j]
        z10 = z[i+1, j]
        z01 = z[i, j+1]
        z11 = z[i+1, j+1]

        # If we have missing values, fall back to nearest neighbor over the full grid.
        if !(isfinite(z00) && isfinite(z10) && isfinite(z01) && isfinite(z11))
            best = NaN
            best_d2 = Inf
            for ii = 1:nx, jj = 1:ny
                zij = z[ii, jj]
                isfinite(zij) || continue
                dx = xs[ii] - xqc
                dy = ys[jj] - yqc
                d2 = dx * dx + dy * dy
                if d2 < best_d2
                    best_d2 = d2
                    best = zij
                end
            end
            isfinite(best) || throw(ArgumentError("surface contains no finite values"))
            return best
        end

        α = x1 <= x0 ? 0.0 : (xqc - x0) / (x1 - x0)
        β = y1 <= y0 ? 0.0 : (yqc - y0) / (y1 - y0)

        # Bilinear interpolation.
        z0 = (1 - α) * z00 + α * z10
        z1 = (1 - α) * z01 + α * z11
        return (1 - β) * z0 + β * z1
    end
end

############################
# R0 model evaluation
############################

"""Evaluate the ohmic resistance model `R0(soc, temp_c)`.

All models are clamped to be non-negative.
"""
@inline function r0_ohm(m::R0Constant, _soc::Float64, _temp_c::Float64)::Float64
    return max(0.0, Float64(m.r0_ohm))
end

@inline function r0_ohm(m::R0PiecewiseTemp, _soc::Float64, temp_c::Float64)::Float64
    r = _interp_pwl(m.temp_pts_c, m.r0_pts_ohm, Float64(temp_c))
    return max(0.0, m.scale * r)
end

@inline function r0_ohm(m::R0PolynomialTemp, _soc::Float64, temp_c::Float64)::Float64
    T = Float64(temp_c)
    # Horner's rule.
    acc = 0.0
    for i = length(m.coeffs):-1:1
        acc = acc * T + m.coeffs[i]
    end
    return max(0.0, m.scale * acc)
end

@inline function r0_ohm(m::R0BilinearSurface, soc::Float64, temp_c::Float64)::Float64
    r = _interp_bilinear(m.surface, Float64(temp_c), _clamp01(soc))
    return max(0.0, m.scale * r)
end

@inline function r0_ohm(m::R0Sum, soc::Float64, temp_c::Float64)::Float64
    return r0_ohm(m.a, soc, temp_c) + r0_ohm(m.b, soc, temp_c)
end

"""Load a bilinear surface from a CSV file (header + numeric rows).

The CSV is expected to contain at least three columns: `x_col`, `y_col`, `z_col`.

`y_units`/`y_convention` mirror the OCV CSV conventions so SOC axes can be expressed
as either percent or fraction, and as either SOC remaining or "used" (DoD).
"""
function load_bilinear_surface_csv(
    path::AbstractString;
    x_col::AbstractString = "temp_c",
    y_col::AbstractString = "soc",
    z_col::AbstractString = "r0_ohm",
    y_units::Symbol = :fraction,
    y_convention::Symbol = :remaining,
)
    y_units_n =
        y_units in (:fraction, :frac) ? :fraction :
        y_units in (:percent, :pct, :percentage) ? :percent :
        throw(ArgumentError("y_units must be one of :fraction|:percent (got $(y_units))"))
    y_convention_n =
        y_convention in (:remaining, :soc, :soc_remaining) ? :remaining :
        y_convention in (:used, :dod, :depth_of_discharge, :used_soc) ? :used :
        throw(
            ArgumentError(
                "y_convention must be one of :remaining|:used|:dod (got $(y_convention))",
            ),
        )

    xs_raw = Float64[]
    ys_raw = Float64[]
    acc = Dict{Tuple{Float64,Float64},Tuple{Float64,Int}}()

    open(String(path), "r") do io
        eof(io) && throw(ArgumentError("empty surface CSV: $(path)"))
        hdr = split(chomp(readline(io)), ',')
        idx_x = findfirst(==(String(x_col)), hdr)
        idx_y = findfirst(==(String(y_col)), hdr)
        idx_z = findfirst(==(String(z_col)), hdr)
        idx_x === nothing &&
            throw(ArgumentError("x column '$(x_col)' not found in CSV header"))
        idx_y === nothing &&
            throw(ArgumentError("y column '$(y_col)' not found in CSV header"))
        idx_z === nothing &&
            throw(ArgumentError("z column '$(z_col)' not found in CSV header"))

        for (ln0, line) in enumerate(eachline(io))
            line_s = strip(line)
            isempty(line_s) && continue
            cols = split(line_s, ',')
            length(cols) >= max(idx_x, idx_y, idx_z) || throw(
                ArgumentError(
                    "CSV row $(ln0+1) has too few columns (expected >= $(max(idx_x, idx_y, idx_z)))",
                ),
            )

            x = parse(Float64, strip(cols[idx_x]))
            y = parse(Float64, strip(cols[idx_y]))
            z = parse(Float64, strip(cols[idx_z]))

            if y_units_n === :percent
                y /= 100.0
            end
            if y_convention_n === :used
                y = 1.0 - y
            end
            y = _clamp01(y)

            push!(xs_raw, x)
            push!(ys_raw, y)

            key = (x, y)
            if haskey(acc, key)
                s, n = acc[key]
                acc[key] = (s + z, n + 1)
            else
                acc[key] = (z, 1)
            end
        end
    end

    length(xs_raw) >= 2 || throw(ArgumentError("surface CSV contained < 2 data rows"))

    xs = sort!(unique(xs_raw))
    ys = sort!(unique(ys_raw))
    nx = length(xs)
    ny = length(ys)
    nx >= 2 || throw(ArgumentError("surface x grid collapsed to <2 unique values"))
    ny >= 2 || throw(ArgumentError("surface y grid collapsed to <2 unique values"))

    z_mat = fill(NaN, nx, ny)
    for ((x, y), (s, n)) in acc
        i = searchsortedfirst(xs, x)
        j = searchsortedfirst(ys, y)
        if i <= nx && j <= ny && xs[i] == x && ys[j] == y
            z_mat[i, j] = s / n
        end
    end

    return BilinearSurface(xs, ys, z_mat)
end

"""1st-order Thevenin equivalent battery model.

Terminal voltage:
  `V = OCV(SOC) - I*R0(SOC,T) - V1`

Polarization voltage state (optional RC branch):
  `dV1/dt = -(1/(R1*C1)) * V1 + (1/C1) * I`

SOC:
  `dSOC/dt = -I / Q`

Notes
-----
- `R0` is modeled by an `AbstractR0Model` and can be constant or data-driven.
- If the thermal hook is disabled, `temp_c` is treated as a constant parameter.
"""
struct TheveninBattery{R0M<:AbstractR0Model} <: AbstractBatteryModel
    capacity_c::Float64
    soc0::Float64                # initial SOC used by `Plant.init_plant_state`

    ocv::OCVTable
    r0_model::R0M
    temp_c::Float64
    thermal::ThermalParams

    r1::Float64
    c1::Float64
    v1_0::Float64                # initial polarization voltage used by `Plant.init_plant_state`

    min_voltage_v::Float64

    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
end

function TheveninBattery(;
    capacity_ah::Float64 = 5.0,
    soc0::Float64 = 1.0,
    # Default ~3S LiPo-ish open circuit curve (very rough).
    ocv_soc::Vector{Float64} = [0.0, 0.1, 0.5, 0.9, 1.0],
    ocv_v::Vector{Float64} = [9.0, 10.8, 11.4, 12.3, 12.6],
    # Pre-built OCV table (used by Aircraft.build_battery).
    ocv::Union{Nothing,OCVTable} = nothing,
    # Back-compat: constant R0 if no explicit model is provided.
    r0::Float64 = 0.02,
    r0_model::Union{Nothing,AbstractR0Model} = nothing,
    temp_c::Float64 = 25.0,
    thermal::ThermalParams = ThermalParams(),
    r1::Float64 = 0.01,
    c1::Float64 = 2000.0,
    v1_0::Float64 = 0.0,
    min_voltage_v::Float64 = 0.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
)
    _deprecate_direct_constructor(:TheveninBattery)
    capacity_c = capacity_ah * 3600.0
    ocv_tbl = ocv === nothing ? OCVTable(ocv_soc, ocv_v) : ocv
    r0m = r0_model === nothing ? R0Constant(Float64(r0)) : r0_model
    return TheveninBattery{typeof(r0m)}(
        capacity_c,
        _clamp01(soc0),
        ocv_tbl,
        r0m,
        Float64(temp_c),
        thermal,
        Float64(r1),
        Float64(c1),
        Float64(v1_0),
        Float64(min_voltage_v),
        Float64(low_thr),
        Float64(crit_thr),
        Float64(emerg_thr),
    )
end

@inline thermal_params(b::TheveninBattery) = b.thermal

"""Low-level constructor for pre-built parameter objects.

This is what `Aircraft.build_battery` uses after loading OCV tables and resistance
surfaces at build time.
"""
function _thevenin_from_params(;
    capacity_ah::Float64,
    soc0::Float64,
    ocv::OCVTable,
    r0_model::AbstractR0Model,
    temp_c::Float64 = 25.0,
    thermal::ThermalParams = ThermalParams(),
    r1::Float64 = 0.0,
    c1::Float64 = 0.0,
    v1_0::Float64 = 0.0,
    min_voltage_v::Float64 = 0.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
)
    capacity_c = capacity_ah * 3600.0
    r0m = r0_model
    return TheveninBattery{typeof(r0m)}(
        capacity_c,
        _clamp01(soc0),
        ocv,
        r0m,
        Float64(temp_c),
        thermal,
        Float64(r1),
        Float64(c1),
        Float64(v1_0),
        Float64(min_voltage_v),
        Float64(low_thr),
        Float64(crit_thr),
        Float64(emerg_thr),
    )
end

"""Construct a battery state from model parameters."""
@inline function battery_state(b::TheveninBattery)
    return BatteryState(soc = _clamp01(b.soc0), v1 = Float64(b.v1_0), last_current_a = 0.0)
end

function step!(b::TheveninBattery, st::BatteryState, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    st.last_current_a = I

    # SOC coulomb counting.
    st.soc = _clamp01(st.soc - (I * dt) / b.capacity_c)

    # Polarization voltage dynamics.
    if b.r1 > 0.0 && b.c1 > 0.0
        τ = b.r1 * b.c1
        if τ < 1e-9
            st.v1 = I * b.r1
        else
            α = exp(-dt / τ)
            st.v1 = st.v1 * α + I * b.r1 * (1.0 - α)
        end
    else
        st.v1 = 0.0
    end

    return nothing
end

function status(b::TheveninBattery, st::BatteryState)::BatteryStatus
    rem = _clamp01(st.soc)
    ocv = _interp_ocv(b.ocv, rem)
    r0 = r0_ohm(b.r0_model, rem, b.temp_c)
    V = ocv - st.last_current_a * r0 - st.v1
    V = max(b.min_voltage_v, V)
    return BatteryStatus(
        connected = true,
        voltage_v = V,
        current_a = st.last_current_a,
        temperature_c = b.temp_c,
        remaining = rem,
        warning = _warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr),
    )
end

end # module Powertrain
