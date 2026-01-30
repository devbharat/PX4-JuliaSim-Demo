"""Spec → Engine builder.

This file is the implementation of the **aircraft composition layer**:

* It consumes an `AircraftSpec` (instances + connections + integration config)
* It produces the fully wired simulation inputs:
  - `timeline`
  - `plant0`
  - `dynfun`
  - `integrator`
  - runtime sources (autopilot/wind/scenario/estimator)

The builder applies spec-driven PX4 parameters for allocator geometry (CA_*),
enabling multi-rotor layouts to drive PX4 directly.
"""

using Random
using StaticArrays

using ..Runtime
using ..Recording
using ..Sources

using ..Plant
using ..Types
using ..RigidBody: RigidBodyState
using ..Vehicles
using ..Propulsion
using ..Powertrain
using ..PlantModels
using ..Environment
using ..Scenario
using ..Estimators

using ..Autopilots
using ..Integrators
using ..Contacts

using PX4Lockstep:
    param_set!, param_notify!, param_get, param_preinit_set!, control_alloc_update_params!

const _SIM = parentmodule(@__MODULE__)


"""Internal build output.

This is intentionally *not* exported yet. It is the structured result of
`build_aircraft_instance(...)` and is consumed by `build_engine(...)`.
"""
struct AircraftInstance{TL,PS,D,INT,S}
    timeline::TL
    plant0::PS
    dynfun::D
    integrator::INT
    sources::S
    meta::Dict{Symbol,Any}
end


# -----------------
# Helper builders
# -----------------

@inline function _integrator_from_symbol(name::Symbol)
    if name === :Euler
        return Integrators.EulerIntegrator()
    elseif name === :RK4
        return Integrators.RK4Integrator()
    elseif name === :RK23
        return Integrators.RK23Integrator()
    elseif name === :RK45
        return Integrators.RK45Integrator()
    else
        throw(
            ArgumentError(
                "Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)",
            ),
        )
    end
end

@inline function _resolve_integrator(spec::AircraftSpec)
    integ = spec.plant.integrator
    if integ isa Symbol
        return _integrator_from_symbol(integ)
    end
    return integ
end

function _build_atmosphere(env::EnvironmentSpec)
    env.atmosphere === :isa1976 ||
        throw(ArgumentError("Unsupported atmosphere=$(env.atmosphere)"))
    return Environment.ISA1976()
end

function _build_gravity(env::EnvironmentSpec)
    if env.gravity === :uniform
        return Environment.UniformGravity(env.gravity_mps2)
    elseif env.gravity === :spherical
        return Environment.SphericalGravity(env.gravity_mu, env.gravity_r0_m)
    end
    throw(ArgumentError("Unsupported gravity=$(env.gravity)"))
end

function _build_wind(env::EnvironmentSpec)
    if env.wind === :none
        return Environment.NoWind()
    elseif env.wind === :ou
        return Environment.OUWind(
            mean = env.wind_mean_ned,
            σ = env.wind_sigma_ned,
            τ_s = env.wind_tau_s,
        )
    elseif env.wind === :constant
        return Environment.ConstantWind(env.wind_mean_ned)
    end
    throw(ArgumentError("Unsupported wind=$(env.wind)"))
end

function _build_env_live(spec::AircraftSpec)
    envspec = spec.environment
    return Environment.EnvironmentModel(
        atmosphere = _build_atmosphere(envspec),
        wind = _build_wind(envspec),
        gravity = _build_gravity(envspec),
        origin = spec.home,
    )
end

function _build_env_replay(spec::AircraftSpec)
    envspec = spec.environment
    return Environment.EnvironmentModel(
        atmosphere = _build_atmosphere(envspec),
        wind = Environment.NoWind(),
        gravity = _build_gravity(envspec),
        origin = spec.home,
    )
end

function _build_scenario(spec::AircraftSpec)
    s = Scenario.EventScenario()
    Scenario.arm_at!(s, spec.scenario.arm_time_s)
    Scenario.mission_start_at!(s, spec.scenario.mission_time_s)
    return s
end

function _build_estimator(spec::AircraftSpec)
    est = spec.estimator
    if est.kind === :none
        return nothing
    end
    base = Estimators.NoisyEstimator(
        pos_sigma_m = est.pos_sigma_m,
        vel_sigma_mps = est.vel_sigma_mps,
        yaw_sigma_rad = est.yaw_sigma_rad,
        rate_sigma_rad_s = est.rate_sigma_rad_s,
        bias_tau_s = est.bias_tau_s,
        rate_bias_sigma_rad_s = est.rate_bias_sigma_rad_s,
    )
    delay_s = est.delay_s === nothing ? 2 * spec.timeline.dt_autopilot_s : est.delay_s
    dt_est = est.dt_est_s === nothing ? spec.timeline.dt_autopilot_s : est.dt_est_s
    return Estimators.DelayedEstimator(base; delay_s = delay_s, dt_est = dt_est)
end

function _build_default_timeline(;
    t_end_s::Float64,
    dt_autopilot_s::Float64,
    dt_wind_s::Float64,
    dt_log_s::Float64,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    scenario_source = nothing,
)
    t0_us = UInt64(0)
    t_end_us = Runtime.dt_to_us(t_end_s)
    dt_ap_us = Runtime.dt_to_us(dt_autopilot_s)
    dt_wind_us = Runtime.dt_to_us(dt_wind_s)
    dt_log_us = Runtime.dt_to_us(dt_log_s)
    dt_phys_us = dt_phys_s === nothing ? nothing : Runtime.dt_to_us(dt_phys_s)

    return Runtime.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scenario = scenario_source,
    )
end

function _build_actuator_model(mspec::AbstractActuatorModelSpec, N::Int)
    if mspec isa DirectActuatorSpec
        return Vehicles.DirectActuators()
    elseif mspec isa FirstOrderActuatorSpec
        return Vehicles.FirstOrderActuators{N}(τ = mspec.τ)
    elseif mspec isa SecondOrderActuatorSpec
        return Vehicles.SecondOrderActuators{N}(
            ωn = mspec.ωn,
            ζ = mspec.ζ,
            rate_limit = mspec.rate_limit,
        )
    else
        throw(ArgumentError("Unknown actuator model spec: $(typeof(mspec))"))
    end
end

function _select_battery_spec(spec::AircraftSpec, id::BatteryId)
    for b in spec.power.batteries
        b.id == id && return b
    end
    throw(ArgumentError("Battery id=$(id) not found in spec.power.batteries"))
end

function _build_battery(bs::BatterySpec)
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

    # -----------------------------
    # Thermal hook
    # -----------------------------
    # Temperature is a plant state. If thermal is disabled, its derivative is held at 0
    # and the initial value acts as a constant.
    temp_init =
        bs.thermal.initial_temp_c === nothing ? bs.temp_c : bs.thermal.initial_temp_c
    temp_amb = bs.thermal.ambient_temp_c === nothing ? bs.temp_c : bs.thermal.ambient_temp_c

    c_th_default =
        pack_asset_loaded === nothing ? nothing : pack_asset_loaded.thermal_c_th_j_per_k
    k_default =
        pack_asset_loaded === nothing ? nothing :
        pack_asset_loaded.thermal_k_to_ambient_w_per_k
    if pack_asset_loaded !== nothing
        if :c_th_j_per_k in bs.thermal.explicit_keys &&
           c_th_default !== nothing &&
           bs.thermal.c_th_j_per_k !== c_th_default
            @warn "Battery $(bs.id): TOML thermal.c_th_j_per_k=$(bs.thermal.c_th_j_per_k) overrides asset value $(c_th_default)"
        end
        if :k_to_ambient_w_per_k in bs.thermal.explicit_keys &&
           k_default !== nothing &&
           bs.thermal.k_to_ambient_w_per_k !== k_default
            @warn "Battery $(bs.id): TOML thermal.k_to_ambient_w_per_k=$(bs.thermal.k_to_ambient_w_per_k) overrides asset value $(k_default)"
        end
        if :ambient_temp_c in bs.thermal.explicit_keys &&
           bs.thermal.ambient_temp_c !== nothing
            @warn "Battery $(bs.id): TOML thermal.ambient_temp_c=$(bs.thermal.ambient_temp_c) overrides asset defaults"
        end
        if :initial_temp_c in bs.thermal.explicit_keys &&
           bs.thermal.initial_temp_c !== nothing
            @warn "Battery $(bs.id): TOML thermal.initial_temp_c=$(bs.thermal.initial_temp_c) overrides asset defaults"
        end
    end

    c_th =
        bs.thermal.c_th_j_per_k !== nothing ? bs.thermal.c_th_j_per_k :
        (c_th_default !== nothing ? c_th_default : 600.0)
    k_to_amb =
        bs.thermal.k_to_ambient_w_per_k !== nothing ? bs.thermal.k_to_ambient_w_per_k :
        (k_default !== nothing ? k_default : 0.0)

    thermal_params = Powertrain.ThermalParams(
        enabled = bs.thermal.enabled,
        c_th_j_per_k = c_th,
        k_to_ambient_w_per_k = k_to_amb,
        ambient_temp_c = temp_amb,
    )

    temp_c = temp_init

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

"""Build a battery model object from a `BatterySpec` (programmatic TOML-equivalent)."""
build_battery(spec::BatterySpec) = _build_battery(spec)

"""Build the tuple of battery *parameter objects* from `spec.power.batteries`.

Battery *state* lives in `Plant.PlantState` and is integrated by the plant model.
"""
function _build_batteries(spec::AircraftSpec)
    B = length(spec.power.batteries)
    B > 0 || throw(ArgumentError("power.batteries must be non-empty"))
    return ntuple(i -> _build_battery(spec.power.batteries[i]), Val(B))
end

"""Build a simple `PlantModels.PowerNetwork` from the aircraft spec.

The topology is intentionally simple:
* every motor must be assigned to exactly one bus
* every battery must be assigned to exactly one bus

Cross-feed/diode OR-ing is out of scope.
"""
function _build_power_network(spec::AircraftSpec)
    N = length(spec.actuation.motors)
    B = length(spec.power.batteries)
    K = length(spec.power.buses)

    # ID → index maps (deterministic, order-driven).
    motor_idx = Dict{MotorId,Int}()
    for (i, m) in enumerate(spec.actuation.motors)
        motor_idx[m.id] = i
    end
    bat_idx = Dict{BatteryId,Int}()
    for (i, b) in enumerate(spec.power.batteries)
        bat_idx[b.id] = i
    end

    # Assign each motor/battery to exactly one bus.
    # (Spec validation already enforces topology; keep only internal assertions here.)
    bus_for_motor = fill(0, N)
    bus_for_battery = fill(0, B)

    for (k, bus) in enumerate(spec.power.buses)
        for mid in bus.motor_ids
            i = motor_idx[mid]
            @assert bus_for_motor[i] == 0
            bus_for_motor[i] = k
        end
        for bid in bus.battery_ids
            i = bat_idx[bid]
            @assert bus_for_battery[i] == 0
            bus_for_battery[i] = k
        end
    end

    @assert all(x -> x != 0, bus_for_motor)
    @assert all(x -> x != 0, bus_for_battery)

    return PowerNetwork{N,B,K}(
        bus_for_motor = SVector{N,Int}(ntuple(i -> bus_for_motor[i], N)),
        bus_for_battery = SVector{B,Int}(ntuple(i -> bus_for_battery[i], B)),
        avionics_load_w = SVector{K,Float64}(
            ntuple(i -> Float64(spec.power.buses[i].avionics_load_w), K),
        ),
        share_mode = spec.power.share_mode,
    )
end

function _build_vehicle(
    spec::AircraftSpec;
    x0_override::Union{Nothing,RigidBodyState} = nothing,
)
    a = spec.airframe

    # Motor/servo actuator models are sized to match the PX4 ABI arrays.
    motor_act = _build_actuator_model(spec.actuation.motor_actuators, 12)
    servo_act = _build_actuator_model(spec.actuation.servo_actuators, 8)

    # Rigid-body model params.
    N = length(spec.actuation.motors)
    rotor_pos = SVector{N,Vec3}(ntuple(i -> a.rotor_pos_body_m[i], N))

    # Propulsor axes: required spec field, normalized here.
    axis_src = a.rotor_axis_body_m
    rotor_axis = SVector{N,Vec3}(
        ntuple(i -> begin
            v = axis_src[i]
            invn = inv(sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3]))
            v .* invn
        end, N),
    )
    # Propulsion (generic multirotor default motor+prop set).
    gravity_model = _build_gravity(spec.environment)
    g_ned = Environment.gravity_accel(gravity_model, Types.vec3(0.0, 0.0, 0.0), 0.0)
    hover_T = a.mass_kg * g_ned[3] / Float64(N)
    p = a.propulsion
    prop = Propulsion.default_multirotor_set(
        N = N,
        km_m = p.km_m,
        V_nom = p.V_nom,
        ρ_nom = p.rho_nom,
        thrust_hover_per_rotor_n = hover_T,
        rotor_radius_m = p.rotor_radius_m,
        inflow_kT = p.inflow_kT,
        inflow_kQ = p.inflow_kQ,
        esc_eta = p.esc.eta,
        esc_deadzone = p.esc.deadzone,
        motor_kv_rpm_per_volt = p.motor.kv_rpm_per_volt,
        motor_r_ohm = p.motor.r_ohm,
        motor_j_kgm2 = p.motor.j_kgm2,
        motor_i0_a = p.motor.i0_a,
        motor_viscous_friction_nm_per_rad_s = p.motor.viscous_friction_nm_per_rad_s,
        motor_max_current_a = p.motor.max_current_a,
        thrust_calibration_mult = p.thrust_calibration_mult,
    )
    if p.rotor_dir !== nothing
        prop.rotor_dir = SVector{N,Float64}(ntuple(i -> Float64(p.rotor_dir[i]), N))
    end

    # Rigid-body inertia tensor (kg*m^2).
    Ixx, Iyy, Izz = a.inertia_diag_kgm2
    Ixy, Ixz, Iyz = a.inertia_products_kgm2
    I_body = @SMatrix [
        Ixx Ixy Ixz
        Ixy Iyy Iyz
        Ixz Iyz Izz
    ]

    # Precompute inverse for the hot path.
    I_body_inv = inv(I_body)

    # Rotor inertias (kg*m^2) are owned by the propulsion units.
    rotor_J =
        SVector{N,Float64}(ntuple(i -> Propulsion.rotor_inertia_kgm2(prop.units[i]), N))

    params = Vehicles.QuadrotorParams{N}(
        mass = a.mass_kg,
        inertia_kgm2 = I_body,
        inertia_inv_kgm2 = I_body_inv,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = rotor_J,
        rotor_dir = prop.rotor_dir,
        linear_drag = a.linear_drag,
        angular_damping = a.angular_damping,
    )

    model = Vehicles.GenericMultirotor{N}(params)

    x0 = x0_override === nothing ? a.x0 : x0_override
    return Vehicles.VehicleInstance(model, motor_act, servo_act, prop, x0)
end



# -----------------
# PX4 parameterization helpers
# -----------------

"""Derive PX4 control allocator (CA_*) parameters from the aircraft spec.

This matches the old lockstep C-side defaults, but is now spec-driven.
"""
function _derive_ca_params(spec::AircraftSpec, vehicle::Vehicles.VehicleInstance)
    cfg = spec.px4.lockstep_config
    cfg.enable_control_allocator == 0 && return PX4ParamSpec[]

    N = length(spec.actuation.motors)
    pos = spec.airframe.rotor_pos_body_m
    length(pos) == N || throw(
        ArgumentError("Cannot derive CA params: rotor_pos_body_m length != motor count"),
    )

    # Rotor/propulsor axes: required by spec; thrust is applied along -axis_b.
    axis_src = spec.airframe.rotor_axis_body_m
    length(axis_src) == N || throw(
        ArgumentError("Cannot derive CA params: rotor_axis_body_m length != motor count"),
    )

    km_mag = spec.airframe.propulsion.km_m
    rotor_dir = vehicle.propulsion.rotor_dir

    params = PX4ParamSpec[
        PX4ParamSpec("CA_AIRFRAME", 0),              # multicopter (custom geometry)
        PX4ParamSpec("CA_ROTOR_COUNT", N),
    ]

    for i = 1:N
        p = pos[i]
        a = axis_src[i]
        n2 = a[1] * a[1] + a[2] * a[2] + a[3] * a[3]
        n2 > 1e-12 || throw(
            ArgumentError(
                "Cannot derive CA params: rotor_axis_body_m[$i] is near-zero: $a",
            ),
        )
        invn = inv(sqrt(n2))
        axis_b = a .* invn
        # PX4 CA_ROTOR*_A* expects the *thrust vector direction* in body frame.
        # Our convention stores propulsor axis such that F = -T * axis_b.
        axis_thrust = -axis_b
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PX", Float32(p[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PY", Float32(p[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PZ", Float32(p[3])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_KM", Float32(km_mag * rotor_dir[i])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AX", Float32(axis_thrust[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AY", Float32(axis_thrust[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AZ", Float32(axis_thrust[3])))
    end

    return params
end

"""Apply a list of PX4 parameters to a running lockstep autopilot."""
function _apply_px4_params!(
    ap::Autopilots.PX4LockstepAutopilot,
    params::Vector{PX4ParamSpec},
)
    for p in params
        param_set!(ap.handle, p.name, p.value)
    end
    return nothing
end

function _debug_px4_params!(
    ap::Autopilots.PX4LockstepAutopilot,
    params::Vector{PX4ParamSpec},
)
    flag = lowercase(String(get(ENV, "PX4_LOCKSTEP_DEBUG_PARAMS", "")))
    if !(flag in ("1", "true", "yes", "on"))
        return nothing
    end

    if isempty(params)
        println("[PX4 params] (no spec params applied)")
        return nothing
    end

    println("[PX4 params] applied:")
    for p in params
        value = try
            param_get(ap.handle, p.name)
        catch err
            "<error: $(sprint(showerror, err))>"
        end
        println("  ", p.name, " = ", value)
    end
    return nothing
end


@inline function _is_ca_axis_param(name::AbstractString)::Bool
    # PX4 CA rotor axis params: CA_ROTOR<i>_AX / _AY / _AZ
    return startswith(name, "CA_ROTOR") &&
           (endswith(name, "_AX") || endswith(name, "_AY") || endswith(name, "_AZ"))
end

function _spec_summary(spec::AircraftSpec)::String
    nm = spec.name
    nmot = length(spec.actuation.motors)
    nb = length(spec.power.batteries)
    ns = length(spec.sensors)
    return "name=$(nm); motors=$(nmot); batteries=$(nb); sensors=$(ns)"
end


# -----------------
# Instance builder
# -----------------

"""Build a fully wired aircraft instance for a given run mode.

Returns `(instance, ap)` where `ap` is the initialized PX4 autopilot handle for
live/record modes (and `nothing` for replay). Call `Autopilots.close!(ap)` when
finished.
"""
function build_aircraft_instance(
    spec::AircraftSpec;
    mode::Symbol = :live,
    recording_in = nothing,
    telemetry = spec.telemetry,
    log_sinks = spec.log_sinks,
)
    # ---------
    # REPLAY
    # ---------
    if mode === :replay
        # Load recording if a path was provided.
        rec = if recording_in isa Recording.Tier0Recording
            recording_in
        elseif recording_in isa AbstractString
            Recording.read_recording(recording_in)
        else
            throw(
                ArgumentError(
                    "build_aircraft_instance(mode=:replay) requires recording_in (Tier0Recording or path)",
                ),
            )
        end

        traces = Recording.tier0_traces(rec)
        scn_tr = try
            Recording.scenario_traces(rec)
        catch e
            throw(
                ArgumentError(
                    "Recording is missing scenario streams needed for replay (faults/ap_cmd/landed). " *
                    "Re-record with record_faults_evt=true.\n\nOriginal error: $e",
                ),
            )
        end

        # Environment: disable live wind (wind comes from trace).
        env = _build_env_replay(spec)

        # Build param objects from spec but override initial RB to match the recording.
        vehicle = _build_vehicle(spec; x0_override = rec.plant0.rb)

        # Multi-battery power network (recording must match spec topology).
        batteries = _build_batteries(spec)
        power_net = _build_power_network(spec)

        # Explicit actuator -> physical propulsor mapping.
        N = length(spec.actuation.motors)
        if length(rec.plant0.rotor_ω) != N
            throw(
                ArgumentError(
                    "Replay recording motor count does not match spec: recording N=$(length(rec.plant0.rotor_ω)) spec N=$(N)",
                ),
            )
        end
        if length(rec.plant0.power.soc) != length(batteries)
            throw(
                ArgumentError(
                    "Replay recording battery count does not match spec: recording B=$(length(rec.plant0.power.soc)) spec B=$(length(batteries))",
                ),
            )
        end
        motor_map = Vehicles.MotorMap{N}(
            SVector{N,Int}(ntuple(i -> spec.actuation.motors[i].channel, N)),
        )
        M = length(spec.actuation.servos)
        servo_map = Vehicles.ServoMap{M}(
            SVector{M,Int}(ntuple(i -> spec.actuation.servos[i].channel, M)),
        )

        dynfun = PlantModels.CoupledMultirotorModel(
            vehicle.model,
            env,
            spec.plant.contact,
            vehicle.motor_actuators,
            vehicle.servo_actuators,
            vehicle.propulsion,
            batteries,
            power_net;
            motor_map = motor_map,
            servo_map = servo_map,
        )

        integrator = _resolve_integrator(spec)

        wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing
        scenario = Sources.ReplayScenarioSource(
            scn_tr.ap_cmd,
            scn_tr.landed,
            scn_tr.faults;
            wind_dist = wind_dist,
        )

        sources = (
            autopilot = Sources.ReplayAutopilotSource(traces.cmd),
            wind = Sources.ReplayWindSource(traces.wind_base_ned),
            scenario = scenario,
            estimator = Sources.NullEstimatorSource(),
        )

        meta = Dict{Symbol,Any}(:aircraft_spec_summary => _spec_summary(spec))

        inst = AircraftInstance(rec.timeline, rec.plant0, dynfun, integrator, sources, meta)
        return inst, nothing
    end

    # ---------
    # LIVE / RECORD
    # ---------

    # Environment with live wind.
    env = _build_env_live(spec)

    # Scenario + timeline.
    scenario_obj = _build_scenario(spec)
    scenario_src = Sources.LiveScenarioSource(scenario_obj)
    timeline = _build_default_timeline(
        t_end_s = spec.timeline.t_end_s,
        dt_autopilot_s = spec.timeline.dt_autopilot_s,
        dt_wind_s = spec.timeline.dt_wind_s,
        dt_log_s = spec.timeline.dt_log_s,
        dt_phys_s = spec.timeline.dt_phys_s,
        scenario_source = scenario_src,
    )

    # Wind + estimator.
    wind_src =
        Sources.LiveWindSource(env.wind, Random.Xoshiro(spec.seed), spec.timeline.dt_wind_s)
    estimator_obj = _build_estimator(spec)
    estimator_src =
        estimator_obj === nothing ? Sources.NullEstimatorSource() :
        Sources.LiveEstimatorSource(
            estimator_obj,
            Random.Xoshiro(spec.seed + 1),
            spec.timeline.dt_autopilot_s,
        )

    # Plant-side param objects.
    vehicle = _build_vehicle(spec)

    # Multi-battery power network.
    batteries = _build_batteries(spec)
    power_net = _build_power_network(spec)

    # Explicit actuator -> physical propulsor mapping.
    N = length(spec.actuation.motors)
    motor_map = Vehicles.MotorMap{N}(
        SVector{N,Int}(ntuple(i -> spec.actuation.motors[i].channel, N)),
    )
    M = length(spec.actuation.servos)
    servo_map = Vehicles.ServoMap{M}(
        SVector{M,Int}(ntuple(i -> spec.actuation.servos[i].channel, M)),
    )

    dynfun = PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        spec.plant.contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        batteries,
        power_net;
        motor_map = motor_map,
        servo_map = servo_map,
    )

    integrator = _resolve_integrator(spec)

    plant0 = Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        batteries,
    )

    # Apply spec-driven PX4 parameters.
    px4_params = PX4ParamSpec[]
    if spec.px4.derive_ca_params
        append!(px4_params, _derive_ca_params(spec, vehicle))
    end
    # Explicit overrides win.
    append!(px4_params, spec.px4.params)

    # Some PX4 branches do not expose CA_ROTOR*_A* parameters (tilt/axis support). We
    # therefore stage those as *optional* and apply them after PX4 init if they exist.
    axis_params = PX4ParamSpec[]
    for p in px4_params
        if _is_ca_axis_param(p.name)
            push!(axis_params, p)
        else
            param_preinit_set!(p.name, p.value; libpath = spec.px4.libpath)
        end
    end

    # PX4 autopilot (lockstep) init.
    ap = Autopilots.init!(
        config = spec.px4.lockstep_config,
        libpath = spec.px4.libpath,
        home = spec.home,
        uorb_cfg = spec.px4.uorb_cfg,
        edge_trigger = spec.px4.edge_trigger,
    )

    # Apply optional CA_ROTOR*_A* params if the PX4 build supports them.
    axis_applied = false
    if !isempty(axis_params)
        for p in axis_params
            # param_get throws if param doesn't exist; treat that as "not supported".
            supported = try
                _ = param_get(ap.handle, p.name)
                true
            catch
                false
            end
            if supported
                param_set!(ap.handle, p.name, p.value)
                axis_applied = true
            end
        end
        # Reload CA params without a global param_notify.
        (axis_applied && spec.px4.lockstep_config.enable_control_allocator != 0) &&
            control_alloc_update_params!(ap.handle)
    end

    # Only broadcast a parameter update if explicit overrides were provided.
    isempty(spec.px4.params) || param_notify!(ap.handle)
    _debug_px4_params!(ap, px4_params)

    # Initial step at t=0 so PX4 has a consistent state baseline.
    _ = Autopilots.autopilot_step(
        ap,
        UInt64(0),
        plant0.rb.pos_ned,
        plant0.rb.vel_ned,
        plant0.rb.q_bn,
        plant0.rb.ω_body,
        Autopilots.AutopilotCommand();
        landed = true,
        battery = Powertrain.BatteryStatus(),
    )

    # Load mission if provided (after uORB topics are initialized).
    if spec.px4.mission_path !== nothing
        Autopilots.load_mission!(ap, spec.px4.mission_path)
    end

    autopilot_src = Sources.LiveAutopilotSource(ap)

    sources = (
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = scenario_src,
        estimator = estimator_src,
    )

    meta = Dict{Symbol,Any}(
        :home => spec.home,
        :mission => spec.px4.mission_path,
        :aircraft_spec_summary => _spec_summary(spec),
    )

    inst = AircraftInstance(timeline, plant0, dynfun, integrator, sources, meta)
    return inst, ap
end


# -----------------
# Public entrypoint
# -----------------

"""Build and run an engine from an `AircraftSpec`.

This is the stable build entrypoint used by `Workflows.simulate_iris_mission`.

Modes
-----
* `:live`   -> return `Runtime.Engine`
* `:record` -> return `Recording.Tier0Recording` (and optionally write to `recording_out`)
* `:replay` -> return `Runtime.Engine`
"""
function build_engine(
    spec::AircraftSpec;
    mode::Symbol = :live,
    recording_in = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
    telemetry = spec.telemetry,
    log_sinks = spec.log_sinks,
    report_timing::Bool = false,
)
    validate_spec(spec; mode = mode, recording_in = recording_in)

    # Multirotor specs only. Additional airframe kinds will be enabled
    # in later phases.

    inst, ap = build_aircraft_instance(
        spec;
        mode = mode,
        recording_in = recording_in,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )

    try
        if mode === :record
            recorder = Recording.InMemoryRecorder()
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :record,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                recorder = recorder,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end

            rec = Recording.Tier0Recording(
                timeline = inst.timeline,
                plant0 = inst.plant0,
                recorder = recorder,
                meta = inst.meta,
            )
            if recording_out !== nothing
                Recording.write_recording(recording_out, rec)
            end
            return rec

        elseif mode === :replay
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :replay,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end
            return eng

        elseif mode === :live
            t0_ns = time_ns()
            eng = _SIM.simulate(
                mode = :live,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            if report_timing
                wall_s = (time_ns() - t0_ns) / 1e9
                sim_s = (inst.timeline.t_end_us - inst.timeline.t0_us) / 1e6
                rtf = sim_s / wall_s
                println(
                    "Runtime: sim_s=$(round(sim_s, digits=3)) wall_s=$(round(wall_s, digits=3)) rtf=$(round(rtf, digits=3))",
                )
            end
            return eng

        else
            throw(
                ArgumentError(
                    "build_engine: unknown mode=$(mode) (expected :live|:record|:replay)",
                ),
            )
        end
    finally
        ap === nothing || Autopilots.close!(ap)
    end
end
