"""Aircraft spec TOML I/O.

This module provides a small, dependency-light path from declarative TOML aircraft
specs to the existing `AircraftSpec` structs and `build_engine` entrypoint.

Design goals
------------
* **Deterministic**: parsing is pure; relative paths resolve against the TOML
  directory.
* **Strict by default**: unknown keys throw to catch typos early.
* **Composable**: supports `extends = ["base.toml", ...]` with deep table merges.

TOML schema
-----------
This intentionally mirrors the spec struct layout:

Top-level:
  - schema_version = 1
  - extends = ["base.toml"]  (optional)
  - [aircraft]
  - [home]
  - [px4]
  - [timeline]
  - [environment]
  - [scenario]
  - [estimator]
  - [plant]
  - [airframe]
  - [actuation]
  - [power]
  - [sensors]
  - [run] (optional convenience for `run_spec`)
"""

using TOML

using ..Types: Vec3, Quat, vec3, quat_normalize
using ..RigidBody: RigidBodyState
using ..Contacts
using ..Integrators
using ..Autopilots
using ..Logging

import ...LockstepConfig


# Root module handle (PX4Lockstep). Used to resolve generated uORB message types.
const _ROOT = parentmodule(parentmodule(@__MODULE__))


# -----------------------------------------------------------------------------
# Small utilities
# -----------------------------------------------------------------------------

@inline _sym(x::Symbol) = x
@inline _sym(x::AbstractString) = Symbol(x)

function _as_string(x, ctx::AbstractString)
    x isa AbstractString || error("$ctx must be a string (got $(typeof(x)))")
    return String(x)
end


# -----------------------------------------------------------------------------
# Internal default spec (generic multirotor)
# -----------------------------------------------------------------------------

const _DEFAULT_MULTIROTOR_SPEC_PATH =
    joinpath(@__DIR__, "assets", "multirotor_default.toml")
const _DEFAULT_MULTIROTOR_SPEC_CACHE = Ref{Union{Nothing,AircraftSpec}}(nothing)

"""Path to the built-in generic multirotor spec (Sim internal)."""
default_multirotor_spec_path() = _DEFAULT_MULTIROTOR_SPEC_PATH

"""Load the built-in generic multirotor spec (Sim internal)."""
function default_multirotor_spec(; strict::Bool = true)
    return _default_multirotor_spec_cached(strict = strict)
end

function _as_bool(x, ctx::AbstractString)
    x isa Bool || error("$ctx must be a bool (got $(typeof(x)))")
    return x
end

function _as_int(x, ctx::AbstractString)
    x isa Integer || error("$ctx must be an integer (got $(typeof(x)))")
    return Int(x)
end

function _as_f64(x, ctx::AbstractString)
    x isa Real || error("$ctx must be a number (got $(typeof(x)))")
    return Float64(x)
end

function _as_table(x, ctx::AbstractString)
    x isa AbstractDict || error("$ctx must be a table (got $(typeof(x)))")
    return x
end

function _as_array(x, ctx::AbstractString)
    x isa AbstractVector || error("$ctx must be an array (got $(typeof(x)))")
    return x
end

function _known_keys!(tbl::AbstractDict, allowed::Set{String}, ctx::AbstractString)
    for k in keys(tbl)
        ks = String(k)
        ks in allowed || error("Unknown key '$ks' in $ctx")
    end
    return nothing
end

function _resolve_path(
    base_dir::AbstractString,
    p::Union{Nothing,AbstractString};
    strict::Bool,
)
    p === nothing && return nothing
    s = String(p)
    s = Base.expanduser(s)
    # Empty string is almost always accidental; treat as "unset".
    isempty(strip(s)) && return nothing
    return isabspath(s) ? normpath(s) : normpath(joinpath(base_dir, s))
end


# -----------------------------------------------------------------------------
# TOML deep merge helpers (for `extends`)
# -----------------------------------------------------------------------------

"""Deep-merge TOML dictionaries.

Rules
-----
* Tables merge recursively.
* Arrays replace (not append).
* Scalars replace.
"""
function _deep_merge(a::AbstractDict, b::AbstractDict)
    out = Dict{String,Any}()
    # Copy A
    for (k, v) in a
        out[String(k)] = v
    end
    # Merge/override from B
    for (k_any, vb) in b
        k = String(k_any)
        if haskey(out, k)
            va = out[k]
            if va isa AbstractDict && vb isa AbstractDict
                out[k] = _deep_merge(va, vb)
            else
                # arrays + scalars replace
                out[k] = vb
            end
        else
            out[k] = vb
        end
    end
    return out
end

function _load_toml_with_extends(path::AbstractString; strict::Bool, _stack = String[])
    abspath_ = abspath(path)
    abspath_ in _stack &&
        error("TOML extends cycle detected: $(join(vcat(_stack, [abspath_]), " -> "))")
    push!(_stack, abspath_)
    cfg = TOML.parsefile(abspath_)
    base_dir = dirname(abspath_)

    # Collect extends.
    ex = get(cfg, "extends", nothing)
    merged = Dict{String,Any}()
    if ex !== nothing
        ex_arr = _as_array(ex, "extends")
        for (i, e) in enumerate(ex_arr)
            epath = _resolve_path(base_dir, _as_string(e, "extends[$i]"); strict = strict)
            epath === nothing && error("extends[$i] resolved to nothing")
            base_cfg = _load_toml_with_extends(epath; strict = strict, _stack = _stack)
            merged = _deep_merge(merged, base_cfg)
        end
    end

    # Overlay this file last.
    merged = _deep_merge(merged, cfg)

    pop!(_stack)
    return merged
end


# -----------------------------------------------------------------------------
# Type parsers
# -----------------------------------------------------------------------------

function _parse_vec3(x, ctx::AbstractString)::Vec3
    arr = _as_array(x, ctx)
    length(arr) == 3 || error("$ctx must have length 3")
    return vec3(
        _as_f64(arr[1], "$ctx[1]"),
        _as_f64(arr[2], "$ctx[2]"),
        _as_f64(arr[3], "$ctx[3]"),
    )
end

function _parse_quat(x, ctx::AbstractString)::Quat
    arr = _as_array(x, ctx)
    length(arr) == 4 || error("$ctx must have length 4 (w,x,y,z)")
    q = Quat(
        _as_f64(arr[1], "$ctx[1]"),
        _as_f64(arr[2], "$ctx[2]"),
        _as_f64(arr[3], "$ctx[3]"),
        _as_f64(arr[4], "$ctx[4]"),
    )
    return quat_normalize(q)
end

"""Euler (roll,pitch,yaw) → quaternion (Body→NED), ZYX convention.

The angles are intrinsic rotations about body axes X,Y,Z applied as roll→pitch→yaw.
This matches the common aerospace convention and is consistent with `yaw_from_quat`.
"""
function _quat_from_euler(roll::Float64, pitch::Float64, yaw::Float64)::Quat
    cr = cos(roll / 2)
    sr = sin(roll / 2)
    cp = cos(pitch / 2)
    sp = sin(pitch / 2)
    cy = cos(yaw / 2)
    sy = sin(yaw / 2)

    q = Quat(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return quat_normalize(q)
end

function _parse_rigid_body_state(tbl_any; strict::Bool, ctx::AbstractString)
    tbl = _as_table(tbl_any, ctx)
    # Accept both unicode and ASCII keys for convenience.
    strict && _known_keys!(
        tbl,
        Set([
            "pos_ned",
            "vel_ned",
            "q_bn",
            "euler_deg",
            "euler_rad",
            "ω_body",
            "omega_body",
        ]),
        ctx,
    )

    pos =
        haskey(tbl, "pos_ned") ? _parse_vec3(tbl["pos_ned"], "$ctx.pos_ned") : vec3(0, 0, 0)
    vel =
        haskey(tbl, "vel_ned") ? _parse_vec3(tbl["vel_ned"], "$ctx.vel_ned") : vec3(0, 0, 0)
    ω = if haskey(tbl, "ω_body")
        _parse_vec3(tbl["ω_body"], "$ctx.ω_body")
    elseif haskey(tbl, "omega_body")
        _parse_vec3(tbl["omega_body"], "$ctx.omega_body")
    else
        vec3(0, 0, 0)
    end

    q = Quat(1.0, 0.0, 0.0, 0.0)
    if haskey(tbl, "q_bn")
        q = _parse_quat(tbl["q_bn"], "$ctx.q_bn")
    elseif haskey(tbl, "euler_deg")
        e = _as_array(tbl["euler_deg"], "$ctx.euler_deg")
        length(e) == 3 || error("$ctx.euler_deg must have length 3")
        roll = deg2rad(_as_f64(e[1], "$ctx.euler_deg[1]"))
        pitch = deg2rad(_as_f64(e[2], "$ctx.euler_deg[2]"))
        yaw = deg2rad(_as_f64(e[3], "$ctx.euler_deg[3]"))
        q = _quat_from_euler(roll, pitch, yaw)
    elseif haskey(tbl, "euler_rad")
        e = _as_array(tbl["euler_rad"], "$ctx.euler_rad")
        length(e) == 3 || error("$ctx.euler_rad must have length 3")
        roll = _as_f64(e[1], "$ctx.euler_rad[1]")
        pitch = _as_f64(e[2], "$ctx.euler_rad[2]")
        yaw = _as_f64(e[3], "$ctx.euler_rad[3]")
        q = _quat_from_euler(roll, pitch, yaw)
    end

    return RigidBodyState(pos_ned = pos, vel_ned = vel, q_bn = q, ω_body = ω)
end


function _parse_contact(tbl_any; strict::Bool, ctx::AbstractString)
    if tbl_any isa AbstractString
        k = lowercase(String(tbl_any))
        if k in ("none", "no", "off", "no_contact", "nocontact")
            return Contacts.NoContact()
        elseif k in ("flat", "flat_ground", "ground")
            return Contacts.FlatGroundContact()
        else
            error(
                "$ctx: unknown contact kind '$tbl_any' (expected 'flat_ground' or 'no_contact')",
            )
        end
    end

    tbl = _as_table(tbl_any, ctx)
    # Allow ASCII alias `mu` for convenience.
    strict && _known_keys!(
        tbl,
        Set(["kind", "k_n_per_m", "c_n_per_mps", "μ", "mu", "v_eps", "enable_friction"]),
        ctx,
    )
    kind = lowercase(String(get(tbl, "kind", "flat_ground")))
    if kind in ("flat", "flat_ground", "ground")
        return Contacts.FlatGroundContact(
            k_n_per_m = _as_f64(get(tbl, "k_n_per_m", 5_000.0), "$ctx.k_n_per_m"),
            c_n_per_mps = _as_f64(get(tbl, "c_n_per_mps", 600.0), "$ctx.c_n_per_mps"),
            μ = _as_f64(get(tbl, haskey(tbl, "μ") ? "μ" : "mu", 0.8), "$ctx.mu"),
            v_eps = _as_f64(get(tbl, "v_eps", 0.05), "$ctx.v_eps"),
            enable_friction = _as_bool(
                get(tbl, "enable_friction", true),
                "$ctx.enable_friction",
            ),
        )
    elseif kind in ("none", "no", "off", "no_contact", "nocontact")
        return Contacts.NoContact()
    else
        error("$ctx: unknown contact kind '$kind' (expected 'flat_ground' or 'no_contact')")
    end
end

@inline function _norm_kind(x, ctx::AbstractString)
    return Symbol(lowercase(_as_string(x, ctx)))
end

function _parse_environment(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::EnvironmentSpec,
)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "wind",
        "wind_mean_ned",
        "wind_sigma_ned",
        "wind_tau_s",
        "atmosphere",
        "gravity",
        "gravity_mps2",
        "gravity_mu",
        "gravity_r0_m",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    wind = haskey(tbl, "wind") ? _norm_kind(tbl["wind"], "$ctx.wind") : base.wind
    wind =
        wind in (:none, :no, :off, :no_wind) ? :none :
        wind in (:ou, :ou_wind, :turbulence) ? :ou :
        wind in (:constant, :const) ? :constant : wind
    wind in (:none, :ou, :constant) ||
        error("$ctx.wind must be one of 'none', 'ou', 'constant' (got $(wind))")

    wind_mean_ned =
        haskey(tbl, "wind_mean_ned") ?
        _parse_vec3(tbl["wind_mean_ned"], "$ctx.wind_mean_ned") : base.wind_mean_ned
    wind_sigma_ned =
        haskey(tbl, "wind_sigma_ned") ?
        _parse_vec3(tbl["wind_sigma_ned"], "$ctx.wind_sigma_ned") : base.wind_sigma_ned
    wind_tau_s =
        haskey(tbl, "wind_tau_s") ? _as_f64(tbl["wind_tau_s"], "$ctx.wind_tau_s") :
        base.wind_tau_s

    atmosphere =
        haskey(tbl, "atmosphere") ? _norm_kind(tbl["atmosphere"], "$ctx.atmosphere") :
        base.atmosphere
    atmosphere in (:isa1976, :isa) || error("$ctx.atmosphere must be 'isa1976'")

    gravity =
        haskey(tbl, "gravity") ? _norm_kind(tbl["gravity"], "$ctx.gravity") : base.gravity
    gravity = gravity in (:uniform, :spherical) ? gravity : gravity
    gravity in (:uniform, :spherical) ||
        error("$ctx.gravity must be 'uniform' or 'spherical'")

    g_mps2 =
        haskey(tbl, "gravity_mps2") ? _as_f64(tbl["gravity_mps2"], "$ctx.gravity_mps2") :
        base.gravity_mps2
    g_mu =
        haskey(tbl, "gravity_mu") ? _as_f64(tbl["gravity_mu"], "$ctx.gravity_mu") :
        base.gravity_mu
    g_r0 =
        haskey(tbl, "gravity_r0_m") ? _as_f64(tbl["gravity_r0_m"], "$ctx.gravity_r0_m") :
        base.gravity_r0_m

    return EnvironmentSpec(
        wind = wind,
        wind_mean_ned = wind_mean_ned,
        wind_sigma_ned = wind_sigma_ned,
        wind_tau_s = wind_tau_s,
        atmosphere = atmosphere == :isa ? :isa1976 : atmosphere,
        gravity = gravity,
        gravity_mps2 = g_mps2,
        gravity_mu = g_mu,
        gravity_r0_m = g_r0,
    )
end

function _parse_scenario(tbl_any; strict::Bool, ctx::AbstractString, base::ScenarioSpec)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set(["arm_time_s", "mission_time_s"])
    strict && _known_keys!(tbl, allowed, ctx)
    arm_time_s =
        haskey(tbl, "arm_time_s") ? _as_f64(tbl["arm_time_s"], "$ctx.arm_time_s") :
        base.arm_time_s
    mission_time_s =
        haskey(tbl, "mission_time_s") ?
        _as_f64(tbl["mission_time_s"], "$ctx.mission_time_s") : base.mission_time_s
    return ScenarioSpec(arm_time_s = arm_time_s, mission_time_s = mission_time_s)
end

function _parse_estimator(tbl_any; strict::Bool, ctx::AbstractString, base::EstimatorSpec)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "kind",
        "pos_sigma_m",
        "vel_sigma_mps",
        "yaw_sigma_rad",
        "rate_sigma_rad_s",
        "bias_tau_s",
        "rate_bias_sigma_rad_s",
        "delay_s",
        "dt_est_s",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    kind = haskey(tbl, "kind") ? _norm_kind(tbl["kind"], "$ctx.kind") : base.kind
    kind = kind in (:none, :no, :off) ? :none : kind
    kind in (:none, :noisy_delayed) || error("$ctx.kind must be 'noisy_delayed' or 'none'")

    pos_sigma_m =
        haskey(tbl, "pos_sigma_m") ? _parse_vec3(tbl["pos_sigma_m"], "$ctx.pos_sigma_m") :
        base.pos_sigma_m
    vel_sigma_mps =
        haskey(tbl, "vel_sigma_mps") ?
        _parse_vec3(tbl["vel_sigma_mps"], "$ctx.vel_sigma_mps") : base.vel_sigma_mps
    yaw_sigma_rad =
        haskey(tbl, "yaw_sigma_rad") ? _as_f64(tbl["yaw_sigma_rad"], "$ctx.yaw_sigma_rad") :
        base.yaw_sigma_rad
    rate_sigma_rad_s =
        haskey(tbl, "rate_sigma_rad_s") ?
        _parse_vec3(tbl["rate_sigma_rad_s"], "$ctx.rate_sigma_rad_s") :
        base.rate_sigma_rad_s
    bias_tau_s =
        haskey(tbl, "bias_tau_s") ? _as_f64(tbl["bias_tau_s"], "$ctx.bias_tau_s") :
        base.bias_tau_s
    rate_bias_sigma_rad_s =
        haskey(tbl, "rate_bias_sigma_rad_s") ?
        _parse_vec3(tbl["rate_bias_sigma_rad_s"], "$ctx.rate_bias_sigma_rad_s") :
        base.rate_bias_sigma_rad_s
    delay_s =
        haskey(tbl, "delay_s") ?
        (tbl["delay_s"] === nothing ? nothing : _as_f64(tbl["delay_s"], "$ctx.delay_s")) :
        base.delay_s
    dt_est_s =
        haskey(tbl, "dt_est_s") ?
        (
            tbl["dt_est_s"] === nothing ? nothing :
            _as_f64(tbl["dt_est_s"], "$ctx.dt_est_s")
        ) : base.dt_est_s

    return EstimatorSpec(
        kind = kind,
        pos_sigma_m = pos_sigma_m,
        vel_sigma_mps = vel_sigma_mps,
        yaw_sigma_rad = yaw_sigma_rad,
        rate_sigma_rad_s = rate_sigma_rad_s,
        bias_tau_s = bias_tau_s,
        rate_bias_sigma_rad_s = rate_bias_sigma_rad_s,
        delay_s = delay_s,
        dt_est_s = dt_est_s,
    )
end

function _parse_integrator(x, ctx::AbstractString; strict::Bool)
    if x isa Symbol
        return x
    elseif x isa AbstractString
        return Symbol(x)
    elseif x isa AbstractDict
        tbl = _as_table(x, ctx)
        kind_raw = get(tbl, "kind", nothing)
        kind_raw === nothing && error("$ctx.kind is required (e.g. 'RK45')")
        kind_str = lowercase(_as_string(kind_raw, "$ctx.kind"))
        kind_sym = Symbol(uppercase(kind_str))

        if kind_sym === :RK23 || kind_sym === :RK45
            # Adaptive integrators: allow full tolerance + step-size config.
            allowed = Set([
                "kind",
                "rtol_pos",
                "atol_pos",
                "rtol_vel",
                "atol_vel",
                "rtol_ω",
                "atol_ω",
                "rtol_omega",
                "atol_omega",
                "atol_att_rad",
                "plant_error_control",
                "rtol_act",
                "atol_act",
                "rtol_actdot",
                "atol_actdot",
                "rtol_rotor",
                "atol_rotor",
                "rtol_soc",
                "atol_soc",
                "rtol_v1",
                "atol_v1",
                "h_min",
                "h_max",
                "h_init",
                "max_substeps",
                "safety",
                "min_factor",
                "max_factor",
                "quantize_us",
            ])
            strict && _known_keys!(tbl, allowed, ctx)

            integ =
                kind_sym === :RK23 ? Integrators.RK23Integrator() :
                Integrators.RK45Integrator()
            rtol_pos = _as_f64(get(tbl, "rtol_pos", integ.rtol_pos), "$ctx.rtol_pos")
            atol_pos = _as_f64(get(tbl, "atol_pos", integ.atol_pos), "$ctx.atol_pos")
            rtol_vel = _as_f64(get(tbl, "rtol_vel", integ.rtol_vel), "$ctx.rtol_vel")
            atol_vel = _as_f64(get(tbl, "atol_vel", integ.atol_vel), "$ctx.atol_vel")
            rtol_ω =
                haskey(tbl, "rtol_ω") ? _as_f64(tbl["rtol_ω"], "$ctx.rtol_ω") :
                (
                    haskey(tbl, "rtol_omega") ?
                    _as_f64(tbl["rtol_omega"], "$ctx.rtol_omega") : integ.rtol_ω
                )
            atol_ω =
                haskey(tbl, "atol_ω") ? _as_f64(tbl["atol_ω"], "$ctx.atol_ω") :
                (
                    haskey(tbl, "atol_omega") ?
                    _as_f64(tbl["atol_omega"], "$ctx.atol_omega") : integ.atol_ω
                )
            atol_att_rad =
                _as_f64(get(tbl, "atol_att_rad", integ.atol_att_rad), "$ctx.atol_att_rad")
            plant_error_control = _as_bool(
                get(tbl, "plant_error_control", integ.plant_error_control),
                "$ctx.plant_error_control",
            )

            rtol_act = _as_f64(get(tbl, "rtol_act", integ.rtol_act), "$ctx.rtol_act")
            atol_act = _as_f64(get(tbl, "atol_act", integ.atol_act), "$ctx.atol_act")
            rtol_actdot =
                _as_f64(get(tbl, "rtol_actdot", integ.rtol_actdot), "$ctx.rtol_actdot")
            atol_actdot =
                _as_f64(get(tbl, "atol_actdot", integ.atol_actdot), "$ctx.atol_actdot")
            rtol_rotor =
                _as_f64(get(tbl, "rtol_rotor", integ.rtol_rotor), "$ctx.rtol_rotor")
            atol_rotor =
                _as_f64(get(tbl, "atol_rotor", integ.atol_rotor), "$ctx.atol_rotor")
            rtol_soc = _as_f64(get(tbl, "rtol_soc", integ.rtol_soc), "$ctx.rtol_soc")
            atol_soc = _as_f64(get(tbl, "atol_soc", integ.atol_soc), "$ctx.atol_soc")
            rtol_v1 = _as_f64(get(tbl, "rtol_v1", integ.rtol_v1), "$ctx.rtol_v1")
            atol_v1 = _as_f64(get(tbl, "atol_v1", integ.atol_v1), "$ctx.atol_v1")

            h_min = _as_f64(get(tbl, "h_min", integ.h_min), "$ctx.h_min")
            h_max = _as_f64(get(tbl, "h_max", integ.h_max), "$ctx.h_max")
            h_init = _as_f64(get(tbl, "h_init", integ.h_init), "$ctx.h_init")
            max_substeps = Int(
                _as_int(get(tbl, "max_substeps", integ.max_substeps), "$ctx.max_substeps"),
            )
            safety = _as_f64(get(tbl, "safety", integ.safety), "$ctx.safety")
            min_factor =
                _as_f64(get(tbl, "min_factor", integ.min_factor), "$ctx.min_factor")
            max_factor =
                _as_f64(get(tbl, "max_factor", integ.max_factor), "$ctx.max_factor")
            quantize_us =
                _as_bool(get(tbl, "quantize_us", integ.quantize_us), "$ctx.quantize_us")

            if kind_sym === :RK23
                return Integrators.RK23Integrator(
                    rtol_pos = rtol_pos,
                    atol_pos = atol_pos,
                    rtol_vel = rtol_vel,
                    atol_vel = atol_vel,
                    rtol_ω = rtol_ω,
                    atol_ω = atol_ω,
                    atol_att_rad = atol_att_rad,
                    plant_error_control = plant_error_control,
                    rtol_act = rtol_act,
                    atol_act = atol_act,
                    rtol_actdot = rtol_actdot,
                    atol_actdot = atol_actdot,
                    rtol_rotor = rtol_rotor,
                    atol_rotor = atol_rotor,
                    rtol_soc = rtol_soc,
                    atol_soc = atol_soc,
                    rtol_v1 = rtol_v1,
                    atol_v1 = atol_v1,
                    h_min = h_min,
                    h_max = h_max,
                    h_init = h_init,
                    max_substeps = max_substeps,
                    safety = safety,
                    min_factor = min_factor,
                    max_factor = max_factor,
                    quantize_us = quantize_us,
                )
            end

            return Integrators.RK45Integrator(
                rtol_pos = rtol_pos,
                atol_pos = atol_pos,
                rtol_vel = rtol_vel,
                atol_vel = atol_vel,
                rtol_ω = rtol_ω,
                atol_ω = atol_ω,
                atol_att_rad = atol_att_rad,
                plant_error_control = plant_error_control,
                rtol_act = rtol_act,
                atol_act = atol_act,
                rtol_actdot = rtol_actdot,
                atol_actdot = atol_actdot,
                rtol_rotor = rtol_rotor,
                atol_rotor = atol_rotor,
                rtol_soc = rtol_soc,
                atol_soc = atol_soc,
                rtol_v1 = rtol_v1,
                atol_v1 = atol_v1,
                h_min = h_min,
                h_max = h_max,
                h_init = h_init,
                max_substeps = max_substeps,
                safety = safety,
                min_factor = min_factor,
                max_factor = max_factor,
                quantize_us = quantize_us,
            )
        elseif kind_sym === :RK4
            strict && _known_keys!(tbl, Set(["kind"]), ctx)
            return Integrators.RK4Integrator()
        elseif kind_sym === :EULER
            strict && _known_keys!(tbl, Set(["kind"]), ctx)
            return Integrators.EulerIntegrator()
        else
            error(
                "$ctx.kind must be one of 'Euler', 'RK4', 'RK23', 'RK45' (got '$kind_str')",
            )
        end
    else
        error("$ctx must be a string/symbol or table (e.g. 'RK45' or {kind='RK45'})")
    end
end

function _resolve_uorb_msg_type(type_name::AbstractString)
    sym = Symbol(type_name)
    isdefined(_ROOT, sym) ||
        error("Unknown uORB message type '$type_name' (not defined in PX4Lockstep module)")
    T = getfield(_ROOT, sym)
    T isa DataType || error("uORB type '$type_name' did not resolve to a DataType")
    T <: _ROOT.UORBMsg ||
        error("uORB type '$type_name' is not a subtype of PX4Lockstep.UORBMsg")
    return T
end

function _parse_uorb_interface(tbl_any; strict::Bool, ctx::AbstractString)
    tbl = _as_table(tbl_any, ctx)
    strict && _known_keys!(tbl, Set(["pubs", "subs"]), ctx)

    pubs = Autopilots.UORBPubSpec[]
    subs = Autopilots.UORBSubSpec[]

    if haskey(tbl, "pubs")
        arr = _as_array(tbl["pubs"], "$ctx.pubs")
        for (i, p_any) in enumerate(arr)
            p = _as_table(p_any, "$ctx.pubs[$i]")
            strict && _known_keys!(
                p,
                Set(["key", "type", "instance", "priority", "queue_size"]),
                "$ctx.pubs[$i]",
            )
            key = _sym(_as_string(get(p, "key", nothing), "$ctx.pubs[$i].key"))
            typ = _resolve_uorb_msg_type(
                _as_string(get(p, "type", nothing), "$ctx.pubs[$i].type"),
            )
            inst = Int32(get(p, "instance", -1))
            inst >= -1 ||
                error("$ctx.pubs[$i].instance must be -1 (auto) or >= 0 (got $(inst))")
            prio = Int32(get(p, "priority", 0))
            qs = get(p, "queue_size", nothing)
            qs_i32 =
                qs === nothing ? nothing : Int32(_as_int(qs, "$ctx.pubs[$i].queue_size"))
            push!(
                pubs,
                Autopilots.UORBPubSpec(
                    key = key,
                    type = typ,
                    instance = inst,
                    priority = prio,
                    queue_size = qs_i32,
                ),
            )
        end
    end

    if haskey(tbl, "subs")
        arr = _as_array(tbl["subs"], "$ctx.subs")
        for (i, s_any) in enumerate(arr)
            s = _as_table(s_any, "$ctx.subs[$i]")
            strict && _known_keys!(s, Set(["key", "type", "instance"]), "$ctx.subs[$i]")
            key = _sym(_as_string(get(s, "key", nothing), "$ctx.subs[$i].key"))
            typ = _resolve_uorb_msg_type(
                _as_string(get(s, "type", nothing), "$ctx.subs[$i].type"),
            )
            inst_raw = _as_int(get(s, "instance", 0), "$ctx.subs[$i].instance")
            inst_raw >= 0 || error("$ctx.subs[$i].instance must be >= 0 (got $(inst_raw))")
            inst = UInt32(inst_raw)
            push!(subs, Autopilots.UORBSubSpec(key = key, type = typ, instance = inst))
        end
    end

    return Autopilots.PX4UORBInterfaceConfig(pubs = pubs, subs = subs)
end

function _parse_lockstep_cfg(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::LockstepConfig,
)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "dataman_use_ram",
        "enable_commander",
        "commander_rate_hz",
        "navigator_rate_hz",
        "mc_pos_control_rate_hz",
        "mc_att_control_rate_hz",
        "mc_rate_control_rate_hz",
        "enable_control_allocator",
        "control_allocator_rate_hz",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    # LockstepConfig is isbits, so we construct a new one.
    return LockstepConfig(
        dataman_use_ram = Int32(get(tbl, "dataman_use_ram", base.dataman_use_ram)),
        enable_commander = Int32(get(tbl, "enable_commander", base.enable_commander)),
        commander_rate_hz = Int32(get(tbl, "commander_rate_hz", base.commander_rate_hz)),
        navigator_rate_hz = Int32(get(tbl, "navigator_rate_hz", base.navigator_rate_hz)),
        mc_pos_control_rate_hz = Int32(
            get(tbl, "mc_pos_control_rate_hz", base.mc_pos_control_rate_hz),
        ),
        mc_att_control_rate_hz = Int32(
            get(tbl, "mc_att_control_rate_hz", base.mc_att_control_rate_hz),
        ),
        mc_rate_control_rate_hz = Int32(
            get(tbl, "mc_rate_control_rate_hz", base.mc_rate_control_rate_hz),
        ),
        enable_control_allocator = Int32(
            get(tbl, "enable_control_allocator", base.enable_control_allocator),
        ),
        control_allocator_rate_hz = Int32(
            get(tbl, "control_allocator_rate_hz", base.control_allocator_rate_hz),
        ),
    )
end

function _parse_px4_params(arr_any; strict::Bool, ctx::AbstractString)
    arr_any === nothing && return PX4ParamSpec[]
    arr = _as_array(arr_any, ctx)
    out = PX4ParamSpec[]
    for (i, p_any) in enumerate(arr)
        p = _as_table(p_any, "$ctx[$i]")
        strict && _known_keys!(p, Set(["name", "value"]), "$ctx[$i]")
        name = _as_string(get(p, "name", nothing), "$ctx[$i].name")
        v = get(p, "value", nothing)
        v === nothing && error("$ctx[$i].value is required")
        if v isa Integer
            push!(out, PX4ParamSpec(name, Int32(v)))
        elseif v isa Real
            push!(out, PX4ParamSpec(name, Float32(v)))
        else
            error("$ctx[$i].value must be int or float (got $(typeof(v)))")
        end
    end
    return out
end


function _parse_actuator_model(tbl_any; strict::Bool, ctx::AbstractString)
    # Allow string shorthand: kind = "direct" etc.
    if tbl_any isa AbstractString
        k = lowercase(String(tbl_any))
        k in ("direct", "algebraic") && return DirectActuatorSpec()
        k in ("first_order", "firstorder") && return FirstOrderActuatorSpec()
        k in ("second_order", "secondorder") && return SecondOrderActuatorSpec()
        error("$ctx: unknown actuator model '$tbl_any'")
    end

    tbl = _as_table(tbl_any, ctx)
    strict && _known_keys!(tbl, Set(["kind", "τ", "ωn", "ζ", "rate_limit"]), ctx)
    kind = lowercase(String(get(tbl, "kind", "direct")))
    if kind in ("direct", "algebraic")
        return DirectActuatorSpec()
    elseif kind in ("first_order", "firstorder")
        return FirstOrderActuatorSpec(τ = _as_f64(get(tbl, "τ", 0.05), "$ctx.τ"))
    elseif kind in ("second_order", "secondorder")
        return SecondOrderActuatorSpec(
            ωn = _as_f64(get(tbl, "ωn", 20.0), "$ctx.ωn"),
            ζ = _as_f64(get(tbl, "ζ", 0.7), "$ctx.ζ"),
            rate_limit = _as_f64(get(tbl, "rate_limit", Inf), "$ctx.rate_limit"),
        )
    else
        error("$ctx.kind: unknown kind '$kind' (expected direct|first_order|second_order)")
    end
end


function _parse_airframe(tbl_any; strict::Bool, ctx::AbstractString, base::AirframeSpec)
    tbl = _as_table(tbl_any, ctx)
    allowed = Set([
        "kind",
        "mass_kg",
        "inertia_diag_kgm2",
        "inertia_products_kgm2",
        "inertia_kgm2",
        "rotor_pos_body_m",
        "rotor_axis_body_m",
        "linear_drag",
        "angular_damping",
        "x0",
        "propulsion",
    ])
    strict && _known_keys!(tbl, allowed, ctx)

    kind = haskey(tbl, "kind") ? _sym(_as_string(tbl["kind"], "$ctx.kind")) : base.kind
    mass_kg =
        haskey(tbl, "mass_kg") ? _as_f64(tbl["mass_kg"], "$ctx.mass_kg") : base.mass_kg

    # Inertia: allow either full 3x3 tensor or (diag + products).
    inertia_diag = base.inertia_diag_kgm2
    inertia_prod = base.inertia_products_kgm2

    if haskey(tbl, "inertia_kgm2")
        I_any = tbl["inertia_kgm2"]
        arr = _as_array(I_any, "$ctx.inertia_kgm2")
        if length(arr) == 3 && all(x -> x isa AbstractVector, arr)
            row1 = _as_array(arr[1], "$ctx.inertia_kgm2[1]")
            row2 = _as_array(arr[2], "$ctx.inertia_kgm2[2]")
            row3 = _as_array(arr[3], "$ctx.inertia_kgm2[3]")
            (length(row1) == 3 && length(row2) == 3 && length(row3) == 3) ||
                error("$ctx.inertia_kgm2 must be a 3x3 nested array")
            I11 = _as_f64(row1[1], "$ctx.inertia_kgm2[1][1]")
            I12 = _as_f64(row1[2], "$ctx.inertia_kgm2[1][2]")
            I13 = _as_f64(row1[3], "$ctx.inertia_kgm2[1][3]")
            I21 = _as_f64(row2[1], "$ctx.inertia_kgm2[2][1]")
            I22 = _as_f64(row2[2], "$ctx.inertia_kgm2[2][2]")
            I23 = _as_f64(row2[3], "$ctx.inertia_kgm2[2][3]")
            I31 = _as_f64(row3[1], "$ctx.inertia_kgm2[3][1]")
            I32 = _as_f64(row3[2], "$ctx.inertia_kgm2[3][2]")
            I33 = _as_f64(row3[3], "$ctx.inertia_kgm2[3][3]")

            # Require symmetry (products of inertia). Keep the error explicit so
            # users don't silently get a different tensor than they intended.
            atol = 1e-12
            (abs(I12 - I21) <= atol && abs(I13 - I31) <= atol && abs(I23 - I32) <= atol) ||
                error("$ctx.inertia_kgm2 must be symmetric (I12==I21, I13==I31, I23==I32)")

            inertia_diag = vec3(I11, I22, I33)
            inertia_prod = vec3(I12, I13, I23)
        elseif length(arr) == 9
            # Flat 9-element row-major representation.
            v = [_as_f64(arr[i], "$ctx.inertia_kgm2[$i]") for i = 1:9]
            I11, I12, I13, I21, I22, I23, I31, I32, I33 = v
            atol = 1e-12
            (abs(I12 - I21) <= atol && abs(I13 - I31) <= atol && abs(I23 - I32) <= atol) ||
                error("$ctx.inertia_kgm2 must be symmetric (I12==I21, I13==I31, I23==I32)")
            inertia_diag = vec3(I11, I22, I33)
            inertia_prod = vec3(I12, I13, I23)
        else
            error(
                "$ctx.inertia_kgm2 must be either a 3x3 nested array or a flat 9-element array",
            )
        end
    else
        inertia_diag =
            haskey(tbl, "inertia_diag_kgm2") ?
            _parse_vec3(tbl["inertia_diag_kgm2"], "$ctx.inertia_diag_kgm2") :
            base.inertia_diag_kgm2
        inertia_prod =
            haskey(tbl, "inertia_products_kgm2") ?
            _parse_vec3(tbl["inertia_products_kgm2"], "$ctx.inertia_products_kgm2") :
            base.inertia_products_kgm2
    end

    rotor_pos = base.rotor_pos_body_m
    if haskey(tbl, "rotor_pos_body_m")
        arr = _as_array(tbl["rotor_pos_body_m"], "$ctx.rotor_pos_body_m")
        rotor_pos =
            Vec3[_parse_vec3(arr[i], "$ctx.rotor_pos_body_m[$i]") for i = 1:length(arr)]
    end

    rotor_axis = base.rotor_axis_body_m
    if haskey(tbl, "rotor_axis_body_m")
        arr = _as_array(tbl["rotor_axis_body_m"], "$ctx.rotor_axis_body_m")
        rotor_axis =
            Vec3[_parse_vec3(arr[i], "$ctx.rotor_axis_body_m[$i]") for i = 1:length(arr)]
    end

    linear_drag =
        haskey(tbl, "linear_drag") ? _as_f64(tbl["linear_drag"], "$ctx.linear_drag") :
        base.linear_drag
    ang_damp =
        haskey(tbl, "angular_damping") ?
        _parse_vec3(tbl["angular_damping"], "$ctx.angular_damping") : base.angular_damping

    x0 = base.x0
    if haskey(tbl, "x0")
        x0 = _parse_rigid_body_state(tbl["x0"]; strict = strict, ctx = "$ctx.x0")
    end

    prop = base.propulsion
    if haskey(tbl, "propulsion")
        p = _as_table(tbl["propulsion"], "$ctx.propulsion")
        strict && _known_keys!(
            p,
            Set([
                "kind",
                "km_m",
                "V_nom",
                "rho_nom",
                "rotor_radius_m",
                "inflow_kT",
                "inflow_kQ",
                "rotor_dir",
                "thrust_calibration_mult",
                "esc",
                "motor",
            ]),
            "$ctx.propulsion",
        )
        esc_tbl = haskey(p, "esc") ? _as_table(p["esc"], "$ctx.propulsion.esc") : nothing
        if esc_tbl !== nothing
            strict && _known_keys!(esc_tbl, Set(["eta", "deadzone"]), "$ctx.propulsion.esc")
        end
        motor_tbl =
            haskey(p, "motor") ? _as_table(p["motor"], "$ctx.propulsion.motor") : nothing
        if motor_tbl !== nothing
            strict && _known_keys!(
                motor_tbl,
                Set([
                    "kv_rpm_per_volt",
                    "r_ohm",
                    "j_kgm2",
                    "i0_a",
                    "viscous_friction_nm_per_rad_s",
                    "max_current_a",
                ]),
                "$ctx.propulsion.motor",
            )
        end
        prop = PropulsionSpec(
            kind = haskey(p, "kind") ?
                   _sym(_as_string(p["kind"], "$ctx.propulsion.kind")) : prop.kind,
            km_m = haskey(p, "km_m") ? _as_f64(p["km_m"], "$ctx.propulsion.km_m") :
                   prop.km_m,
            V_nom = haskey(p, "V_nom") ? _as_f64(p["V_nom"], "$ctx.propulsion.V_nom") :
                    prop.V_nom,
            rho_nom = haskey(p, "rho_nom") ?
                      _as_f64(p["rho_nom"], "$ctx.propulsion.rho_nom") : prop.rho_nom,
            rotor_radius_m = haskey(p, "rotor_radius_m") ?
                             _as_f64(
                p["rotor_radius_m"],
                "$ctx.propulsion.rotor_radius_m",
            ) : prop.rotor_radius_m,
            inflow_kT = haskey(p, "inflow_kT") ?
                        _as_f64(p["inflow_kT"], "$ctx.propulsion.inflow_kT") :
                        prop.inflow_kT,
            inflow_kQ = haskey(p, "inflow_kQ") ?
                        _as_f64(p["inflow_kQ"], "$ctx.propulsion.inflow_kQ") :
                        prop.inflow_kQ,
            esc_eta = esc_tbl !== nothing && haskey(esc_tbl, "eta") ?
                      _as_f64(esc_tbl["eta"], "$ctx.propulsion.esc.eta") : prop.esc_eta,
            esc_deadzone = esc_tbl !== nothing && haskey(esc_tbl, "deadzone") ?
                           _as_f64(esc_tbl["deadzone"], "$ctx.propulsion.esc.deadzone") : prop.esc_deadzone,
            motor_kv_rpm_per_volt = motor_tbl !== nothing &&
                                    haskey(motor_tbl, "kv_rpm_per_volt") ?
                                    _as_f64(
                motor_tbl["kv_rpm_per_volt"],
                "$ctx.propulsion.motor.kv_rpm_per_volt",
            ) : prop.motor_kv_rpm_per_volt,
            motor_r_ohm = motor_tbl !== nothing && haskey(motor_tbl, "r_ohm") ?
                          _as_f64(motor_tbl["r_ohm"], "$ctx.propulsion.motor.r_ohm") :
                          prop.motor_r_ohm,
            motor_j_kgm2 = motor_tbl !== nothing && haskey(motor_tbl, "j_kgm2") ?
                           _as_f64(motor_tbl["j_kgm2"], "$ctx.propulsion.motor.j_kgm2") : prop.motor_j_kgm2,
            motor_i0_a = motor_tbl !== nothing && haskey(motor_tbl, "i0_a") ?
                         _as_f64(motor_tbl["i0_a"], "$ctx.propulsion.motor.i0_a") :
                         prop.motor_i0_a,
            motor_viscous_friction_nm_per_rad_s = motor_tbl !== nothing && haskey(
                motor_tbl,
                "viscous_friction_nm_per_rad_s",
            ) ?
                                                  _as_f64(
                motor_tbl["viscous_friction_nm_per_rad_s"],
                "$ctx.propulsion.motor.viscous_friction_nm_per_rad_s",
            ) : prop.motor_viscous_friction_nm_per_rad_s,
            motor_max_current_a = motor_tbl !== nothing &&
                                  haskey(motor_tbl, "max_current_a") ?
                                  _as_f64(
                motor_tbl["max_current_a"],
                "$ctx.propulsion.motor.max_current_a",
            ) : prop.motor_max_current_a,
            thrust_calibration_mult = haskey(p, "thrust_calibration_mult") ?
                                      _as_f64(
                p["thrust_calibration_mult"],
                "$ctx.propulsion.thrust_calibration_mult",
            ) : prop.thrust_calibration_mult,
            rotor_dir = haskey(p, "rotor_dir") ?
                        [
                Float64(v) for v in _as_array(p["rotor_dir"], "$ctx.propulsion.rotor_dir")
            ] : prop.rotor_dir,
        )
    end

    return AirframeSpec(
        kind = kind,
        mass_kg = mass_kg,
        inertia_diag_kgm2 = inertia_diag,
        inertia_products_kgm2 = inertia_prod,
        rotor_pos_body_m = rotor_pos,
        rotor_axis_body_m = rotor_axis,
        linear_drag = linear_drag,
        angular_damping = ang_damp,
        x0 = x0,
        propulsion = prop,
    )
end


function _parse_actuation(tbl_any; strict::Bool, ctx::AbstractString, base::ActuationSpec)
    tbl = _as_table(tbl_any, ctx)
    strict && _known_keys!(
        tbl,
        Set(["motors", "servos", "motor_actuators", "servo_actuators"]),
        ctx,
    )

    motors = base.motors
    if haskey(tbl, "motors")
        arr = _as_array(tbl["motors"], "$ctx.motors")
        motors = MotorSpec[]
        for (i, m_any) in enumerate(arr)
            m = _as_table(m_any, "$ctx.motors[$i]")
            strict && _known_keys!(m, Set(["id", "channel"]), "$ctx.motors[$i]")
            id = _sym(_as_string(get(m, "id", nothing), "$ctx.motors[$i].id"))
            ch = _as_int(get(m, "channel", nothing), "$ctx.motors[$i].channel")
            push!(motors, MotorSpec(id = id, channel = ch))
        end
    end

    servos = base.servos
    if haskey(tbl, "servos")
        arr = _as_array(tbl["servos"], "$ctx.servos")
        servos = ServoSpec[]
        for (i, s_any) in enumerate(arr)
            s = _as_table(s_any, "$ctx.servos[$i]")
            strict && _known_keys!(s, Set(["id", "channel"]), "$ctx.servos[$i]")
            id = _sym(_as_string(get(s, "id", nothing), "$ctx.servos[$i].id"))
            ch = _as_int(get(s, "channel", nothing), "$ctx.servos[$i].channel")
            push!(servos, ServoSpec(id = id, channel = ch))
        end
    end

    motor_act =
        haskey(tbl, "motor_actuators") ?
        _parse_actuator_model(
            tbl["motor_actuators"];
            strict = strict,
            ctx = "$ctx.motor_actuators",
        ) : base.motor_actuators
    servo_act =
        haskey(tbl, "servo_actuators") ?
        _parse_actuator_model(
            tbl["servo_actuators"];
            strict = strict,
            ctx = "$ctx.servo_actuators",
        ) : base.servo_actuators

    return ActuationSpec(
        motors = motors,
        servos = servos,
        motor_actuators = motor_act,
        servo_actuators = servo_act,
    )
end


function _parse_power(tbl_any; strict::Bool, ctx::AbstractString, base::PowerSpec)
    tbl = _as_table(tbl_any, ctx)
    strict && _known_keys!(tbl, Set(["batteries", "buses", "share_mode"]), ctx)

    bats = base.batteries
    if haskey(tbl, "batteries")
        arr = _as_array(tbl["batteries"], "$ctx.batteries")
        bats = BatterySpec[]
        for (i, b_any) in enumerate(arr)
            b = _as_table(b_any, "$ctx.batteries[$i]")
            allowed = Set([
                "id",
                "model",
                "capacity_ah",
                "soc0",
                "ocv_soc",
                "ocv_v",
                "r0",
                "r1",
                "c1",
                "v1_0",
                "min_voltage_v",
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
            push!(
                bats,
                BatterySpec(
                    id = id,
                    model = model,
                    capacity_ah = haskey(b, "capacity_ah") ?
                                  _as_f64(
                        b["capacity_ah"],
                        "$ctx.batteries[$i].capacity_ah",
                    ) : 5.0,
                    soc0 = haskey(b, "soc0") ?
                           _as_f64(b["soc0"], "$ctx.batteries[$i].soc0") : 1.0,
                    ocv_soc = ocv_soc,
                    ocv_v = ocv_v,
                    r0 = haskey(b, "r0") ? _as_f64(b["r0"], "$ctx.batteries[$i].r0") :
                         0.020,
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
                    low_thr = haskey(b, "low_thr") ?
                              _as_f64(b["low_thr"], "$ctx.batteries[$i].low_thr") : 0.15,
                    crit_thr = haskey(b, "crit_thr") ?
                               _as_f64(b["crit_thr"], "$ctx.batteries[$i].crit_thr") : 0.10,
                    emerg_thr = haskey(b, "emerg_thr") ?
                                _as_f64(b["emerg_thr"], "$ctx.batteries[$i].emerg_thr") :
                                0.05,
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


function _parse_sensors(tbl_any; strict::Bool, ctx::AbstractString)
    # Support both:
    #   [[sensors]] kind="gps" ...   (top-level array)
    # and:
    #   [sensors]
    #   sensors = [ ... ]            (table wrapping)
    #   or
    #   [[sensors.sensors]] ...      (array under [sensors])

    arr_any = nothing
    if tbl_any isa AbstractVector
        arr_any = tbl_any
    else
        tbl = _as_table(tbl_any, ctx)
        strict && _known_keys!(tbl, Set(["sensors"]), ctx)
        arr_any = get(tbl, "sensors", nothing)
    end

    arr_any === nothing && return AbstractSensorSpec[]
    arr = _as_array(arr_any, ctx)
    out = AbstractSensorSpec[]
    for (i, s_any) in enumerate(arr)
        s = _as_table(s_any, "$ctx[$i]")
        strict && _known_keys!(s, Set(["kind", "id", "uorb_instance"]), "$ctx[$i]")
        kind = lowercase(_as_string(get(s, "kind", nothing), "$ctx[$i].kind"))
        id = _sym(_as_string(get(s, "id", nothing), "$ctx[$i].id"))
        inst =
            haskey(s, "uorb_instance") ?
            _as_int(s["uorb_instance"], "$ctx[$i].uorb_instance") : 0
        if kind == "gps"
            push!(out, GpsSpec(id = id, uorb_instance = inst))
        elseif kind in ("rangefinder", "distance")
            push!(out, RangefinderSpec(id = id, uorb_instance = inst))
        elseif kind == "radar"
            push!(out, RadarSpec(id = id, uorb_instance = inst))
        else
            error(
                "$ctx[$i].kind: unknown sensor kind '$kind' (expected gps|rangefinder|radar)",
            )
        end
    end
    return out
end


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

"""Parse an `AircraftSpec` from a TOML dictionary.

`base_dir` is used to resolve relative paths. If `base_spec` is provided, it
supplies defaults for any fields not present in the TOML.
"""
function spec_from_toml_dict(
    cfg_any::AbstractDict;
    base_dir::AbstractString = ".",
    strict::Bool = true,
    base_spec::Union{Nothing,AircraftSpec} = nothing,
)
    cfg = Dict{String,Any}(String(k) => v for (k, v) in cfg_any)

    # Top-level key guard.
    top_allowed = Set([
        "schema_version",
        "extends",
        "aircraft",
        "home",
        "px4",
        "timeline",
        "environment",
        "scenario",
        "estimator",
        "plant",
        "airframe",
        "actuation",
        "power",
        "sensors",
        "run",
    ])
    strict && _known_keys!(cfg, top_allowed, "root")

    schema = get(cfg, "schema_version", 1)
    _as_int(schema, "schema_version") == 1 ||
        error("Unsupported schema_version=$schema (expected 1)")

    # Optional name/seed metadata.
    name_sym = :aircraft
    atbl = nothing
    if haskey(cfg, "aircraft")
        atbl = _as_table(cfg["aircraft"], "aircraft")
        strict && _known_keys!(atbl, Set(["name", "seed"]), "aircraft")
        if haskey(atbl, "name")
            name_sym = _sym(_as_string(atbl["name"], "aircraft.name"))
        end
    end

    # Start from provided base spec (or default struct values).
    spec =
        base_spec === nothing ? AircraftSpec(name = name_sym) :
        AircraftSpec(
            name = name_sym,
            px4 = base_spec.px4,
            timeline = base_spec.timeline,
            environment = base_spec.environment,
            scenario = base_spec.scenario,
            estimator = base_spec.estimator,
            plant = base_spec.plant,
            airframe = base_spec.airframe,
            actuation = base_spec.actuation,
            power = base_spec.power,
            sensors = base_spec.sensors,
            seed = base_spec.seed,
            home = base_spec.home,
            telemetry = base_spec.telemetry,
            log_sinks = base_spec.log_sinks,
        )
    if atbl !== nothing && haskey(atbl, "seed")
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = _as_int(atbl["seed"], "aircraft.seed"),
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- home ---
    if haskey(cfg, "home")
        h = _as_table(cfg["home"], "home")
        strict && _known_keys!(h, Set(["lat_deg", "lon_deg", "alt_msl_m"]), "home")
        home = Autopilots.HomeLocation(
            lat_deg = haskey(h, "lat_deg") ? _as_f64(h["lat_deg"], "home.lat_deg") :
                      spec.home.lat_deg,
            lon_deg = haskey(h, "lon_deg") ? _as_f64(h["lon_deg"], "home.lon_deg") :
                      spec.home.lon_deg,
            alt_msl_m = haskey(h, "alt_msl_m") ?
                        _as_f64(h["alt_msl_m"], "home.alt_msl_m") : spec.home.alt_msl_m,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- px4 ---
    if haskey(cfg, "px4")
        p = _as_table(cfg["px4"], "px4")
        allowed = Set([
            "mission_path",
            "libpath",
            "edge_trigger",
            "derive_ca_params",
            "lockstep",
            "uorb",
            "params",
        ])
        strict && _known_keys!(p, allowed, "px4")

        mission_path =
            haskey(p, "mission_path") ?
            _resolve_path(
                base_dir,
                _as_string(p["mission_path"], "px4.mission_path");
                strict = strict,
            ) : spec.px4.mission_path
        libpath =
            haskey(p, "libpath") ?
            _resolve_path(
                base_dir,
                _as_string(p["libpath"], "px4.libpath");
                strict = strict,
            ) : spec.px4.libpath

        lockstep_cfg = spec.px4.lockstep_config
        if haskey(p, "lockstep")
            lockstep_cfg = _parse_lockstep_cfg(
                p["lockstep"];
                strict = strict,
                ctx = "px4.lockstep",
                base = lockstep_cfg,
            )
        end

        uorb_cfg = spec.px4.uorb_cfg
        if haskey(p, "uorb")
            uorb_cfg = _parse_uorb_interface(p["uorb"]; strict = strict, ctx = "px4.uorb")
        end

        params =
            haskey(p, "params") ?
            _parse_px4_params(p["params"]; strict = strict, ctx = "px4.params") :
            spec.px4.params
        derive_ca =
            haskey(p, "derive_ca_params") ?
            _as_bool(p["derive_ca_params"], "px4.derive_ca_params") :
            spec.px4.derive_ca_params
        edge_trigger =
            haskey(p, "edge_trigger") ? _as_bool(p["edge_trigger"], "px4.edge_trigger") :
            spec.px4.edge_trigger

        px4 = PX4Spec(
            mission_path = mission_path,
            libpath = libpath,
            lockstep_config = lockstep_cfg,
            uorb_cfg = uorb_cfg,
            params = params,
            derive_ca_params = derive_ca,
            edge_trigger = edge_trigger,
        )

        spec = AircraftSpec(
            name = spec.name,
            px4 = px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- timeline ---
    if haskey(cfg, "timeline")
        t = _as_table(cfg["timeline"], "timeline")
        strict && _known_keys!(
            t,
            Set(["t_end_s", "dt_autopilot_s", "dt_wind_s", "dt_log_s", "dt_phys_s"]),
            "timeline",
        )
        timeline = TimelineSpec(
            t_end_s = haskey(t, "t_end_s") ? _as_f64(t["t_end_s"], "timeline.t_end_s") :
                      spec.timeline.t_end_s,
            dt_autopilot_s = haskey(t, "dt_autopilot_s") ?
                             _as_f64(t["dt_autopilot_s"], "timeline.dt_autopilot_s") :
                             spec.timeline.dt_autopilot_s,
            dt_wind_s = haskey(t, "dt_wind_s") ?
                        _as_f64(t["dt_wind_s"], "timeline.dt_wind_s") :
                        spec.timeline.dt_wind_s,
            dt_log_s = haskey(t, "dt_log_s") ?
                       _as_f64(t["dt_log_s"], "timeline.dt_log_s") :
                       spec.timeline.dt_log_s,
            dt_phys_s = haskey(t, "dt_phys_s") ?
                        (
                t["dt_phys_s"] === nothing ? nothing :
                _as_f64(t["dt_phys_s"], "timeline.dt_phys_s")
            ) : spec.timeline.dt_phys_s,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- environment ---
    if haskey(cfg, "environment")
        env = _parse_environment(
            cfg["environment"];
            strict = strict,
            ctx = "environment",
            base = spec.environment,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = env,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- scenario ---
    if haskey(cfg, "scenario")
        scn = _parse_scenario(
            cfg["scenario"];
            strict = strict,
            ctx = "scenario",
            base = spec.scenario,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = scn,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- estimator ---
    if haskey(cfg, "estimator")
        est = _parse_estimator(
            cfg["estimator"];
            strict = strict,
            ctx = "estimator",
            base = spec.estimator,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = est,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- plant ---
    if haskey(cfg, "plant")
        p = _as_table(cfg["plant"], "plant")
        strict && _known_keys!(p, Set(["integrator", "contact"]), "plant")
        integ =
            haskey(p, "integrator") ?
            _parse_integrator(p["integrator"], "plant.integrator"; strict = strict) :
            spec.plant.integrator
        contact =
            haskey(p, "contact") ?
            _parse_contact(p["contact"]; strict = strict, ctx = "plant.contact") :
            spec.plant.contact
        plant = PlantSpec(integrator = integ, contact = contact)
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- airframe ---
    if haskey(cfg, "airframe")
        af = _parse_airframe(
            cfg["airframe"];
            strict = strict,
            ctx = "airframe",
            base = spec.airframe,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = af,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- actuation ---
    if haskey(cfg, "actuation")
        act = _parse_actuation(
            cfg["actuation"];
            strict = strict,
            ctx = "actuation",
            base = spec.actuation,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = act,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- power ---
    if haskey(cfg, "power")
        pw = _parse_power(cfg["power"]; strict = strict, ctx = "power", base = spec.power)
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = pw,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # --- sensors ---
    if haskey(cfg, "sensors")
        # Allow either [sensors] table (with sensors=[...]) or [[sensors]] array at top-level.
        sens = _parse_sensors(cfg["sensors"]; strict = strict, ctx = "sensors")
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = spec.airframe,
            actuation = spec.actuation,
            power = spec.power,
            sensors = sens,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    # Convenience: if rotor axes not specified, default to classic multirotor axes.
    # This helps keep TOMLs shorter while preserving the required axis fields.
    if isempty(spec.airframe.rotor_axis_body_m) && !isempty(spec.airframe.rotor_pos_body_m)
        N = length(spec.airframe.rotor_pos_body_m)
        af = AirframeSpec(
            kind = spec.airframe.kind,
            mass_kg = spec.airframe.mass_kg,
            inertia_diag_kgm2 = spec.airframe.inertia_diag_kgm2,
            inertia_products_kgm2 = spec.airframe.inertia_products_kgm2,
            rotor_pos_body_m = spec.airframe.rotor_pos_body_m,
            rotor_axis_body_m = Vec3[vec3(0.0, 0.0, 1.0) for _ = 1:N],
            linear_drag = spec.airframe.linear_drag,
            angular_damping = spec.airframe.angular_damping,
            x0 = spec.airframe.x0,
            propulsion = spec.airframe.propulsion,
        )
        spec = AircraftSpec(
            name = spec.name,
            px4 = spec.px4,
            timeline = spec.timeline,
            environment = spec.environment,
            scenario = spec.scenario,
            estimator = spec.estimator,
            plant = spec.plant,
            airframe = af,
            actuation = spec.actuation,
            power = spec.power,
            sensors = spec.sensors,
            seed = spec.seed,
            home = spec.home,
            telemetry = spec.telemetry,
            log_sinks = spec.log_sinks,
        )
    end

    return spec
end


function _default_multirotor_spec_cached(; strict::Bool)
    spec = _DEFAULT_MULTIROTOR_SPEC_CACHE[]
    spec !== nothing && return spec
    cfg = _load_toml_with_extends(_DEFAULT_MULTIROTOR_SPEC_PATH; strict = strict)
    base_dir = dirname(abspath(_DEFAULT_MULTIROTOR_SPEC_PATH))
    spec = spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict)
    _DEFAULT_MULTIROTOR_SPEC_CACHE[] = spec
    return spec
end

function _resolve_base_spec(path::AbstractString, base_spec; strict::Bool)
    if base_spec === :default
        abspath(path) == abspath(_DEFAULT_MULTIROTOR_SPEC_PATH) && return nothing
        return _default_multirotor_spec_cached(strict = strict)
    elseif base_spec === nothing || base_spec isa AircraftSpec
        return base_spec
    end
    error("base_spec must be :default, nothing, or an AircraftSpec")
end

"""Load an `AircraftSpec` from a TOML file path.

By default this is strict and **does not** apply internal defaults. To explicitly
layer the built-in generic multirotor defaults, pass `base_spec=:default`.
"""
function load_spec(path::AbstractString; strict::Bool = true, base_spec = nothing)
    cfg = _load_toml_with_extends(path; strict = strict)
    base_dir = dirname(abspath(path))
    base = _resolve_base_spec(path, base_spec; strict = strict)
    return spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict, base_spec = base)
end


"""Load, parse, and run an aircraft spec TOML.

This is a convenience wrapper around:
  `load_spec(path) |> build_engine(spec; ...)`

The optional `[run]` table can provide defaults for mode/record paths.
Passing explicit `mode`/`recording_*` arguments overrides the `[run]` section.
"""
function run_spec(
    path::AbstractString;
    mode = nothing,
    recording_in = nothing,
    recording_out = nothing,
    strict::Bool = true,
    base_spec = nothing,
    telemetry = nothing,
    log_sinks = nothing,
)
    cfg = _load_toml_with_extends(path; strict = strict)
    base_dir = dirname(abspath(path))
    base = _resolve_base_spec(path, base_spec; strict = strict)
    spec = spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict, base_spec = base)

    # Read run defaults.
    run_tbl = get(cfg, "run", Dict{String,Any}())
    if !(run_tbl isa AbstractDict)
        error("run must be a table")
    end
    strict && _known_keys!(
        run_tbl,
        Set(["mode", "recording_in", "recording_out", "log_csv"]),
        "run",
    )

    mode_sym = if mode === nothing
        if haskey(run_tbl, "mode")
            Symbol(lowercase(_as_string(run_tbl["mode"], "run.mode")))
        else
            :live
        end
    else
        if mode isa Symbol
            mode
        elseif mode isa AbstractString
            Symbol(lowercase(String(mode)))
        else
            error("run_spec: mode must be Symbol or String (got $(typeof(mode)))")
        end
    end
    mode_sym in (:live, :record, :replay) || error("run_spec: unknown mode=$mode_sym")

    if mode_sym !== :replay && spec.px4.libpath === nothing
        error("px4.libpath is required for live/record runs; set it in the TOML.")
    end

    rec_in = recording_in
    if rec_in === nothing && haskey(run_tbl, "recording_in")
        rec_in = _resolve_path(
            base_dir,
            _as_string(run_tbl["recording_in"], "run.recording_in");
            strict = strict,
        )
    end

    rec_out = recording_out
    if rec_out === nothing && haskey(run_tbl, "recording_out")
        rec_out = _resolve_path(
            base_dir,
            _as_string(run_tbl["recording_out"], "run.recording_out");
            strict = strict,
        )
    end

    tel = telemetry === nothing ? spec.telemetry : telemetry
    logs = log_sinks === nothing ? spec.log_sinks : log_sinks
    if logs === nothing && haskey(run_tbl, "log_csv")
        log_path = _resolve_path(
            base_dir,
            _as_string(run_tbl["log_csv"], "run.log_csv");
            strict = strict,
        )
        log_path === nothing || (logs = Logging.CSVLogSink(log_path))
    end

    return build_engine(
        spec;
        mode = mode_sym,
        recording_in = rec_in,
        recording_out = rec_out,
        telemetry = tel,
        log_sinks = logs,
    )
end
