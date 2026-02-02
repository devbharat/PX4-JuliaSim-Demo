# Small utilities

@inline _sym(x::Symbol) = x
@inline _sym(x::AbstractString) = Symbol(x)

function _as_string(x, ctx::AbstractString)
    x isa AbstractString || error("$ctx must be a string (got $(typeof(x)))")
    return String(x)
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
    must_exist::Bool = false,
)
    p === nothing && return nothing
    s = String(p)
    s = Base.expanduser(s)
    # Empty string is almost always accidental; treat as "unset".
    isempty(strip(s)) && return nothing
    resolved = isabspath(s) ? normpath(s) : normpath(joinpath(base_dir, s))
    if must_exist && !isfile(resolved)
        error("File not found: $resolved")
    end
    return resolved
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
            epath = _resolve_path(base_dir, _as_string(e, "extends[$i]"); must_exist = true)
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

@inline function _norm_kind(x, ctx::AbstractString)
    return Symbol(lowercase(_as_string(x, ctx)))
end
