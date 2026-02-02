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

function _parse_px4(
    tbl_any;
    strict::Bool,
    ctx::AbstractString,
    base::PX4Spec,
    base_dir::AbstractString,
)
    p = _as_table(tbl_any, ctx)
    allowed = Set([
        "mission_path",
        "libpath",
        "edge_trigger",
        "derive_ca_params",
        "lockstep",
        "uorb",
        "params",
    ])
    strict && _known_keys!(p, allowed, ctx)

    mission_path =
        haskey(p, "mission_path") ?
        _resolve_path(
            base_dir,
            _as_string(p["mission_path"], "$ctx.mission_path");
            must_exist = false,
        ) : base.mission_path
    libpath =
        haskey(p, "libpath") ?
        _resolve_path(
            base_dir,
            _as_string(p["libpath"], "$ctx.libpath");
            must_exist = false,
        ) : base.libpath

    lockstep_cfg = base.lockstep_config
    if haskey(p, "lockstep")
        lockstep_cfg = _parse_lockstep_cfg(
            p["lockstep"];
            strict = strict,
            ctx = "$ctx.lockstep",
            base = lockstep_cfg,
        )
    end

    uorb_cfg = base.uorb_cfg
    if haskey(p, "uorb")
        uorb_cfg = _parse_uorb_interface(p["uorb"]; strict = strict, ctx = "$ctx.uorb")
    end

    params =
        haskey(p, "params") ?
        _parse_px4_params(p["params"]; strict = strict, ctx = "$ctx.params") : base.params
    derive_ca =
        haskey(p, "derive_ca_params") ?
        _as_bool(p["derive_ca_params"], "$ctx.derive_ca_params") : base.derive_ca_params
    edge_trigger =
        haskey(p, "edge_trigger") ? _as_bool(p["edge_trigger"], "$ctx.edge_trigger") :
        base.edge_trigger

    return PX4Spec(
        mission_path = mission_path,
        libpath = libpath,
        lockstep_config = lockstep_cfg,
        uorb_cfg = uorb_cfg,
        params = params,
        derive_ca_params = derive_ca,
        edge_trigger = edge_trigger,
    )
end
