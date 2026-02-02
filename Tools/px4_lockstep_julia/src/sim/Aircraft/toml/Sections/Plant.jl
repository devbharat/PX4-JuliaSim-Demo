function _parse_contact(tbl_any; strict::Bool, ctx::AbstractString)
    if tbl_any isa AbstractString
        k = lowercase(String(tbl_any))
        if k in ("none", "no", "off", "no_contact", "nocontact")
            return Contacts.NoContact()
        elseif k in ("flat", "flat_ground", "ground", "flat_ground_penalty", "penalty")
            return Contacts.FlatGroundContact()
        elseif k in
               ("flat_ground_constraint", "constraint", "unilateral", "flat_constraint")
            return Contacts.FlatGroundConstraintContact()
        else
            error(
                "$ctx: unknown contact kind '$tbl_any' (expected 'flat_ground', 'flat_ground_constraint', or 'no_contact')",
            )
        end
    end

    tbl = _as_table(tbl_any, ctx)
    # Allow ASCII alias `mu` for convenience.
    strict && _known_keys!(
        tbl,
        Set([
            "kind",
            # penalty
            "k_n_per_m",
            "c_n_per_mps",
            # shared
            "μ",
            "mu",
            "v_eps",
            "enable_friction",
            # constraint
            "z_slop_m",
            "vz_slop_mps",
            # hybrid impact (TOI + impact map)
            "restitution",
            "e",
            "v_rest_threshold_mps",
            "enable_impact_friction",
            # grounded time-stepping
            "h_contact_us",
        ]),
        ctx,
    )
    kind = lowercase(String(get(tbl, "kind", "flat_ground")))
    if kind in ("flat", "flat_ground", "ground", "flat_ground_penalty", "penalty")
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
    elseif kind in ("flat_ground_constraint", "constraint", "unilateral", "flat_constraint")
        z_slop_m = _as_f64(get(tbl, "z_slop_m", 1e-6), "$ctx.z_slop_m")
        vz_slop_mps = _as_f64(get(tbl, "vz_slop_mps", 1e-3), "$ctx.vz_slop_mps")
        vz_slop_mps >= 0.0 || error("$ctx.vz_slop_mps must be >= 0")
        return Contacts.FlatGroundConstraintContact(
            μ = _as_f64(get(tbl, haskey(tbl, "μ") ? "μ" : "mu", 0.8), "$ctx.mu"),
            v_eps = _as_f64(get(tbl, "v_eps", 0.05), "$ctx.v_eps"),
            enable_friction = _as_bool(
                get(tbl, "enable_friction", true),
                "$ctx.enable_friction",
            ),
            restitution = _as_f64(
                get(tbl, haskey(tbl, "e") ? "e" : "restitution", 0.0),
                "$ctx.restitution",
            ),
            v_rest_threshold_mps = _as_f64(
                get(tbl, "v_rest_threshold_mps", 0.05),
                "$ctx.v_rest_threshold_mps",
            ),
            enable_impact_friction = _as_bool(
                get(tbl, "enable_impact_friction", true),
                "$ctx.enable_impact_friction",
            ),
            h_contact_us = UInt64(
                max(1, _as_int(get(tbl, "h_contact_us", 1000), "$ctx.h_contact_us")),
            ),
            z_slop_m = z_slop_m,
            vz_slop_mps = vz_slop_mps,
        )
    elseif kind in ("none", "no", "off", "no_contact", "nocontact")
        return Contacts.NoContact()
    else
        error(
            "$ctx: unknown contact kind '$kind' (expected 'flat_ground', 'flat_ground_constraint', or 'no_contact')",
        )
    end
end

function _parse_adaptive_integrator(
    kind_sym::Symbol,
    tbl,
    ctx::AbstractString;
    strict::Bool,
)
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
        "rtol_temp",
        "atol_temp",
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

    integ = kind_sym === :RK23 ? Integrators.RK23Integrator() : Integrators.RK45Integrator()
    rtol_pos = _as_f64(get(tbl, "rtol_pos", integ.rtol_pos), "$ctx.rtol_pos")
    atol_pos = _as_f64(get(tbl, "atol_pos", integ.atol_pos), "$ctx.atol_pos")
    rtol_vel = _as_f64(get(tbl, "rtol_vel", integ.rtol_vel), "$ctx.rtol_vel")
    atol_vel = _as_f64(get(tbl, "atol_vel", integ.atol_vel), "$ctx.atol_vel")
    rtol_ω =
        haskey(tbl, "rtol_ω") ? _as_f64(tbl["rtol_ω"], "$ctx.rtol_ω") :
        (
            haskey(tbl, "rtol_omega") ? _as_f64(tbl["rtol_omega"], "$ctx.rtol_omega") :
            integ.rtol_ω
        )
    atol_ω =
        haskey(tbl, "atol_ω") ? _as_f64(tbl["atol_ω"], "$ctx.atol_ω") :
        (
            haskey(tbl, "atol_omega") ? _as_f64(tbl["atol_omega"], "$ctx.atol_omega") :
            integ.atol_ω
        )
    atol_att_rad =
        _as_f64(get(tbl, "atol_att_rad", integ.atol_att_rad), "$ctx.atol_att_rad")
    plant_error_control = _as_bool(
        get(tbl, "plant_error_control", integ.plant_error_control),
        "$ctx.plant_error_control",
    )

    rtol_act = _as_f64(get(tbl, "rtol_act", integ.rtol_act), "$ctx.rtol_act")
    atol_act = _as_f64(get(tbl, "atol_act", integ.atol_act), "$ctx.atol_act")
    rtol_actdot = _as_f64(get(tbl, "rtol_actdot", integ.rtol_actdot), "$ctx.rtol_actdot")
    atol_actdot = _as_f64(get(tbl, "atol_actdot", integ.atol_actdot), "$ctx.atol_actdot")
    rtol_rotor = _as_f64(get(tbl, "rtol_rotor", integ.rtol_rotor), "$ctx.rtol_rotor")
    atol_rotor = _as_f64(get(tbl, "atol_rotor", integ.atol_rotor), "$ctx.atol_rotor")
    rtol_soc = _as_f64(get(tbl, "rtol_soc", integ.rtol_soc), "$ctx.rtol_soc")
    atol_soc = _as_f64(get(tbl, "atol_soc", integ.atol_soc), "$ctx.atol_soc")
    rtol_v1 = _as_f64(get(tbl, "rtol_v1", integ.rtol_v1), "$ctx.rtol_v1")
    atol_v1 = _as_f64(get(tbl, "atol_v1", integ.atol_v1), "$ctx.atol_v1")
    rtol_temp = _as_f64(get(tbl, "rtol_temp", integ.rtol_temp), "$ctx.rtol_temp")
    atol_temp = _as_f64(get(tbl, "atol_temp", integ.atol_temp), "$ctx.atol_temp")

    h_min = _as_f64(get(tbl, "h_min", integ.h_min), "$ctx.h_min")
    h_max = _as_f64(get(tbl, "h_max", integ.h_max), "$ctx.h_max")
    h_init = _as_f64(get(tbl, "h_init", integ.h_init), "$ctx.h_init")
    max_substeps =
        Int(_as_int(get(tbl, "max_substeps", integ.max_substeps), "$ctx.max_substeps"))
    safety = _as_f64(get(tbl, "safety", integ.safety), "$ctx.safety")
    min_factor = _as_f64(get(tbl, "min_factor", integ.min_factor), "$ctx.min_factor")
    max_factor = _as_f64(get(tbl, "max_factor", integ.max_factor), "$ctx.max_factor")
    quantize_us = _as_bool(get(tbl, "quantize_us", integ.quantize_us), "$ctx.quantize_us")

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
            rtol_temp = rtol_temp,
            atol_temp = atol_temp,
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
        rtol_temp = rtol_temp,
        atol_temp = atol_temp,
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

function _parse_integrator(x, ctx::AbstractString; strict::Bool)
    if x isa Symbol
        return x
    elseif x isa AbstractString
        kind_str = lowercase(_as_string(x, ctx))
        kind_sym = Symbol(uppercase(kind_str))
        if kind_sym === :EULER
            return :Euler
        elseif kind_sym === :RK4 || kind_sym === :RK23 || kind_sym === :RK45
            return kind_sym
        end
        error("$ctx must be one of 'Euler', 'RK4', 'RK23', 'RK45' (got '$kind_str')")
    elseif x isa AbstractDict
        tbl = _as_table(x, ctx)
        kind_raw = get(tbl, "kind", nothing)
        kind_raw === nothing && error("$ctx.kind is required (e.g. 'RK45')")
        kind_str = lowercase(_as_string(kind_raw, "$ctx.kind"))
        kind_sym = Symbol(uppercase(kind_str))

        if kind_sym === :RK23 || kind_sym === :RK45
            return _parse_adaptive_integrator(kind_sym, tbl, ctx; strict = strict)
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

function _parse_plant(tbl_any; strict::Bool, ctx::AbstractString, base::PlantSpec)
    p = _as_table(tbl_any, ctx)
    strict && _known_keys!(p, Set(["integrator", "contact"]), ctx)
    integ =
        haskey(p, "integrator") ?
        _parse_integrator(p["integrator"], "$ctx.integrator"; strict = strict) :
        base.integrator
    contact =
        haskey(p, "contact") ?
        _parse_contact(p["contact"]; strict = strict, ctx = "$ctx.contact") : base.contact
    return PlantSpec(integrator = integ, contact = contact)
end
