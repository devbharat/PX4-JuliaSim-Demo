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
        if strict &&
           (haskey(tbl, "inertia_diag_kgm2") || haskey(tbl, "inertia_products_kgm2"))
            error(
                "$ctx.inertia_kgm2 cannot be combined with inertia_diag_kgm2 or inertia_products_kgm2 in strict mode",
            )
        end
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
            abs(I12 - I21) < atol || error("$ctx.inertia_kgm2 must be symmetric")
            abs(I13 - I31) < atol || error("$ctx.inertia_kgm2 must be symmetric")
            abs(I23 - I32) < atol || error("$ctx.inertia_kgm2 must be symmetric")

            inertia_diag = vec3(I11, I22, I33)
            inertia_prod = vec3(I12, I13, I23)
        elseif length(arr) == 9 && all(x -> !(x isa AbstractVector), arr)
            I11 = _as_f64(arr[1], "$ctx.inertia_kgm2[1]")
            I12 = _as_f64(arr[2], "$ctx.inertia_kgm2[2]")
            I13 = _as_f64(arr[3], "$ctx.inertia_kgm2[3]")
            I21 = _as_f64(arr[4], "$ctx.inertia_kgm2[4]")
            I22 = _as_f64(arr[5], "$ctx.inertia_kgm2[5]")
            I23 = _as_f64(arr[6], "$ctx.inertia_kgm2[6]")
            I31 = _as_f64(arr[7], "$ctx.inertia_kgm2[7]")
            I32 = _as_f64(arr[8], "$ctx.inertia_kgm2[8]")
            I33 = _as_f64(arr[9], "$ctx.inertia_kgm2[9]")

            atol = 1e-12
            abs(I12 - I21) < atol || error("$ctx.inertia_kgm2 must be symmetric")
            abs(I13 - I31) < atol || error("$ctx.inertia_kgm2 must be symmetric")
            abs(I23 - I32) < atol || error("$ctx.inertia_kgm2 must be symmetric")

            inertia_diag = vec3(I11, I22, I33)
            inertia_prod = vec3(I12, I13, I23)
        else
            error("$ctx.inertia_kgm2 must be a 3x3 nested array")
        end
    else
        if haskey(tbl, "inertia_diag_kgm2")
            inertia_diag = _parse_vec3(tbl["inertia_diag_kgm2"], "$ctx.inertia_diag_kgm2")
        end
        if haskey(tbl, "inertia_products_kgm2")
            inertia_prod =
                _parse_vec3(tbl["inertia_products_kgm2"], "$ctx.inertia_products_kgm2")
        end
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
        esc = prop.esc
        if esc_tbl !== nothing
            esc = EscSpec(
                eta = haskey(esc_tbl, "eta") ?
                      _as_f64(esc_tbl["eta"], "$ctx.propulsion.esc.eta") : esc.eta,
                deadzone = haskey(esc_tbl, "deadzone") ?
                           _as_f64(esc_tbl["deadzone"], "$ctx.propulsion.esc.deadzone") : esc.deadzone,
            )
        end

        motor = prop.motor
        if motor_tbl !== nothing
            motor = MotorSpec(
                kv_rpm_per_volt = haskey(motor_tbl, "kv_rpm_per_volt") ?
                                  _as_f64(
                    motor_tbl["kv_rpm_per_volt"],
                    "$ctx.propulsion.motor.kv_rpm_per_volt",
                ) : motor.kv_rpm_per_volt,
                r_ohm = haskey(motor_tbl, "r_ohm") ?
                        _as_f64(motor_tbl["r_ohm"], "$ctx.propulsion.motor.r_ohm") :
                        motor.r_ohm,
                j_kgm2 = haskey(motor_tbl, "j_kgm2") ?
                         _as_f64(motor_tbl["j_kgm2"], "$ctx.propulsion.motor.j_kgm2") :
                         motor.j_kgm2,
                i0_a = haskey(motor_tbl, "i0_a") ?
                       _as_f64(motor_tbl["i0_a"], "$ctx.propulsion.motor.i0_a") :
                       motor.i0_a,
                viscous_friction_nm_per_rad_s = haskey(
                    motor_tbl,
                    "viscous_friction_nm_per_rad_s",
                ) ?
                                                _as_f64(
                    motor_tbl["viscous_friction_nm_per_rad_s"],
                    "$ctx.propulsion.motor.viscous_friction_nm_per_rad_s",
                ) : motor.viscous_friction_nm_per_rad_s,
                max_current_a = haskey(motor_tbl, "max_current_a") ?
                                _as_f64(
                    motor_tbl["max_current_a"],
                    "$ctx.propulsion.motor.max_current_a",
                ) : motor.max_current_a,
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
            esc = esc,
            motor = motor,
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
