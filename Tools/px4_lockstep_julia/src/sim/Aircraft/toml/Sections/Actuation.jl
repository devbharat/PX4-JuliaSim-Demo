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
        motors = MotorChannelSpec[]
        for (i, m_any) in enumerate(arr)
            m = _as_table(m_any, "$ctx.motors[$i]")
            strict && _known_keys!(m, Set(["id", "channel"]), "$ctx.motors[$i]")
            id = _sym(_as_string(get(m, "id", nothing), "$ctx.motors[$i].id"))
            ch = _as_int(get(m, "channel", nothing), "$ctx.motors[$i].channel")
            push!(motors, MotorChannelSpec(id = id, channel = ch))
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
