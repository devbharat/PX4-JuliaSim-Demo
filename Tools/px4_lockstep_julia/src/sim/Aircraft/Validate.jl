"""Spec validation helpers.

Validation is intentionally lightweight and fail-fast, but it checks the
*instance graph* at a basic level (IDs, counts, simple wiring invariants).

This can be extended to validate:
* actuator → propulsor maps
* uORB instance wiring
* sensor timing + injection schedules
* multi-bus power topology
"""

"""Validate an aircraft spec for a given run mode.

Throws an ErrorException / ArgumentError on failure.
"""
function validate_spec(spec::AircraftSpec; mode::Symbol = :live, recording_in = nothing)
    mode in (:live, :record, :replay) ||
        throw(ArgumentError("Unknown mode=$mode (expected :live|:record|:replay)"))

    # Timeline sanity
    spec.timeline.t_end_s > 0 || throw(ArgumentError("t_end_s must be > 0"))
    spec.timeline.dt_autopilot_s > 0 || throw(ArgumentError("dt_autopilot_s must be > 0"))
    spec.timeline.dt_wind_s > 0 || throw(ArgumentError("dt_wind_s must be > 0"))
    spec.timeline.dt_log_s > 0 || throw(ArgumentError("dt_log_s must be > 0"))
    if spec.timeline.dt_phys_s !== nothing
        spec.timeline.dt_phys_s > 0 || throw(ArgumentError("dt_phys_s must be > 0"))
    end

    # Environment sanity
    env = spec.environment
    env.wind in (:none, :ou, :constant) ||
        throw(ArgumentError("environment.wind must be one of :none|:ou|:constant"))
    all(isfinite, (env.wind_mean_ned[1], env.wind_mean_ned[2], env.wind_mean_ned[3])) ||
        throw(ArgumentError("environment.wind_mean_ned contains non-finite values"))
    all(isfinite, (env.wind_sigma_ned[1], env.wind_sigma_ned[2], env.wind_sigma_ned[3])) ||
        throw(ArgumentError("environment.wind_sigma_ned contains non-finite values"))
    if env.wind === :ou
        env.wind_tau_s > 0 ||
            throw(ArgumentError("environment.wind_tau_s must be > 0 for OU wind"))
        (
            env.wind_sigma_ned[1] >= 0 &&
            env.wind_sigma_ned[2] >= 0 &&
            env.wind_sigma_ned[3] >= 0
        ) || throw(ArgumentError("environment.wind_sigma_ned must be >= 0 for OU wind"))
    end
    env.atmosphere === :isa1976 ||
        throw(ArgumentError("environment.atmosphere must be :isa1976"))
    env.gravity in (:uniform, :spherical) ||
        throw(ArgumentError("environment.gravity must be :uniform or :spherical"))
    if env.gravity === :uniform
        env.gravity_mps2 > 0 || throw(ArgumentError("environment.gravity_mps2 must be > 0"))
    else
        env.gravity_mu > 0 || throw(ArgumentError("environment.gravity_mu must be > 0"))
        env.gravity_r0_m > 0 || throw(ArgumentError("environment.gravity_r0_m must be > 0"))
    end

    # Scenario sanity
    function _require_us_quantized(val::Float64, name::AbstractString)
        val >= 0 || throw(ArgumentError("$name must be >= 0 (got $val)"))
        us = round(Int64, val * 1e6)
        abs(val - (Float64(us) * 1e-6)) <= 1e-15 ||
            throw(ArgumentError("$name must be an integer multiple of 1 µs (got $(val))"))
    end
    _require_us_quantized(spec.scenario.arm_time_s, "scenario.arm_time_s")
    _require_us_quantized(spec.scenario.mission_time_s, "scenario.mission_time_s")

    # Estimator sanity
    est = spec.estimator
    est.kind in (:none, :noisy_delayed) ||
        throw(ArgumentError("estimator.kind must be :none or :noisy_delayed"))
    if est.kind !== :none
        if est.dt_est_s !== nothing
            abs(est.dt_est_s - spec.timeline.dt_autopilot_s) <= 1e-12 || throw(
                ArgumentError(
                    "estimator.dt_est_s must match timeline.dt_autopilot_s " *
                    "(dt_est_s=$(est.dt_est_s), dt_autopilot_s=$(spec.timeline.dt_autopilot_s))",
                ),
            )
        end
        if est.delay_s !== nothing
            _require_us_quantized(est.delay_s, "estimator.delay_s")
        end
    end

    # -----------------------------
    # Instance graph sanity
    # -----------------------------

    # Motors
    motors = spec.actuation.motors
    isempty(motors) && throw(ArgumentError("actuation.motors must be non-empty"))
    motor_ids = [m.id for m in motors]
    length(unique(motor_ids)) == length(motor_ids) ||
        throw(ArgumentError("Duplicate motor IDs in actuation.motors"))
    motor_channels = [m.channel for m in motors]
    length(unique(motor_channels)) == length(motor_channels) || throw(
        ArgumentError(
            "Duplicate motor channels in actuation.motors (each motor should have a unique PX4 output channel)",
        ),
    )
    for m in motors
        (1 <= m.channel <= 12) ||
            throw(ArgumentError("Motor $(m.id) has channel=$(m.channel) (expected 1..12)"))
    end
    length(motors) <= 12 || throw(
        ArgumentError(
            "actuation.motors length=$(length(motors)) exceeds PX4 ABI motor channels (12)",
        ),
    )

    # Servos (optional)
    servos = spec.actuation.servos
    servo_ids = [s.id for s in servos]
    length(unique(servo_ids)) == length(servo_ids) ||
        throw(ArgumentError("Duplicate servo IDs in actuation.servos"))
    servo_channels = [s.channel for s in servos]
    length(unique(servo_channels)) == length(servo_channels) || throw(
        ArgumentError(
            "Duplicate servo channels in actuation.servos (each servo should have a unique PX4 output channel)",
        ),
    )
    for s in servos
        (1 <= s.channel <= 8) ||
            throw(ArgumentError("Servo $(s.id) has channel=$(s.channel) (expected 1..8)"))
    end

    # Airframe (multirotor only)
    spec.airframe.kind === :multirotor || throw(
        ArgumentError(
            "Unsupported airframe.kind=$(spec.airframe.kind) (supports :multirotor)",
        ),
    )
    length(spec.airframe.rotor_pos_body_m) == length(motors) || throw(
        ArgumentError(
            "airframe.rotor_pos_body_m length=$(length(spec.airframe.rotor_pos_body_m)) must match actuation.motors length=$(length(motors))",
        ),
    )

    # Inertia tensor sanity (symmetric positive-definite).
    Ixx, Iyy, Izz = spec.airframe.inertia_diag_kgm2
    Ixy, Ixz, Iyz = spec.airframe.inertia_products_kgm2
    all(isfinite, (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)) ||
        throw(ArgumentError("airframe inertia contains non-finite values"))
    Ixx > 0.0 || throw(ArgumentError("airframe inertia: Ixx must be > 0 (got $Ixx)"))
    Iyy > 0.0 || throw(ArgumentError("airframe inertia: Iyy must be > 0 (got $Iyy)"))
    Izz > 0.0 || throw(ArgumentError("airframe inertia: Izz must be > 0 (got $Izz)"))
    m2 = Ixx * Iyy - Ixy * Ixy
    m2 > 0.0 || throw(
        ArgumentError(
            "airframe inertia: leading 2x2 principal minor must be > 0 (got $m2); check Ixy",
        ),
    )
    detI =
        Ixx * (Iyy * Izz - Iyz * Iyz) - Ixy * (Ixy * Izz - Iyz * Ixz) +
        Ixz * (Ixy * Iyz - Iyy * Ixz)
    detI > 0.0 || throw(
        ArgumentError(
            "airframe inertia: determinant must be > 0 (got $detI); inertia tensor is not positive-definite",
        ),
    )

    # Per-rotor axes
    length(spec.airframe.rotor_axis_body_m) == length(motors) || throw(
        ArgumentError(
            "airframe.rotor_axis_body_m length=$(length(spec.airframe.rotor_axis_body_m)) must match actuation.motors length=$(length(motors))",
        ),
    )

    # Axis sanity: must be non-zero and finite.
    for (i, a) in enumerate(spec.airframe.rotor_axis_body_m)
        (isfinite(a[1]) && isfinite(a[2]) && isfinite(a[3])) || throw(
            ArgumentError("airframe.rotor_axis_body_m[$i] contains non-finite values: $a"),
        )
        n2 = a[1] * a[1] + a[2] * a[2] + a[3] * a[3]
        n2 > 1e-12 ||
            throw(ArgumentError("airframe.rotor_axis_body_m[$i] is near-zero: $a"))
    end

    # Propulsion rotor direction overrides (optional).
    if spec.airframe.propulsion.rotor_dir !== nothing
        rotor_dir = spec.airframe.propulsion.rotor_dir
        length(rotor_dir) == length(motors) || throw(
            ArgumentError(
                "airframe.propulsion.rotor_dir length=$(length(rotor_dir)) must match actuation.motors length=$(length(motors))",
            ),
        )
        for (i, v) in enumerate(rotor_dir)
            isfinite(v) ||
                throw(ArgumentError("airframe.propulsion.rotor_dir[$i] is not finite"))
            abs(abs(v) - 1.0) <= 1e-6 || throw(
                ArgumentError("airframe.propulsion.rotor_dir[$i] must be ±1 (got $(v))"),
            )
        end
    end

    # Power system (multi-battery + multi-bus)
    bats = spec.power.batteries
    isempty(bats) && throw(ArgumentError("power.batteries must be non-empty"))
    bat_ids = [b.id for b in bats]
    length(unique(bat_ids)) == length(bat_ids) ||
        throw(ArgumentError("Duplicate battery IDs in power.batteries"))

    buses = spec.power.buses
    isempty(buses) && throw(ArgumentError("power.buses must be non-empty"))
    bus_ids = [b.id for b in buses]
    length(unique(bus_ids)) == length(bus_ids) ||
        throw(ArgumentError("Duplicate bus IDs in power.buses"))

    motor_id_set = Set(motor_ids)
    servo_id_set = Set(servo_ids)
    bat_id_set = Set(bat_ids)

    # Topology constraints: each motor/battery must map to exactly one bus.
    motor_bus_count = Dict{MotorId,Int}(mid => 0 for mid in motor_ids)
    bat_bus_count = Dict{BatteryId,Int}(bid => 0 for bid in bat_ids)
    for bus in buses
        isempty(bus.battery_ids) &&
            throw(ArgumentError("Bus $(bus.id) must have >= 1 battery"))
        for bid in bus.battery_ids
            bid in bat_id_set ||
                throw(ArgumentError("Bus $(bus.id) references unknown battery id=$bid"))
            bat_bus_count[bid] = get(bat_bus_count, bid, 0) + 1
        end
        for mid in bus.motor_ids
            mid in motor_id_set ||
                throw(ArgumentError("Bus $(bus.id) references unknown motor id=$mid"))
            motor_bus_count[mid] = get(motor_bus_count, mid, 0) + 1
        end
        for sid in bus.servo_ids
            sid in servo_id_set ||
                throw(ArgumentError("Bus $(bus.id) references unknown servo id=$sid"))
        end
    end

    for mid in motor_ids
        c = get(motor_bus_count, mid, 0)
        c == 1 || throw(
            ArgumentError("Motor id=$mid must be assigned to exactly one bus (got $c)"),
        )
    end
    for bid in bat_ids
        c = get(bat_bus_count, bid, 0)
        c == 1 || throw(
            ArgumentError("Battery id=$bid must be assigned to exactly one bus (got $c)"),
        )
    end

    # Power share mode
    spec.power.share_mode in (:inv_r0, :equal) ||
        throw(ArgumentError("power.share_mode must be :inv_r0 or :equal"))

    if mode === :replay
        recording_in === nothing &&
            throw(ArgumentError("build_engine(mode=:replay) requires recording_in"))
    end
    if mode !== :replay
        libpath = spec.px4.libpath
        (libpath === nothing || isempty(strip(libpath))) &&
            throw(ArgumentError("px4.libpath is required for live/record runs"))
    end

    # -----------------------------
    # PX4 params
    # -----------------------------
    if !isempty(spec.px4.params)
        names = String[]
        for p in spec.px4.params
            nm = strip(p.name)
            isempty(nm) &&
                throw(ArgumentError("px4.params contains an empty parameter name"))
            push!(names, nm)
        end
        length(unique(names)) == length(names) ||
            throw(ArgumentError("Duplicate PX4 parameter names in px4.params"))
    end

    if mode !== :replay && spec.px4.uorb_cfg === nothing
        throw(ArgumentError("px4.uorb is required for live/record runs (define in TOML)"))
    end
    if mode !== :replay && spec.px4.uorb_cfg !== nothing
        cfg = spec.px4.uorb_cfg
        if isempty(cfg.pubs) && isempty(cfg.subs)
            throw(
                ArgumentError(
                    "px4.uorb must define at least one pub or sub for live/record runs",
                ),
            )
        end
    end

    return nothing
end
