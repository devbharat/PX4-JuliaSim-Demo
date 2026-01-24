"""Spec validation helpers.

Phase 1 keeps validation intentionally lightweight and fail-fast, but it now checks
the *instance graph* at a basic level (IDs, counts, simple wiring invariants).

Later phases will extend this to validate:
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

    # -----------------------------
    # Instance graph sanity (Phase 1)
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

    # Airframe (Phase 2: generic multirotor, Iris remains a preset)
    if spec.name === :iris
        spec.airframe.kind === :iris_quadrotor ||
            throw(ArgumentError("Iris spec requires airframe.kind=:iris_quadrotor"))
        length(motors) == 4 || throw(ArgumentError("Iris spec requires exactly 4 motors"))
        length(spec.airframe.rotor_pos_body_m) == 4 ||
            throw(ArgumentError("Iris spec requires 4 rotor_pos_body_m entries"))

        # Per-rotor axes (Phase 4)
        length(spec.airframe.rotor_axis_body_m) == 4 ||
            throw(ArgumentError("Iris spec requires 4 rotor_axis_body_m entries"))
    else
        spec.airframe.kind === :multirotor || throw(
            ArgumentError(
                "Non-Iris specs require airframe.kind=:multirotor (Phase 2 supports multirotor only)",
            ),
        )
        length(spec.airframe.rotor_pos_body_m) == length(motors) || throw(
            ArgumentError(
                "airframe.rotor_pos_body_m length=$(length(spec.airframe.rotor_pos_body_m)) must match actuation.motors length=$(length(motors))",
            ),
        )

        # Per-rotor axes (Phase 4)
        length(spec.airframe.rotor_axis_body_m) == length(motors) || throw(
            ArgumentError(
                "airframe.rotor_axis_body_m length=$(length(spec.airframe.rotor_axis_body_m)) must match actuation.motors length=$(length(motors))",
            ),
        )
    end

    # Axis sanity (Phase 4): if provided, must be non-zero and finite.
    for (i, a) in enumerate(spec.airframe.rotor_axis_body_m)
        (isfinite(a[1]) && isfinite(a[2]) && isfinite(a[3])) || throw(
            ArgumentError("airframe.rotor_axis_body_m[$i] contains non-finite values: $a"),
        )
        n2 = a[1] * a[1] + a[2] * a[2] + a[3] * a[3]
        n2 > 1e-12 ||
            throw(ArgumentError("airframe.rotor_axis_body_m[$i] is near-zero: $a"))
    end

    # Power system (Phase 5.2: multi-battery + multi-bus)
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

    # Phase 5.2 topology constraints: each motor/battery must map to exactly one bus.
    motor_bus_count = Dict{MotorId,Int}(mid => 0 for mid in motor_ids)
    bat_bus_count = Dict{BatteryId,Int}(bid => 0 for bid in bat_ids)
    for bus in buses
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

    if mode === :replay
        recording_in === nothing &&
            throw(ArgumentError("build_engine(mode=:replay) requires recording_in"))
    end

    # -----------------------------
    # PX4 params (Phase 3)
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

    # Live / record require a mission file for the Iris workflow.
    if mode !== :replay && spec.name === :iris
        if spec.px4.mission_path === nothing
            throw(
                ArgumentError(
                    "Iris live/record runs require mission_path (set spec.px4.mission_path or PX4_LOCKSTEP_MISSION)",
                ),
            )
        end
    end

    return nothing
end
