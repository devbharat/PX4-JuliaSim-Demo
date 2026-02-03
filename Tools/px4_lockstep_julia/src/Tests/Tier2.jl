function tier2_ballistic_freefall()
    t_end_s = 1.0
    dt_phys_s = 0.01
    dt_log_s = 0.01

    rb0 = RigidBody.RigidBodyState(
        pos_ned = Types.vec3(0.0, 0.0, -10.0),
        vel_ned = Types.vec3(0.0, 0.0, 0.0),
        q_bn = Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Types.vec3(0.0, 0.0, 0.0),
    )

    setup = _iris_fullplant(
        ;
        x0 = rb0,
        contact = Contacts.NoContact(),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )

    t_end_us = UInt64(round(Int, t_end_s * 1e6))
    dt_phys_us = UInt64(round(Int, dt_phys_s * 1e6))
    dt_log_us = UInt64(round(Int, dt_log_s * 1e6))

    timeline = Runtime.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = t_end_us + UInt64(1),
        dt_wind_us = t_end_us + UInt64(1),
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scn_times_us = UInt64[],
    )

    cfg = Runtime.EngineConfig(mode = Runtime.MODE_RECORD)
    bus = Runtime.SimBus(time_us = UInt64(0))
    rec = Recording.InMemoryRecorder()
    integrator = Integrators.RK4Integrator()

    eng = Runtime.Engine(
        cfg,
        timeline,
        bus,
        setup.plant0,
        setup.model,
        integrator,
        nothing,
        nothing;
        recorder = rec,
    )
    Runtime.run!(eng)

    g = Environment.gravity_accel(setup.env.gravity, rb0.pos_ned, 0.0)[3]
    z0 = rb0.pos_ned[3]
    vz0 = rb0.vel_ned[3]

    max_z_err = 0.0
    max_vz_err = 0.0
    max_qnorm_err = 0.0
    min_rotor = Inf
    soc_ok = true
    v1_ok = true
    n_logs = 0

    for (t_us, x) in zip(rec.times[:plant], rec.values[:plant])
        t = Float64(t_us) * 1e-6
        z_exp = z0 + vz0 * t + 0.5 * g * t * t
        vz_exp = vz0 + g * t

        max_z_err = max(max_z_err, abs(x.rb.pos_ned[3] - z_exp))
        max_vz_err = max(max_vz_err, abs(x.rb.vel_ned[3] - vz_exp))
        max_qnorm_err = max(max_qnorm_err, abs(_qnorm(x.rb.q_bn) - 1.0))
        min_rotor = min(min_rotor, minimum(x.rotor_ω))
        soc_ok &= 0.0 <= x.power.soc[1] <= 1.0
        v1_ok &= isfinite(x.power.v1[1])
        n_logs += 1
    end

    return (
        max_z_err = max_z_err,
        max_vz_err = max_vz_err,
        max_qnorm_err = max_qnorm_err,
        min_rotor = min_rotor,
        soc_ok = soc_ok,
        v1_ok = v1_ok,
        count = n_logs,
    )
end

function tier2_hover_force_balance()
    setup = _iris_fullplant(; x0 = RigidBody.RigidBodyState(), contact = Contacts.NoContact())

    alt_msl_m = setup.env.origin.alt_msl_m - setup.plant0.rb.pos_ned[3]
    ρ = Environment.air_density(setup.env.atmosphere, alt_msl_m)
    g = Environment.gravity_accel(setup.env.gravity, setup.plant0.rb.pos_ned, 0.0)[3]
    m = Vehicles.mass(setup.veh.model)

    prop = setup.veh.propulsion.units[1].prop
    function total_thrust(ω::Float64)
        Ti = Propulsion.prop_thrust(prop, ρ, ω, 0.0)
        return 4.0 * Ti
    end

    T_target = m * g
    ω_lo = 0.0
    ω_hi = 10.0
    for _ in 1:30
        if total_thrust(ω_hi) >= T_target
            break
        end
        ω_hi *= 2.0
    end

    for _ in 1:60
        ω_mid = 0.5 * (ω_lo + ω_hi)
        if total_thrust(ω_mid) >= T_target
            ω_hi = ω_mid
        else
            ω_lo = ω_mid
        end
    end
    ω_hover = ω_hi

    x = setup.plant0
    x = Plant.PlantState{4,1}(
        x.rb,
        x.motors_y,
        x.motors_ydot,
        x.servos_y,
        x.servos_ydot,
        SVector{4,Float64}(fill(ω_hover, 4)),
        x.power,
    )
    u = Plant.PlantInput()
    dx = setup.model(0.0, x, u)
    return (vel_dot = dx.rb.vel_dot,)
end

function tier2_bus_solve_residual_sweep()
    p = Propulsion.default_multirotor_set()
    rng = MersenneTwister(123)

    ocv = 16.0
    R0 = 0.05
    Vmin = 0.0

    max_resid = 0.0
    min_ok = true
    max_ok = true

    for _ in 1:25
        ω = SVector{4,Float64}(rand(rng, 4) .* 600.0)
        duty = SVector{4,Float64}(rand(rng, 4))
        v1 = 0.5 * rand(rng)

        V = PlantModels._solve_bus_voltage(p, ω, duty, ocv, v1, R0, Vmin)
        min_ok &= V >= Vmin - 1e-12
        max_ok &= V <= (ocv - v1) + 1e-12

        I = PlantModels._bus_current_total(p, ω, duty, V)
        V_rhs = clamp(Vmin, (ocv - v1) - R0 * I, (ocv - v1))
        max_resid = max(max_resid, abs(V - V_rhs))
    end

    ω0 = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0)
    duty1 = SVector{4,Float64}(0.2, 0.2, 0.2, 0.2)
    duty2 = SVector{4,Float64}(0.4, 0.4, 0.4, 0.4)
    v1 = 0.0

    V1 = PlantModels._solve_bus_voltage(p, ω0, duty1, ocv, v1, R0, Vmin)
    V2 = PlantModels._solve_bus_voltage(p, ω0, duty2, ocv, v1, R0, Vmin)
    monotonic_ok = V2 <= V1 + 1e-9

    return (max_resid = max_resid, min_ok = min_ok, max_ok = max_ok, monotonic_ok = monotonic_ok)
end

function tier2_quad_symmetry_torque()
    model = iris_vehicle_for_tests().model
    env = Environment.EnvironmentModel(wind = Environment.NoWind())
    x_rb = RigidBody.RigidBodyState()
    thrust = 5.0
    Q = 0.05

    u_bal = Propulsion.RotorOutput{4}(
        thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
        shaft_torque_nm = SVector{4,Float64}(Q, Q, -Q, -Q),
        ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        bus_current_a = 0.0,
    )
    dx_bal = Vehicles.dynamics(model, env, 0.0, x_rb, u_bal, Types.vec3(0.0, 0.0, 0.0))

    u_yaw_pos = Propulsion.RotorOutput{4}(
        thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
        shaft_torque_nm = SVector{4,Float64}(Q, Q, Q, Q),
        ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        bus_current_a = 0.0,
    )
    dx_yaw_pos = Vehicles.dynamics(model, env, 0.0, x_rb, u_yaw_pos, Types.vec3(0.0, 0.0, 0.0))

    u_yaw_neg = Propulsion.RotorOutput{4}(
        thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
        shaft_torque_nm = SVector{4,Float64}(-Q, -Q, -Q, -Q),
        ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
        bus_current_a = 0.0,
    )
    dx_yaw_neg = Vehicles.dynamics(model, env, 0.0, x_rb, u_yaw_neg, Types.vec3(0.0, 0.0, 0.0))

    return (bal = dx_bal.ω_dot, yaw_pos = dx_yaw_pos.ω_dot, yaw_neg = dx_yaw_neg.ω_dot)
end

function tier2_rotor_dir_sign_sensitivity()
    setup = _iris_fullplant()
    veh = setup.veh
    env2 = setup.env
    full_model = setup.model
    plant0 = setup.plant0
    ω0 = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
    motors_y = SVector{12,Float64}(0.6, 0.6, 0.6, 0.6, 0, 0, 0, 0, 0, 0, 0, 0)
    xplant = Plant.PlantState{4,1}(
        rb = plant0.rb,
        motors_y = motors_y,
        motors_ydot = plant0.motors_ydot,
        servos_y = plant0.servos_y,
        servos_ydot = plant0.servos_ydot,
        rotor_ω = ω0,
        power = Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )
    cmd = Vehicles.ActuatorCommand(motors = motors_y, servos = plant0.servos_y)
    u = Plant.PlantInput(cmd = cmd, wind_ned = Types.vec3(0.0, 0.0, 0.0), faults = Faults.FaultState())
    y1 = PlantModels.plant_outputs(full_model, 0.0, xplant, u)
    prop2 = Propulsion.QuadRotorSet(veh.propulsion.units, -veh.propulsion.rotor_dir)
    model2 = PlantModels.CoupledMultirotorModel(
        veh.model,
        env2,
        Contacts.NoContact(),
        veh.motor_actuators,
        veh.servo_actuators,
        prop2,
        setup.batt,
    )
    y2 = PlantModels.plant_outputs(model2, 0.0, xplant, u)
    return (ok = y2.rotors.shaft_torque_nm ≈ -y1.rotors.shaft_torque_nm,)
end

function tier2_fault_semantics()
    setup = _iris_fullplant()
    model_f = setup.model
    plant0_f = setup.plant0
    t = 0.0

    ω0 = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
    motors_y = SVector{12,Float64}(0.6, 0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    x = Plant.PlantState{4,1}(
        rb = plant0_f.rb,
        motors_y = motors_y,
        motors_ydot = plant0_f.motors_ydot,
        servos_y = plant0_f.servos_y,
        servos_ydot = plant0_f.servos_ydot,
        rotor_ω = ω0,
        power = Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    cmd = Vehicles.ActuatorCommand(motors = motors_y, servos = plant0_f.servos_y)

    u_nom = Plant.PlantInput(
        cmd = cmd,
        wind_ned = Types.vec3(0.0, 0.0, 0.0),
        faults = Faults.FaultState(),
    )
    y_nom = PlantModels.plant_outputs(model_f, t, x, u_nom)
    dx_nom = model_f(t, x, u_nom)

    u_dis = Plant.PlantInput(
        cmd = cmd,
        wind_ned = Types.vec3(0.0, 0.0, 0.0),
        faults = Faults.FaultState(motor_disable_mask = UInt32(0x1)),
    )
    y_dis = PlantModels.plant_outputs(model_f, t, x, u_dis)
    dx_dis = model_f(t, x, u_dis)

    u_disc = Plant.PlantInput(
        cmd = cmd,
        wind_ned = Types.vec3(0.0, 0.0, 0.0),
        faults = Faults.FaultState(battery_connected = false),
    )
    y_disc = PlantModels.plant_outputs(model_f, t, x, u_disc)

    bus_est = Runtime.SimBus(time_us = UInt64(0))
    plant_rb = RigidBody.RigidBodyState(pos_ned = Types.vec3(1.0, 2.0, 3.0))
    bus_est.est = Estimators.EstimatedState(
        pos_ned = Types.vec3(9.0, 9.0, 9.0),
        vel_ned = Types.vec3(0.0, 0.0, 0.0),
        q_bn = Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Types.vec3(0.0, 0.0, 0.0),
    )
    est = Sources.LiveEstimatorSource(Estimators.TruthEstimator(), MersenneTwister(123), 0.01)

    bus_est.faults = Faults.FaultState(sensor_fault_mask = Faults.SENSOR_FAULT_EST_FREEZE)
    Runtime.update!(est, bus_est, plant_rb, UInt64(0))
    est_freeze_ok = bus_est.est.pos_ned == Types.vec3(9.0, 9.0, 9.0)

    bus_est.faults = Faults.FaultState()
    Runtime.update!(est, bus_est, plant_rb, UInt64(0))
    est_update_ok = bus_est.est.pos_ned == plant_rb.pos_ned

    return (
        motor_current_disabled = y_dis.rotors.motor_current_a[1],
        motor_current_nom = y_nom.rotors.motor_current_a[1],
        bus_current_nom = y_nom.rotors.bus_current_a,
        bus_current_disabled = y_dis.rotors.bus_current_a,
        rotor_ω_dot_disabled = dx_dis.rotor_ω_dot[1],
        rotor_ω_dot_nom = dx_nom.rotor_ω_dot[1],
        battery_connected = y_disc.battery_statuses[1].connected,
        bus_voltage = y_disc.bus_voltage_v[1],
        bus_current_disc = y_disc.rotors.bus_current_a,
        motor_current_all_zero = all(y_disc.rotors.motor_current_a .== 0.0),
        est_freeze_ok = est_freeze_ok,
        est_update_ok = est_update_ok,
    )
end

function tier2_boundary_order_probe()
    t_end_us = UInt64(30_000)
    tl = Runtime.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    order_state = ProbeOrderState(Float64[])
    scenario = ProbeScenarioSource(order_state)
    wind_src = ProbeWindSource(order_state)
    estimator = ProbeEstimatorSource(order_state)
    autopilot = ProbeAutopilotSource(order_state)

    cfg = Runtime.EngineConfig(mode = Runtime.MODE_LIVE, enable_derived_outputs = false)
    sim = Runtime.Engine(
        cfg,
        tl,
        Runtime.SimBus(time_us = UInt64(0)),
        RigidBody.RigidBodyState(),
        ZeroRB(),
        Integrators.EulerIntegrator(),
        autopilot,
        wind_src;
        scenario = scenario,
        estimator = estimator,
        telemetry = Runtime.NullTelemetry(),
        recorder = Recording.NullRecorder(),
    )

    Runtime.run!(sim)
    seen = order_state.seen
    chunk_ok = length(seen) % 3 == 0
    if chunk_ok
        for i in 1:3:length(seen)
            chunk_ok &= (seen[i] == 1.0) && (seen[i + 1] == 2.0) && (seen[i + 2] == 3.0)
        end
    end
    order_ok = chunk_ok && (sim.bus.wind_ned[1] == 4.0)

    return (order_ok = order_ok, seen = seen, final_wind = sim.bus.wind_ned[1])
end

"""Compute Tier 2 full-plant contract results (used for precompile and tests)."""
function tier2_fullplant_results()
    return (
        ballistic = tier2_ballistic_freefall(),
        hover = tier2_hover_force_balance(),
        bus = tier2_bus_solve_residual_sweep(),
        quad_sym = tier2_quad_symmetry_torque(),
        rotor_dir = tier2_rotor_dir_sign_sensitivity(),
        faults = tier2_fault_semantics(),
        boundary_order = tier2_boundary_order_probe(),
    )
end
