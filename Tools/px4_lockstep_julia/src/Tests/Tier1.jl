"""Compute Tier 1 subsystem results (used for precompile and tests)."""
function tier1_env_isa1976_spot_checks()
    atm = Environment.ISA1976()
    return (
        t0 = Environment.air_temperature(atm, 0.0),
        p0 = Environment.air_pressure(atm, 0.0),
        rho0 = Environment.air_density(atm, 0.0),
        t11 = Environment.air_temperature(atm, 11000.0),
        p11 = Environment.air_pressure(atm, 11000.0),
        rho11 = Environment.air_density(atm, 11000.0),
        rho0_gt_rho1k = Environment.air_density(atm, 0.0) > Environment.air_density(atm, 1000.0),
        p0_gt_p1k = Environment.air_pressure(atm, 0.0) > Environment.air_pressure(atm, 1000.0),
    )
end

function tier1_ouwind_discrete_update()
    rng_seed = 0x12345678
    rng_ref = MersenneTwister(rng_seed)
    rng = MersenneTwister(rng_seed)
    dt = 0.1
    v0 = Types.vec3(1.0, -2.0, 0.5)
    w = Environment.OUWind(σ = Types.vec3(2.0, 2.0, 2.0), τ_s = 5.0, v_gust = v0)
    ξ = Types.vec3(randn(rng_ref), randn(rng_ref), randn(rng_ref))
    ϕ = exp(-dt / w.τ_s)
    scale = sqrt(1.0 - ϕ * ϕ)
    expected = ϕ * v0 + (w.σ * scale) .* ξ
    Environment.step_wind!(w, Types.vec3(0.0, 0.0, 0.0), 0.0, dt, rng)
    return (phi = w.phi, scale = w.scale, v_gust = w.v_gust, expected = expected)
end

function tier1_battery_step_load()
    batt = build_thevenin(
        capacity_ah = 2.0,
        soc0 = 1.0,
        ocv_soc = [0.0, 1.0],
        ocv_v = [12.0, 12.0],
        r0 = 0.05,
        r1 = 0.10,
        c1 = 10.0,
        v1_0 = 0.0,
        min_voltage_v = 0.0,
    )

    I = 5.0
    dt_batt = 0.01
    t_end = 1.0
    n = Int(floor(t_end / dt_batt))

    Q_c = batt.capacity_c
    τ = batt.r1 * batt.c1
    st = Powertrain.battery_state(batt)
    soc0 = st.soc
    v1_0 = st.v1

    ϕ_b = exp(-dt_batt / τ)
    soc_expected = soc0
    v1_expected = v1_0
    max_soc_err = 0.0
    max_v1_err = 0.0
    max_vt_err = 0.0

    for _k in 1:n
        Powertrain.step!(batt, st, I, dt_batt)
        soc_expected -= I * dt_batt / Q_c
        v1_expected = v1_expected * ϕ_b + I * batt.r1 * (1.0 - ϕ_b)

        soc_err = abs(st.soc - soc_expected)
        max_soc_err = max(max_soc_err, soc_err)
        v1_err = abs(st.v1 - v1_expected)
        max_v1_err = max(max_v1_err, v1_err)

        r0_eff = Powertrain.r0_ohm(batt.r0_model, st.soc, batt.temp_c)
        V_expected = 12.0 - I * r0_eff - v1_expected
        vt_err = abs(Powertrain.status(batt, st).voltage_v - V_expected)
        max_vt_err = max(max_vt_err, vt_err)
    end

    return (
        max_soc_err = max_soc_err,
        max_v1_err = max_v1_err,
        max_vt_err = max_vt_err,
        soc0 = soc0,
        soc = st.soc,
        v1 = st.v1,
    )
end

function tier1_multi_battery_plant_state_math()
    power0 = Plant.PowerState{2}(
        soc = SVector{2,Float64}(1.0, 0.5),
        v1 = SVector{2,Float64}(0.1, 0.2),
    )
    x0 = Plant.PlantState{4,2}(power = power0)

    power_dot = Plant.PowerDeriv{2}(
        soc_dot = SVector{2,Float64}(-0.01, -0.02),
        v1_dot = SVector{2,Float64}(0.03, 0.04),
    )
    k = Plant.PlantDeriv{4,2}(power = power_dot)

    dt_lin = 2.0
    x1 = Plant.plant_add(x0, k, dt_lin)
    x2 = Plant.plant_scale_add(x0, k, k, k, k, dt_lin)
    k2 = Plant.PlantDeriv{4,2}(
        power = Plant.PowerDeriv{2}(
            soc_dot = SVector{2,Float64}(-0.03, -0.01),
            v1_dot = SVector{2,Float64}(0.02, 0.01),
        ),
    )
    x3 = Plant.plant_lincomb(x0, dt_lin, (k, k2), (0.25, 0.75))
    exp1_soc = power0.soc + power_dot.soc_dot * dt_lin
    exp1_v1 = power0.v1 + power_dot.v1_dot * dt_lin
    exp_soc = power0.soc + (0.25 * power_dot.soc_dot + 0.75 * k2.power.soc_dot) * dt_lin
    exp_v1 = power0.v1 + (0.25 * power_dot.v1_dot + 0.75 * k2.power.v1_dot) * dt_lin

    return (
        x1_soc = x1.power.soc,
        x1_v1 = x1.power.v1,
        x2_soc = x2.power.soc,
        x2_v1 = x2.power.v1,
        x3_soc = x3.power.soc,
        x3_v1 = x3.power.v1,
        exp1_soc = exp1_soc,
        exp1_v1 = exp1_v1,
        exp_soc = exp_soc,
        exp_v1 = exp_v1,
    )
end

function tier1_init_plant_state_multi_battery()
    veh = iris_vehicle_for_tests()
    b1 = build_thevenin(soc0 = 0.9, v1_0 = 0.12)
    b2 = build_thevenin(soc0 = 0.8, v1_0 = 0.34)
    x_init = Plant.init_plant_state(
        veh.state,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        (b1, b2),
    )
    return (soc = x_init.power.soc, v1 = x_init.power.v1)
end

function tier1_when_soc_below_min()
    s = Scenario.EventScenario()
    Scenario.when_soc_below!(s, 0.1, (st, ctx, t) -> st)
    e = s.scheduler.events[1]
    plant_low = Plant.PlantState{4,2}(
        power = Plant.PowerState{2}(
            soc = SVector{2,Float64}(0.2, 0.05),
            v1 = SVector{2,Float64}(0.0, 0.0),
        ),
    )
    ctx_low = Scenario.ScenarioContext(
        t_us = UInt64(0),
        t_s = 0.0,
        step = 0,
        plant = plant_low,
        rb = plant_low.rb,
    )
    plant_high = Plant.PlantState{4,2}(
        power = Plant.PowerState{2}(
            soc = SVector{2,Float64}(0.2, 0.15),
            v1 = SVector{2,Float64}(0.0, 0.0),
        ),
    )
    ctx_high = Scenario.ScenarioContext(
        t_us = UInt64(0),
        t_s = 0.0,
        step = 0,
        plant = plant_high,
        rb = plant_high.rb,
    )
    return (
        event_count = length(s.scheduler.events),
        event_is_when = e isa Events.When,
        cond_low = e.condition(s.state, ctx_low, 0.0),
        cond_high = e.condition(s.state, ctx_high, 0.0),
    )
end

function tier1_power_network_share()
    b1 = build_thevenin(
        capacity_ah = 2.0,
        soc0 = 1.0,
        ocv_soc = [0.0, 1.0],
        ocv_v = [12.0, 12.0],
        r0 = 0.05,
        r1 = 0.01,
        c1 = 100.0,
        v1_0 = 0.0,
        min_voltage_v = 0.0,
    )
    b2 = build_thevenin(
        capacity_ah = 2.0,
        soc0 = 1.0,
        ocv_soc = [0.0, 1.0],
        ocv_v = [12.0, 12.0],
        r0 = 0.10,
        r1 = 0.01,
        c1 = 100.0,
        v1_0 = 0.0,
        min_voltage_v = 0.0,
    )
    net = PlantModels.PowerNetwork{4,2,1}(
        bus_for_motor = SVector{4,Int}(1, 1, 1, 1),
        bus_for_battery = SVector{2,Int}(1, 1),
        avionics_load_w = SVector{1,Float64}(12.0),
        share_mode = :inv_r0,
    )
    model = PlantModels.CoupledMultirotorModel(
        iris_vehicle_for_tests().model,
        Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0)),
        Contacts.NoContact(),
        Vehicles.DirectActuators(),
        Vehicles.DirectActuators(),
        Propulsion.default_multirotor_set(),
        (b1, b2),
        net,
    )
    x0_share = Plant.PlantState{4,2}(
        rotor_ω = zero(SVector{4,Float64}),
        power = Plant.PowerState{2}(
            soc = SVector{2,Float64}(1.0, 1.0),
            v1 = SVector{2,Float64}(0.0, 0.0),
        ),
    )
    u = Plant.PlantInput(cmd = Vehicles.ActuatorCommand(), faults = Faults.FaultState())
    y = PlantModels.plant_outputs(model, 0.0, x0_share, u)
    dx = model(0.0, x0_share, u)
    I1 = -dx.power.soc_dot[1] * b1.capacity_c
    I2 = -dx.power.soc_dot[2] * b2.capacity_c
    net_eq = PlantModels.PowerNetwork{4,2,1}(
        bus_for_motor = SVector{4,Int}(1, 1, 1, 1),
        bus_for_battery = SVector{2,Int}(1, 1),
        avionics_load_w = SVector{1,Float64}(12.0),
        share_mode = :equal,
    )
    model_eq = PlantModels.CoupledMultirotorModel(
        iris_vehicle_for_tests().model,
        Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0)),
        Contacts.NoContact(),
        Vehicles.DirectActuators(),
        Vehicles.DirectActuators(),
        Propulsion.default_multirotor_set(),
        (b1, b2),
        net_eq,
    )
    dx_eq = model_eq(0.0, x0_share, u)
    I1_eq = -dx_eq.power.soc_dot[1] * b1.capacity_c
    I2_eq = -dx_eq.power.soc_dot[2] * b2.capacity_c
    return (
        I1 = I1,
        I2 = I2,
        total = y.bus_current_a[1],
        ratio = I1 / I2,
        I1_eq = I1_eq,
        I2_eq = I2_eq,
    )
end

function tier1_multi_bus_voltage()
    b1 = build_thevenin(
        capacity_ah = 2.0,
        soc0 = 1.0,
        ocv_soc = [0.0, 1.0],
        ocv_v = [12.0, 12.0],
        r0 = 0.10,
        r1 = 0.01,
        c1 = 100.0,
        v1_0 = 0.0,
        min_voltage_v = 0.0,
    )
    b2 = build_thevenin(
        capacity_ah = 2.0,
        soc0 = 1.0,
        ocv_soc = [0.0, 1.0],
        ocv_v = [12.0, 12.0],
        r0 = 0.10,
        r1 = 0.01,
        c1 = 100.0,
        v1_0 = 0.0,
        min_voltage_v = 0.0,
    )
    net = PlantModels.PowerNetwork{4,2,2}(
        bus_for_motor = SVector{4,Int}(1, 1, 2, 2),
        bus_for_battery = SVector{2,Int}(1, 2),
        avionics_load_w = SVector{2,Float64}(12.0, 0.0),
        share_mode = :inv_r0,
    )
    model = PlantModels.CoupledMultirotorModel(
        iris_vehicle_for_tests().model,
        Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0)),
        Contacts.NoContact(),
        Vehicles.DirectActuators(),
        Vehicles.DirectActuators(),
        Propulsion.default_multirotor_set(),
        (b1, b2),
        net,
    )
    x0_bus = Plant.PlantState{4,2}(
        rotor_ω = zero(SVector{4,Float64}),
        power = Plant.PowerState{2}(
            soc = SVector{2,Float64}(1.0, 1.0),
            v1 = SVector{2,Float64}(0.0, 0.0),
        ),
    )
    cmd = Vehicles.ActuatorCommand()
    u = Plant.PlantInput(cmd = cmd, faults = Faults.FaultState())
    y = PlantModels.plant_outputs(model, 0.0, x0_bus, u)
    bats = y.battery_statuses
    return (b1_v = bats[1].voltage_v, b2_v = bats[2].voltage_v)
end

function tier1_back_emf_zero_bus_load()
    setup = _iris_fullplant()
    model = setup.model
    batt = model.batteries[1]
    soc = setup.plant0.power.soc[1]
    ocv = PlantModels._battery_ocv(batt, soc)
    Ke = Propulsion.motor_Ke(model.propulsion.units[1].motor)
    duty = 0.2
    ω_hi = 1.2 * (duty * ocv / Ke)
    x = Plant.PlantState{4,1}(
        rb = setup.plant0.rb,
        motors_y = setup.plant0.motors_y,
        motors_ydot = setup.plant0.motors_ydot,
        servos_y = setup.plant0.servos_y,
        servos_ydot = setup.plant0.servos_ydot,
        rotor_ω = SVector{4,Float64}(ω_hi, ω_hi, ω_hi, ω_hi),
        power = setup.plant0.power,
    )
    motors = SVector{12,Float64}(duty, duty, duty, duty, 0, 0, 0, 0, 0, 0, 0, 0)
    u = Plant.PlantInput(cmd = Vehicles.ActuatorCommand(motors = motors))
    y = PlantModels.plant_outputs(model, 0.0, x, u)
    return (bus_current = y.bus_current_a[1], bus_voltage = y.bus_voltage_v[1], ocv = ocv)
end

function tier1_propulsor_axis_geometry()
    env0 = Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0))
    rotor_pos = SVector{1,Types.Vec3}(Types.vec3(0.0, 0.0, 0.0))
    rotor_axis = SVector{1,Types.Vec3}(Types.vec3(1.0, 0.0, 0.0))
    I = Types.Mat3([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    params = Vehicles.QuadrotorParams{1}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = I,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{1,Float64}(0.0),
        rotor_dir = SVector{1,Float64}(1.0),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    model_rb = Vehicles.GenericMultirotor{1}(params)
    x_rb = RigidBody.RigidBodyState()
    out = Propulsion.RotorOutput{1}(
        thrust_n = SVector{1,Float64}(2.0),
        shaft_torque_nm = SVector{1,Float64}(0.5),
        ω_rad_s = SVector{1,Float64}(0.0),
        motor_current_a = SVector{1,Float64}(0.0),
        bus_current_a = 0.0,
    )
    d = Vehicles.dynamics(model_rb, env0, 0.0, x_rb, out, Types.vec3(0.0, 0.0, 0.0))
    return (vel_dot = d.vel_dot, ω_dot = d.ω_dot)
end

function tier1_wrench_composition()
    env0 = Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0))
    I = Types.Mat3([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    rotor_pos = SVector{2,Types.Vec3}(Types.vec3(1.0, 0.0, 0.0), Types.vec3(0.0, 1.0, 0.0))
    rotor_axis = SVector{2,Types.Vec3}(Types.vec3(0.0, 0.0, 1.0), Types.vec3(0.0, 1.0, 0.0))
    params = Vehicles.QuadrotorParams{2}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = I,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{2,Float64}(0.0, 0.0),
        rotor_dir = SVector{2,Float64}(1.0, 1.0),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    model_rb = Vehicles.GenericMultirotor{2}(params)
    x_rb = RigidBody.RigidBodyState()
    out = Propulsion.RotorOutput{2}(
        thrust_n = SVector{2,Float64}(2.0, 3.0),
        shaft_torque_nm = SVector{2,Float64}(0.5, 0.2),
        ω_rad_s = SVector{2,Float64}(0.0, 0.0),
        motor_current_a = SVector{2,Float64}(0.0, 0.0),
        bus_current_a = 0.0,
    )
    d = Vehicles.dynamics(model_rb, env0, 0.0, x_rb, out, Types.vec3(0.0, 0.0, 0.0))
    return (vel_dot = d.vel_dot, ω_dot = d.ω_dot)
end

function tier1_vax_sign_projection()
    env0 = Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0))
    I = Types.Mat3([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    rotor_pos = SVector{1,Types.Vec3}(Types.vec3(0.0, 0.0, 0.0))
    rotor_axis = SVector{1,Types.Vec3}(Types.vec3(1.0, 0.0, 0.0))
    params = Vehicles.QuadrotorParams{1}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = I,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{1,Float64}(0.0),
        rotor_dir = SVector{1,Float64}(1.0),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    model = Vehicles.GenericMultirotor{1}(params)
    motor_act = Vehicles.DirectActuators()
    servo_act = Vehicles.DirectActuators()
    esc = Propulsion.ESCParams()
    motor = Propulsion.BLDCMotorParams()
    units = [Propulsion.MotorPropUnit(esc = esc, motor = motor, prop = SignProp(1.0, 0.5))]
    prop = Propulsion.QuadRotorSet(units, SVector{1,Float64}(1.0))
    battery_vax = build_ideal(voltage_v = 12.0)
    motor_map = Vehicles.MotorMap{1}(SVector{1,Int}(1))
    dynfun = PlantModels.CoupledMultirotorModel(
        model,
        env0,
        Contacts.NoContact(),
        motor_act,
        servo_act,
        prop,
        battery_vax;
        motor_map = motor_map,
    )
    plant0 = Plant.init_plant_state(
        RigidBody.RigidBodyState(),
        motor_act,
        servo_act,
        prop,
        battery_vax,
    )
    plant = Plant.PlantState{1,1}(
        rb = plant0.rb,
        motors_y = plant0.motors_y,
        motors_ydot = plant0.motors_ydot,
        servos_y = plant0.servos_y,
        servos_ydot = plant0.servos_ydot,
        rotor_ω = SVector{1,Float64}(0.0),
        power = plant0.power,
    )
    cmd = Vehicles.ActuatorCommand(motors = plant0.motors_y, servos = plant0.servos_y)
    u_pos = Plant.PlantInput(
        cmd = cmd,
        wind_ned = Types.vec3(1.0, 0.0, 0.0),
        faults = Faults.FaultState(),
    )
    u_neg = Plant.PlantInput(
        cmd = cmd,
        wind_ned = Types.vec3(-1.0, 0.0, 0.0),
        faults = Faults.FaultState(),
    )
    y_pos = PlantModels.plant_outputs(dynfun, 0.0, plant, u_pos)
    y_neg = PlantModels.plant_outputs(dynfun, 0.0, plant, u_neg)
    return (thrust_pos = y_pos.rotors.thrust_n[1], thrust_neg = y_neg.rotors.thrust_n[1])
end

function tier1_twin_forward_props_yaw()
    env0 = Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0))
    I = Types.Mat3([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    r = 0.5
    rotor_pos = SVector{2,Types.Vec3}(Types.vec3(0.0, r, 0.0), Types.vec3(0.0, -r, 0.0))
    rotor_axis = SVector{2,Types.Vec3}(Types.vec3(-1.0, 0.0, 0.0), Types.vec3(-1.0, 0.0, 0.0))
    params = Vehicles.QuadrotorParams{2}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = I,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{2,Float64}(0.0, 0.0),
        rotor_dir = SVector{2,Float64}(1.0, 1.0),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    model_rb = Vehicles.GenericMultirotor{2}(params)
    x_rb = RigidBody.RigidBodyState()
    T_left = 2.0
    T_right = 5.0
    out = Propulsion.RotorOutput{2}(
        thrust_n = SVector{2,Float64}(T_left, T_right),
        shaft_torque_nm = SVector{2,Float64}(0.0, 0.0),
        ω_rad_s = SVector{2,Float64}(0.0, 0.0),
        motor_current_a = SVector{2,Float64}(0.0, 0.0),
        bus_current_a = 0.0,
    )
    d = Vehicles.dynamics(model_rb, env0, 0.0, x_rb, out, Types.vec3(0.0, 0.0, 0.0))
    return (vel_dot = d.vel_dot, ω_dot = d.ω_dot)
end

function tier1_twin_forward_props_roll()
    env0 = Environment.EnvironmentModel(gravity = Environment.UniformGravity(0.0))
    I = Types.Mat3([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    r = 0.5
    rotor_pos = SVector{2,Types.Vec3}(Types.vec3(0.0, r, 0.0), Types.vec3(0.0, -r, 0.0))
    rotor_axis = SVector{2,Types.Vec3}(Types.vec3(-1.0, 0.0, 0.0), Types.vec3(-1.0, 0.0, 0.0))
    params = Vehicles.QuadrotorParams{2}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = I,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = SVector{2,Float64}(0.0, 0.0),
        rotor_dir = SVector{2,Float64}(1.0, 1.0),
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    model_rb = Vehicles.GenericMultirotor{2}(params)
    x_rb = RigidBody.RigidBodyState()
    T_left = 3.0
    T_right = 3.0
    Q_left = 0.4
    Q_right = 0.6
    out = Propulsion.RotorOutput{2}(
        thrust_n = SVector{2,Float64}(T_left, T_right),
        shaft_torque_nm = SVector{2,Float64}(Q_left, Q_right),
        ω_rad_s = SVector{2,Float64}(0.0, 0.0),
        motor_current_a = SVector{2,Float64}(0.0, 0.0),
        bus_current_a = 0.0,
    )
    d = Vehicles.dynamics(model_rb, env0, 0.0, x_rb, out, Types.vec3(0.0, 0.0, 0.0))
    return (ω_dot = d.ω_dot,)
end

function tier1_ca_axis_param_sign()
    rotor_pos = Types.Vec3[Types.vec3(0.0, 0.0, 0.0), Types.vec3(0.0, 0.0, 0.0)]
    rotor_axis = Types.Vec3[Types.vec3(0.0, 0.0, 2.0), Types.vec3(0.0, 1.0, 0.0)]
    airframe = Aircraft.AirframeSpec(
        kind = :multirotor,
        mass_kg = 1.0,
        inertia_diag_kgm2 = Types.vec3(1.0, 1.0, 1.0),
        rotor_pos_body_m = rotor_pos,
        rotor_axis_body_m = rotor_axis,
        linear_drag = 0.0,
        angular_damping = Types.vec3(0.0, 0.0, 0.0),
    )
    motors = Aircraft.MotorChannelSpec[
        Aircraft.MotorChannelSpec(id = :motor1, channel = 1),
        Aircraft.MotorChannelSpec(id = :motor2, channel = 2),
    ]
    actuation = Aircraft.ActuationSpec(
        motors = motors,
        servos = Aircraft.ServoSpec[],
        motor_actuators = Aircraft.DirectActuatorSpec(),
        servo_actuators = Aircraft.DirectActuatorSpec(),
    )
    spec = Aircraft.AircraftSpec(name = :test, airframe = airframe, actuation = actuation)
    vehicle = Aircraft._build_vehicle(spec)
    params = Aircraft._derive_ca_params(spec, vehicle)
    pmap = Dict(p.name => Float64(p.value) for p in params)
    return (rotor0_az = pmap["CA_ROTOR0_AZ"], rotor1_ay = pmap["CA_ROTOR1_AY"])
end

function tier1_subsystems()
    return (
        env = tier1_env_isa1976_spot_checks(),
        ouwind = tier1_ouwind_discrete_update(),
        battery = tier1_battery_step_load(),
        plant_math = tier1_multi_battery_plant_state_math(),
        init_state = tier1_init_plant_state_multi_battery(),
        soc_below = tier1_when_soc_below_min(),
        power_share = tier1_power_network_share(),
        multi_bus = tier1_multi_bus_voltage(),
        back_emf = tier1_back_emf_zero_bus_load(),
        prop_axis = tier1_propulsor_axis_geometry(),
        wrench = tier1_wrench_composition(),
        vax = tier1_vax_sign_projection(),
        twin_yaw = tier1_twin_forward_props_yaw(),
        twin_roll = tier1_twin_forward_props_roll(),
        ca_axis = tier1_ca_axis_param_sign(),
    )
end
