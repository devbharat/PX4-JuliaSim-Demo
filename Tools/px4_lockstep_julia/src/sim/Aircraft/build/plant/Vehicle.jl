"""Vehicle parameter assembly helpers."""

function _build_vehicle(
    spec::AircraftSpec;
    x0_override::Union{Nothing,RigidBodyState} = nothing,
)
    a = spec.airframe

    # Motor/servo actuator models are sized to match the PX4 ABI arrays.
    motor_act = _build_actuator_model(spec.actuation.motor_actuators, 12)
    servo_act = _build_actuator_model(spec.actuation.servo_actuators, 8)

    # Rigid-body model params.
    N = length(spec.actuation.motors)
    rotor_pos = SVector{N,Vec3}(ntuple(i -> a.rotor_pos_body_m[i], N))

    # Propulsor axes: required spec field, normalized here.
    axis_src = a.rotor_axis_body_m
    rotor_axis = SVector{N,Vec3}(
        ntuple(i -> begin
            v = axis_src[i]
            invn = inv(sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3]))
            v .* invn
        end, N),
    )
    # Propulsion (generic multirotor default motor+prop set).
    gravity_model = _build_gravity(spec.environment)
    g_ned = Environment.gravity_accel(gravity_model, Types.vec3(0.0, 0.0, 0.0), 0.0)
    hover_T = a.mass_kg * g_ned[3] / Float64(N)
    p = a.propulsion
    prop = Propulsion.default_multirotor_set(
        N = N,
        km_m = p.km_m,
        V_nom = p.V_nom,
        ρ_nom = p.rho_nom,
        thrust_hover_per_rotor_n = hover_T,
        rotor_radius_m = p.rotor_radius_m,
        inflow_kT = p.inflow_kT,
        inflow_kQ = p.inflow_kQ,
        esc_eta = p.esc.eta,
        esc_deadzone = p.esc.deadzone,
        motor_kv_rpm_per_volt = p.motor.kv_rpm_per_volt,
        motor_r_ohm = p.motor.r_ohm,
        motor_j_kgm2 = p.motor.j_kgm2,
        motor_i0_a = p.motor.i0_a,
        motor_viscous_friction_nm_per_rad_s = p.motor.viscous_friction_nm_per_rad_s,
        motor_max_current_a = p.motor.max_current_a,
        thrust_calibration_mult = p.thrust_calibration_mult,
    )
    if p.rotor_dir !== nothing
        prop.rotor_dir = SVector{N,Float64}(ntuple(i -> Float64(p.rotor_dir[i]), N))
    end

    # Rigid-body inertia tensor (kg*m^2).
    Ixx, Iyy, Izz = a.inertia_diag_kgm2
    Ixy, Ixz, Iyz = a.inertia_products_kgm2
    I_body = @SMatrix [
        Ixx Ixy Ixz
        Ixy Iyy Iyz
        Ixz Iyz Izz
    ]

    # Precompute inverse for the hot path.
    I_body_inv = inv(I_body)

    # Rotor inertias (kg*m^2) are owned by the propulsion units.
    rotor_J =
        SVector{N,Float64}(ntuple(i -> Propulsion.rotor_inertia_kgm2(prop.units[i]), N))

    params = Vehicles.QuadrotorParams{N}(
        mass = a.mass_kg,
        inertia_kgm2 = I_body,
        inertia_inv_kgm2 = I_body_inv,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        rotor_inertia_kgm2 = rotor_J,
        rotor_dir = prop.rotor_dir,
        linear_drag = a.linear_drag,
        angular_damping = a.angular_damping,
    )

    model = Vehicles.GenericMultirotor{N}(params)

    x0 = x0_override === nothing ? a.x0 : x0_override
    return Vehicles.VehicleInstance(model, motor_act, servo_act, prop, x0)
end
