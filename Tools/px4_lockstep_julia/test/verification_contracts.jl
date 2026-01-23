"""System-level and subsystem contract verification tests.

This file is intended to grow into a *verification ladder* (Tier 1/2/3) that complements
pure integrator correctness tests in `verification_cases.jl`.

Design goals:
- Deterministic and cheap enough to run frequently (Tier 1/2).
- Catch frame/sign/unit/coupling mistakes early.
- Avoid requiring the PX4 C bridge for most tests.

Many of these tests are “contract tests”: they validate properties of the plant/environment
models and the runtime engine semantics without relying on closed-loop stability.
"""

const V = Sim.Verification
const T = Sim.Types
const Env = Sim.Environment
const PT = Sim.Powertrain
const RB = Sim.RigidBody

"""Test-only prop model that makes thrust/torque depend on Vax sign."""
struct SignProp <: Sim.Propulsion.AbstractPropParams
    kT::Float64
    kQ::Float64
end

Sim.Propulsion.prop_thrust(p::SignProp, _ρ::Float64, _ω::Float64, Vax::Float64) = p.kT * Vax
Sim.Propulsion.prop_torque(p::SignProp, _ρ::Float64, _ω::Float64, Vax::Float64) = p.kQ * Vax

"""Build a minimal Iris full-plant model (CoupledMultirotorModel) for tests.

Returns a NamedTuple with:
- veh   :: Vehicles.VehicleInstance
- batt  :: Powertrain.TheveninBattery
- env   :: Environment.EnvironmentModel
- model :: PlantModels.CoupledMultirotorModel
- plant0:: Plant.PlantState{4}
"""
function _iris_fullplant(
    ;
    x0::RB.RigidBodyState = RB.RigidBodyState(),
    contact = Sim.Contacts.NoContact(),
    linear_drag::Union{Nothing,Float64} = nothing,
    angular_damping::Union{Nothing,T.Vec3} = nothing,
)
    veh = Sim.iris_default_vehicle(; x0 = x0)
    if linear_drag !== nothing || angular_damping !== nothing
        params = veh.model.params
        params_new = Sim.Vehicles.QuadrotorParams{4}(
            mass = params.mass,
            inertia_diag = params.inertia_diag,
            rotor_pos_body = params.rotor_pos_body,
            rotor_axis_body = params.rotor_axis_body,
            linear_drag = linear_drag === nothing ? params.linear_drag : linear_drag,
            angular_damping =
                angular_damping === nothing ? params.angular_damping : angular_damping,
        )
        veh.model = Sim.Vehicles.IrisQuadrotor(params = params_new)
    end
    batt = Sim.iris_default_battery()
    env = Sim.iris_default_env_replay()

    model = Sim.PlantModels.CoupledMultirotorModel(
        veh.model,
        env,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )
    plant0 = Sim.Plant.init_plant_state(
        veh.state,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )

    return (veh=veh, batt=batt, env=env, model=model, plant0=plant0)
end

"""Compute the L2 norm of a quaternion stored as a 4-vector."""
@inline function _qnorm(q::T.Quat)::Float64
    return sqrt(sum(abs2, q))
end

@testset "Verification Tier 1 - Subsystems" begin
    @testset "Environment ISA1976 spot checks" begin
        # These are classic “sanity anchor” checks: units, constants, and lapse-rate branch.
        atm = Env.ISA1976()

        # Sea-level
        @test isapprox(Env.air_temperature(atm, 0.0), 288.15; atol=1e-9)
        @test isapprox(Env.air_pressure(atm, 0.0), 101325.0; atol=1e-9)
        @test isapprox(Env.air_density(atm, 0.0), 1.225; rtol=1e-3)

        # 11 km (top of troposphere in the ISA definition)
        @test isapprox(Env.air_temperature(atm, 11000.0), 216.65; atol=1e-3)
        @test isapprox(Env.air_pressure(atm, 11000.0), 22632.1; rtol=1e-2)
        @test isapprox(Env.air_density(atm, 11000.0), 0.36391; rtol=1e-2)

        # Basic monotonic sanity
        @test Env.air_density(atm, 0.0) > Env.air_density(atm, 1000.0)
        @test Env.air_pressure(atm, 0.0) > Env.air_pressure(atm, 1000.0)
    end

    @testset "OUWind discrete update contract" begin
        # Verify the OU recurrence matches the exact discrete-time update.
        # The same randn draws are computed with an identical RNG seed to remain stable
        # even if Julia's normal RNG implementation changes.
        rng_seed = 0x12345678
        rng_ref = MersenneTwister(rng_seed)
        rng = MersenneTwister(rng_seed)

        dt = 0.1
        v0 = T.vec3(1.0, -2.0, 0.5)
        w = Env.OUWind(σ = T.vec3(2.0, 2.0, 2.0), τ_s = 5.0, v_gust = v0)

        ξ = T.vec3(randn(rng_ref), randn(rng_ref), randn(rng_ref))
        ϕ = exp(-dt / w.τ_s)
        scale = sqrt(1.0 - ϕ * ϕ)
        expected = ϕ * v0 + (w.σ * scale) .* ξ

        Env.step_wind!(w, T.vec3(0.0, 0.0, 0.0), 0.0, dt, rng)

        @test isapprox(w.phi, ϕ; atol=1e-14)
        @test isapprox(w.scale, scale; atol=1e-14)
        @test isapprox(w.v_gust, expected; atol=1e-12)
    end

    @testset "Powertrain Thevenin battery analytic step-load" begin
        # Battery-only verification under constant discharge current.
        # Closed form for V1 and SOC:
        #   SOC(t) = SOC0 - I*t/Q,  Q=Ah*3600
        #   V1(t)  = V1(0)*exp(-t/τ) + I*R1*(1-exp(-t/τ)), τ=R1*C1
        batt = PT.TheveninBattery(
            capacity_ah=2.0,
            soc0=1.0,
            ocv_soc=[0.0, 1.0],
            ocv_v=[12.0, 12.0],
            r0=0.05,
            r1=0.10,
            c1=100.0,
            v1_0=0.0,
            min_voltage_v=0.0,
        )

        I = 5.0
        dt = 0.01
        t_end = 10.0
        n = Int(floor(t_end / dt))

        Q_c = batt.capacity_c
        τ = batt.r1 * batt.c1
        st = PT.battery_state(batt)
        soc0 = st.soc
        v1_0 = st.v1

        for k in 1:n
            PT.step!(batt, st, I, dt)
            t = k * dt

            soc_expected = soc0 - I * t / Q_c
            v1_expected = v1_0 * exp(-t / τ) + I * batt.r1 * (1.0 - exp(-t / τ))

            @test isapprox(st.soc, soc_expected; atol=1e-12)
            @test isapprox(st.v1, v1_expected; atol=1e-11)

            # Terminal voltage check (OCV is constant here).
            V_expected = 12.0 - I * batt.r0 - v1_expected
            @test isapprox(PT.status(batt, st).voltage_v, V_expected; atol=1e-10)
        end

        @test st.soc < soc0
        @test st.v1 > 0.0
    end

    @testset "Phase 4 - Propulsor axis geometry" begin
        # Force/torque directions should follow the propulsor axis convention:
        # F = -T * axis_b, τ = r × F + axis_b * Q
        env = Env.EnvironmentModel(gravity = Env.UniformGravity(0.0))
        rotor_pos = SVector{1,T.Vec3}(T.vec3(0.0, 0.0, 0.0))
        rotor_axis = SVector{1,T.Vec3}(T.vec3(1.0, 0.0, 0.0)) # axis points +X

        params = Sim.Vehicles.QuadrotorParams{1}(
            mass = 1.0,
            inertia_diag = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body = rotor_pos,
            rotor_axis_body = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )
        model = Sim.Vehicles.GenericMultirotor{1}(params)
        x = RB.RigidBodyState()

        out = Sim.Propulsion.RotorOutput{1}(
            thrust_n = SVector{1,Float64}(2.0),
            shaft_torque_nm = SVector{1,Float64}(0.5),
            ω_rad_s = SVector{1,Float64}(0.0),
            motor_current_a = SVector{1,Float64}(0.0),
            bus_current_a = 0.0,
        )

        d = Sim.Vehicles.dynamics(model, env, 0.0, x, out, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(d.vel_dot[1], -2.0; atol = 1e-12)
        @test isapprox(d.vel_dot[2], 0.0; atol = 1e-12)
        @test isapprox(d.vel_dot[3], 0.0; atol = 1e-12)
        @test isapprox(d.ω_dot[1], 0.5; atol = 1e-12)
        @test isapprox(d.ω_dot[2], 0.0; atol = 1e-12)
        @test isapprox(d.ω_dot[3], 0.0; atol = 1e-12)
    end

    @testset "Phase 4 - Wrench composition (r×F + axis*Q)" begin
        env = Env.EnvironmentModel(gravity = Env.UniformGravity(0.0))
        rotor_pos = SVector{2,T.Vec3}(T.vec3(1.0, 0.0, 0.0), T.vec3(0.0, 1.0, 0.0))
        rotor_axis = SVector{2,T.Vec3}(T.vec3(0.0, 0.0, 1.0), T.vec3(0.0, 1.0, 0.0))

        params = Sim.Vehicles.QuadrotorParams{2}(
            mass = 1.0,
            inertia_diag = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body = rotor_pos,
            rotor_axis_body = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )
        model = Sim.Vehicles.GenericMultirotor{2}(params)
        x = RB.RigidBodyState()

        # Rotor 1: axis +Z, thrust 2, reaction torque 0.5
        # Rotor 2: axis +Y, thrust 3, reaction torque 0.2
        out = Sim.Propulsion.RotorOutput{2}(
            thrust_n = SVector{2,Float64}(2.0, 3.0),
            shaft_torque_nm = SVector{2,Float64}(0.5, 0.2),
            ω_rad_s = SVector{2,Float64}(0.0, 0.0),
            motor_current_a = SVector{2,Float64}(0.0, 0.0),
            bus_current_a = 0.0,
        )

        # Expected force: F = -T * axis
        F_exp = T.vec3(0.0, -3.0, -2.0)
        # Expected torque: r×F + axis*Q
        τ_exp = T.vec3(0.0, 2.2, 0.5)

        d = Sim.Vehicles.dynamics(model, env, 0.0, x, out, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(d.vel_dot[1], F_exp[1]; atol = 1e-12)
        @test isapprox(d.vel_dot[2], F_exp[2]; atol = 1e-12)
        @test isapprox(d.vel_dot[3], F_exp[3]; atol = 1e-12)
        @test isapprox(d.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(d.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(d.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - Vax sign from axis projection" begin
        env = Env.EnvironmentModel(gravity = Env.UniformGravity(0.0))
        rotor_pos = SVector{1,T.Vec3}(T.vec3(0.0, 0.0, 0.0))
        rotor_axis = SVector{1,T.Vec3}(T.vec3(1.0, 0.0, 0.0)) # axis points +X

        params = Sim.Vehicles.QuadrotorParams{1}(
            mass = 1.0,
            inertia_diag = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body = rotor_pos,
            rotor_axis_body = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )
        model = Sim.Vehicles.GenericMultirotor{1}(params)
        motor_act = Sim.Vehicles.DirectActuators()
        servo_act = Sim.Vehicles.DirectActuators()

        esc = Sim.Propulsion.ESCParams()
        motor = Sim.Propulsion.BLDCMotorParams()
        units = [Sim.Propulsion.MotorPropUnit(esc = esc, motor = motor, prop = SignProp(1.0, 0.5))]
        prop = Sim.Propulsion.QuadRotorSet{1}(units, SVector{1,Float64}(1.0))
        battery = PT.IdealBattery()

        motor_map = Sim.Vehicles.MotorMap{1}(SVector{1,Int}(1))
        dynfun = Sim.PlantModels.CoupledMultirotorModel(
            model,
            env,
            Sim.Contacts.NoContact(),
            motor_act,
            servo_act,
            prop,
            battery;
            motor_map = motor_map,
        )

        plant0 = Sim.Plant.init_plant_state(
            Sim.RigidBody.RigidBodyState(),
            motor_act,
            servo_act,
            prop,
            battery,
        )

        plant = Sim.Plant.PlantState{1}(
            rb = plant0.rb,
            motors_y = plant0.motors_y,
            motors_ydot = plant0.motors_ydot,
            servos_y = plant0.servos_y,
            servos_ydot = plant0.servos_ydot,
            rotor_ω = SVector{1,Float64}(0.0),
            batt_soc = plant0.batt_soc,
            batt_v1 = plant0.batt_v1,
        )

        cmd = Sim.Vehicles.ActuatorCommand(motors = plant0.motors_y, servos = plant0.servos_y)

        u_pos = Sim.Plant.PlantInput(
            cmd = cmd,
            wind_ned = T.vec3(1.0, 0.0, 0.0),
            faults = Sim.Faults.FaultState(),
        )
        u_neg = Sim.Plant.PlantInput(
            cmd = cmd,
            wind_ned = T.vec3(-1.0, 0.0, 0.0),
            faults = Sim.Faults.FaultState(),
        )

        y_pos = Sim.plant_outputs(dynfun, 0.0, plant, u_pos)
        y_neg = Sim.plant_outputs(dynfun, 0.0, plant, u_neg)

        @test y_pos.rotors.thrust_n[1] < 0.0
        @test y_neg.rotors.thrust_n[1] > 0.0
    end

    @testset "Phase 4 - Wingtra-style twin forward props (yaw via differential thrust)" begin
        env = Env.EnvironmentModel(gravity = Env.UniformGravity(0.0))
        r = 0.5
        rotor_pos = SVector{2,T.Vec3}(T.vec3(0.0, r, 0.0), T.vec3(0.0, -r, 0.0))
        # Axis chosen so F = -T * axis gives forward +X force.
        rotor_axis = SVector{2,T.Vec3}(T.vec3(-1.0, 0.0, 0.0), T.vec3(-1.0, 0.0, 0.0))

        params = Sim.Vehicles.QuadrotorParams{2}(
            mass = 1.0,
            inertia_diag = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body = rotor_pos,
            rotor_axis_body = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )
        model = Sim.Vehicles.GenericMultirotor{2}(params)
        x = RB.RigidBodyState()

        # Differential thrust: right prop produces more thrust than left prop.
        T_left = 2.0
        T_right = 5.0
        out = Sim.Propulsion.RotorOutput{2}(
            thrust_n = SVector{2,Float64}(T_left, T_right),
            shaft_torque_nm = SVector{2,Float64}(0.0, 0.0),
            ω_rad_s = SVector{2,Float64}(0.0, 0.0),
            motor_current_a = SVector{2,Float64}(0.0, 0.0),
            bus_current_a = 0.0,
        )

        # Expected force: F = -T * axis -> (+X) thrust
        F_exp = T.vec3(T_left + T_right, 0.0, 0.0)
        # Expected yaw torque: r × F, with r at ±Y and F along +X
        τz = r * (T_right - T_left)
        τ_exp = T.vec3(0.0, 0.0, τz)

        d = Sim.Vehicles.dynamics(model, env, 0.0, x, out, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(d.vel_dot[1], F_exp[1]; atol = 1e-12)
        @test isapprox(d.vel_dot[2], F_exp[2]; atol = 1e-12)
        @test isapprox(d.vel_dot[3], F_exp[3]; atol = 1e-12)
        @test isapprox(d.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(d.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(d.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - Twin forward props roll torque from reaction torque" begin
        env = Env.EnvironmentModel(gravity = Env.UniformGravity(0.0))
        r = 0.5
        rotor_pos = SVector{2,T.Vec3}(T.vec3(0.0, r, 0.0), T.vec3(0.0, -r, 0.0))
        rotor_axis = SVector{2,T.Vec3}(T.vec3(-1.0, 0.0, 0.0), T.vec3(-1.0, 0.0, 0.0))

        params = Sim.Vehicles.QuadrotorParams{2}(
            mass = 1.0,
            inertia_diag = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body = rotor_pos,
            rotor_axis_body = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )
        model = Sim.Vehicles.GenericMultirotor{2}(params)
        x = RB.RigidBodyState()

        # Equal thrust; nonzero reaction torque on each prop (about +X due to axis).
        T_left = 3.0
        T_right = 3.0
        Q_left = 0.4
        Q_right = 0.6
        out = Sim.Propulsion.RotorOutput{2}(
            thrust_n = SVector{2,Float64}(T_left, T_right),
            shaft_torque_nm = SVector{2,Float64}(Q_left, Q_right),
            ω_rad_s = SVector{2,Float64}(0.0, 0.0),
            motor_current_a = SVector{2,Float64}(0.0, 0.0),
            bus_current_a = 0.0,
        )

        # r×F cancels in yaw; roll torque comes purely from axis*Q.
        τ_exp = T.vec3(-(Q_left + Q_right), 0.0, 0.0)

        d = Sim.Vehicles.dynamics(model, env, 0.0, x, out, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(d.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(d.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(d.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - CA axis param sign convention" begin
        rotor_pos = T.Vec3[T.vec3(0.0, 0.0, 0.0), T.vec3(0.0, 0.0, 0.0)]
        rotor_axis = T.Vec3[T.vec3(0.0, 0.0, 2.0), T.vec3(0.0, 1.0, 0.0)]

        airframe = Sim.Aircraft.AirframeSpec(
            kind = :multirotor,
            mass_kg = 1.0,
            inertia_diag_kgm2 = T.vec3(1.0, 1.0, 1.0),
            rotor_pos_body_m = rotor_pos,
            rotor_axis_body_m = rotor_axis,
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )

        motors = Sim.Aircraft.MotorSpec[
            Sim.Aircraft.MotorSpec(id = :motor1, channel = 1),
            Sim.Aircraft.MotorSpec(id = :motor2, channel = 2),
        ]

        actuation = Sim.Aircraft.ActuationSpec(
            motors = motors,
            servos = Sim.Aircraft.ServoSpec[],
            motor_actuators = Sim.Aircraft.DirectActuatorSpec(),
            servo_actuators = Sim.Aircraft.DirectActuatorSpec(),
        )

        spec = Sim.Aircraft.AircraftSpec(name = :test, airframe = airframe, actuation = actuation)
        vehicle = Sim.Aircraft._build_vehicle(spec)
        params = Sim.Aircraft._derive_ca_params(spec, vehicle)

        pmap = Dict(p.name => Float64(p.value) for p in params)
        @test isapprox(pmap["CA_ROTOR0_AZ"], -1.0; atol = 1e-12)
        @test isapprox(pmap["CA_ROTOR1_AY"], -1.0; atol = 1e-12)
    end
end


@testset "plant_outputs purity and RHS consistency" begin
    setup = _iris_fullplant()
    model = setup.model
    x0 = setup.plant0

    # Non-trivial input so we exercise the bus solve + battery currents.
    motors = SVector{12,Float64}(0.4, 0.4, 0.4, 0.4, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors)
    u = Sim.Plant.PlantInput(cmd = cmd, wind_ned = Sim.Types.vec3(3.0, 0.0, 0.0))
    t = 0.0

    # Snapshot a few mutable parameter fields; plant_outputs must not mutate them.
    batt = model.battery
    batt_snap = (
        soc0 = batt.soc0,
        v1_0 = batt.v1_0,
        ocv_soc = copy(batt.ocv_soc),
        ocv_v = copy(batt.ocv_v),
        r0 = batt.r0,
        r1 = batt.r1,
        c1 = batt.c1,
        min_voltage_v = batt.min_voltage_v,
    )
    prop = model.propulsion
    prop_snap = copy(prop.units)

    y1 = Sim.PlantModels.plant_outputs(model, t, x0, u)
    y2 = Sim.PlantModels.plant_outputs(model, t, x0, u)
    @test y1 == y2

    @test (
        soc0 = batt.soc0,
        v1_0 = batt.v1_0,
        ocv_soc = copy(batt.ocv_soc),
        ocv_v = copy(batt.ocv_v),
        r0 = batt.r0,
        r1 = batt.r1,
        c1 = batt.c1,
        min_voltage_v = batt.min_voltage_v,
    ) == batt_snap
    @test prop.units == prop_snap

    # Consistency: RHS uses the same bus current in the battery SoC derivative.
    dx = model(t, x0, u)
    I_rhs = -dx.batt_soc_dot * batt.capacity_c
    @test isfinite(y1.bus_current_a)
    @test isfinite(I_rhs)
    @test isapprox(I_rhs, y1.bus_current_a; rtol = 1e-12, atol = 1e-12)

    # Battery disconnected should force zero bus power and a disconnected status.
    u_off = Sim.Plant.PlantInput(
        cmd = cmd,
        wind_ned = u.wind_ned,
        faults = Sim.Faults.FaultState(battery_connected = false),
    )
    y_off = Sim.PlantModels.plant_outputs(model, t, x0, u_off)
    @test y_off.bus_current_a == 0.0
    @test y_off.bus_voltage_v == 0.0
    @test y_off.battery_status !== nothing
    @test y_off.battery_status.connected == false
end

@testset "Verification Tier 2 - Full-plant contract tests" begin
    @testset "Full-plant ballistic free-fall (no thrust, no wind)" begin
        # Goal: catch NED sign, gravity sign, quaternion handling, and engine stepping issues.
        # Setup: Iris full plant, motor duties=0 (default), rotor ω=0, no wind, no contact.

        t_end_s = 1.0
        dt_phys_s = 0.01
        dt_log_s = 0.01

        rb0 = RB.RigidBodyState(
            pos_ned=T.vec3(0.0, 0.0, -10.0),
            vel_ned=T.vec3(0.0, 0.0, 0.0),
            q_bn=T.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body=T.vec3(0.0, 0.0, 0.0),
        )

        setup = _iris_fullplant(
            ;
            x0 = rb0,
            contact = Sim.Contacts.NoContact(),
            linear_drag = 0.0,
            angular_damping = T.vec3(0.0, 0.0, 0.0),
        )

        t_end_us = UInt64(round(Int, t_end_s * 1e6))
        dt_phys_us = UInt64(round(Int, dt_phys_s * 1e6))
        dt_log_us = UInt64(round(Int, dt_log_s * 1e6))

        # No autopilot/wind ticks needed for this contract test.
        timeline = Sim.Runtime.build_timeline(
            UInt64(0),
            t_end_us;
            dt_ap_us=t_end_us + UInt64(1),
            dt_wind_us=t_end_us + UInt64(1),
            dt_log_us=dt_log_us,
            dt_phys_us=dt_phys_us,
            scn_times_us=UInt64[],
        )

        cfg = Sim.Runtime.EngineConfig(mode=Sim.Runtime.MODE_RECORD)
        bus = Sim.Runtime.SimBus(time_us = UInt64(0))
        rec = Sim.Recording.InMemoryRecorder()
        integrator = Sim.Integrators.RK4Integrator()

        eng = Sim.Runtime.Engine(
            cfg,
            timeline,
            bus,
            setup.plant0,
            setup.model,
            integrator,
            nothing, # autopilot
            nothing; # wind
            recorder=rec,
        )
        Sim.Runtime.run!(eng)

        @test haskey(rec.times, :plant)
        @test length(rec.times[:plant]) == length(rec.values[:plant])

        g = Env.gravity_accel(setup.env.gravity, rb0.pos_ned, 0.0)[3]
        z0 = rb0.pos_ned[3]
        vz0 = rb0.vel_ned[3]

        for (t_us, x) in zip(rec.times[:plant], rec.values[:plant])
            t = Float64(t_us) * 1e-6
            z_exp = z0 + vz0 * t + 0.5 * g * t * t
            vz_exp = vz0 + g * t

            @test isapprox(x.rb.pos_ned[3], z_exp; atol=1e-7)
            @test isapprox(x.rb.vel_ned[3], vz_exp; atol=1e-7)

            @test isapprox(_qnorm(x.rb.q_bn), 1.0; atol=1e-12)
            @test all(x.rotor_ω .>= -1e-12)
            @test 0.0 <= x.batt_soc <= 1.0
            @test isfinite(x.batt_v1)
        end
    end

    @testset "Hover force-balance RHS check (t=0)" begin
        # Goal: verify thrust direction and magnitude mapping into RB acceleration.
        # This is a *single RHS evaluation* contract test (not a long sim).

        setup = _iris_fullplant(; x0 = RB.RigidBodyState(), contact = Sim.Contacts.NoContact())

        # Solve for a rotor ω that yields total thrust ≈ m*g at Vax=0 and sea-level density.
        alt_msl_m = setup.env.origin.alt_msl_m - setup.plant0.rb.pos_ned[3]
        ρ = Env.air_density(setup.env.atmosphere, alt_msl_m)
        g = Env.gravity_accel(setup.env.gravity, setup.plant0.rb.pos_ned, 0.0)[3]
        m = Sim.Vehicles.mass(setup.veh.model)

        # Assume symmetric rotors for Iris.
        prop = setup.veh.propulsion.units[1].prop

        function total_thrust(ω::Float64)
            Ti = Sim.Propulsion.prop_thrust(prop, ρ, ω, 0.0)
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
        @test total_thrust(ω_hi) >= T_target

        for _ in 1:60
            ω_mid = 0.5 * (ω_lo + ω_hi)
            if total_thrust(ω_mid) >= T_target
                ω_hi = ω_mid
            else
                ω_lo = ω_mid
            end
        end
        ω_hover = ω_hi

        # Build a PlantState with rotor ω initialized to hover ω.
        x = setup.plant0
        x = Sim.Plant.PlantState{4}(
            x.rb,
            x.motors_y,
            x.motors_ydot,
            x.servos_y,
            x.servos_ydot,
            SVector{4,Float64}(fill(ω_hover, 4)),
            x.batt_soc,
            x.batt_v1,
        )

        u = Sim.Plant.PlantInput() # cmd=0, wind=0, faults=none
        dx = setup.model(0.0, x, u)

        # At identity attitude, hover thrust should cancel gravity => a_down ≈ 0.
        @test isapprox(dx.rb.vel_dot[3], 0.0; atol=1e-6)
        @test isapprox(dx.rb.vel_dot[1], 0.0; atol=1e-9)
        @test isapprox(dx.rb.vel_dot[2], 0.0; atol=1e-9)
    end

    @testset "Bus solve residual sweep" begin
        # Goal: verify the bus-voltage solver returns a consistent fixed-point.
        # This is a deterministic envelope sweep over random-but-safe inputs.

        p = Sim.Propulsion.default_iris_quadrotor_set()
        rng = MersenneTwister(123)

        ocv = 16.0
        R0 = 0.05
        Vmin = 0.0

        for _ in 1:25
            ω = SVector{4,Float64}(rand(rng, 4) .* 600.0)
            duty = SVector{4,Float64}(rand(rng, 4))
            v1 = 0.5 * rand(rng)

            V = Sim.PlantModels._solve_bus_voltage(p, ω, duty, ocv, v1, R0, Vmin)
            @test V >= Vmin - 1e-12
            @test V <= (ocv - v1) + 1e-12

            I = Sim.PlantModels._bus_current_total(p, ω, duty, V)
            V_rhs = clamp(Vmin, (ocv - v1) - R0 * I, (ocv - v1))
            @test isapprox(V, V_rhs; atol=1e-6)
        end

        # Simple monotonic sanity: increasing duty should not increase bus voltage.
        ω0 = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0)
        duty1 = SVector{4,Float64}(0.2, 0.2, 0.2, 0.2)
        duty2 = SVector{4,Float64}(0.4, 0.4, 0.4, 0.4)
        v1 = 0.0

        V1 = Sim.PlantModels._solve_bus_voltage(p, ω0, duty1, ocv, v1, R0, Vmin)
        V2 = Sim.PlantModels._solve_bus_voltage(p, ω0, duty2, ocv, v1, R0, Vmin)
        @test V2 <= V1 + 1e-9
    end

    @testset "Quad symmetry torque test" begin
        # Roll/pitch cancellation for symmetric thrust distribution.
        model = Sim.Vehicles.IrisQuadrotor()
        env = Sim.Environment.EnvironmentModel(wind = Sim.Environment.NoWind())

        x = RB.RigidBodyState()  # identity attitude, zero rates

        thrust = 5.0
        Q = 0.05

        # Balanced yaw torque (two +Q, two -Q) should yield ~zero body angular acceleration.
        u_bal = Sim.Propulsion.RotorOutput{4}(
            thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
            shaft_torque_nm = SVector{4,Float64}(Q, Q, -Q, -Q),
            ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            bus_current_a = 0.0,
        )
        dx = Sim.Vehicles.dynamics(model, env, 0.0, x, u_bal, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(dx.ω_dot[1], 0.0; atol=1e-12)
        @test isapprox(dx.ω_dot[2], 0.0; atol=1e-12)
        @test isapprox(dx.ω_dot[3], 0.0; atol=1e-12)

        # Non-canceling yaw torque should produce a clear yaw acceleration sign.
        u_yaw_pos = Sim.Propulsion.RotorOutput{4}(
            thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
            shaft_torque_nm = SVector{4,Float64}(Q, Q, Q, Q),
            ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            bus_current_a = 0.0,
        )
        dxp = Sim.Vehicles.dynamics(model, env, 0.0, x, u_yaw_pos, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(dxp.ω_dot[1], 0.0; atol=1e-12)
        @test isapprox(dxp.ω_dot[2], 0.0; atol=1e-12)
        @test dxp.ω_dot[3] > 0.0

        u_yaw_neg = Sim.Propulsion.RotorOutput{4}(
            thrust_n = SVector{4,Float64}(thrust, thrust, thrust, thrust),
            shaft_torque_nm = SVector{4,Float64}(-Q, -Q, -Q, -Q),
            ω_rad_s = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            motor_current_a = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0),
            bus_current_a = 0.0,
        )
        dxn = Sim.Vehicles.dynamics(model, env, 0.0, x, u_yaw_neg, T.vec3(0.0, 0.0, 0.0))
        @test isapprox(dxn.ω_dot[1], 0.0; atol=1e-12)
        @test isapprox(dxn.ω_dot[2], 0.0; atol=1e-12)
        @test dxn.ω_dot[3] < 0.0

        # rotor_dir sign sensitivity: propulsion should apply rotor_dir exactly once to the yaw reaction torque.
        setup = _iris_fullplant()
        veh = setup.veh
        env2 = setup.env
        full_model = setup.model
        plant0 = setup.plant0
        ω0 = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
        motors_y = SVector{12,Float64}(0.6, 0.6, 0.6, 0.6, 0, 0, 0, 0, 0, 0, 0, 0)
        xplant = Sim.Plant.PlantState{4}(
            rb = plant0.rb,
            motors_y = motors_y,
            motors_ydot = plant0.motors_ydot,
            servos_y = plant0.servos_y,
            servos_ydot = plant0.servos_ydot,
            rotor_ω = ω0,
            batt_soc = 1.0,
            batt_v1 = 0.0,
        )
        cmd = Sim.Vehicles.ActuatorCommand(
            motors = motors_y,
            servos = plant0.servos_y,
        )
        u = Sim.Plant.PlantInput(cmd = cmd, wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0), faults = Sim.Faults.FaultState())
        y1 = Sim.plant_outputs(full_model, 0.0, xplant, u)
        prop2 = Sim.Propulsion.QuadRotorSet{4}(veh.propulsion.units, -veh.propulsion.rotor_dir)
        model2 = Sim.PlantModels.CoupledMultirotorModel(
            veh.model,
            env2,
            Sim.Contacts.NoContact(),
            veh.motor_actuators,
            veh.servo_actuators,
            prop2,
            setup.batt,
        )
        y2 = Sim.plant_outputs(model2, 0.0, xplant, u)
        @test y2.rotors.shaft_torque_nm ≈ -y1.rotors.shaft_torque_nm
    end

    @testset "Fault semantics (motor disable / battery disconnect / estimator freeze)" begin
        # Plant-level fault consumption: motor disable forces duty=0 -> motor current drops to 0 and ω̇ < 0.
        setup = _iris_fullplant()
        model = setup.model
        plant0 = setup.plant0
        t = 0.0

        # Construct a state with nonzero rotor speed so disable has an immediate effect.
        ω0 = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
        motors_y = SVector{12,Float64}(0.6, 0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        x = Sim.Plant.PlantState{4}(
            rb = plant0.rb,
            motors_y = motors_y,
            motors_ydot = plant0.motors_ydot,
            servos_y = plant0.servos_y,
            servos_ydot = plant0.servos_ydot,
            rotor_ω = ω0,
            batt_soc = 1.0,
            batt_v1 = 0.0,
        )

        cmd = Sim.Vehicles.ActuatorCommand(
            motors = motors_y,
            servos = plant0.servos_y,
        )

        u_nom = Sim.Plant.PlantInput(
            cmd = cmd,
            wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            faults = Sim.Faults.FaultState(),
        )
        y_nom = Sim.plant_outputs(model, t, x, u_nom)
        dx_nom = model(t, x, u_nom)

        u_dis = Sim.Plant.PlantInput(
            cmd = cmd,
            wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            faults = Sim.Faults.FaultState(motor_disable_mask = UInt32(0x1)),
        )
        y_dis = Sim.plant_outputs(model, t, x, u_dis)
        dx_dis = model(t, x, u_dis)

        @test y_dis.rotors.motor_current_a[1] == 0.0
        @test y_nom.rotors.motor_current_a[1] > 0.0
        @test y_dis.rotors.bus_current_a < y_nom.rotors.bus_current_a + 1e-12
        @test dx_dis.rotor_ω_dot[1] < 0.0
        @test dx_dis.rotor_ω_dot[1] < dx_nom.rotor_ω_dot[1] + 1e-12

        # Battery disconnect collapses bus V/I and motor current.
        u_disc = Sim.Plant.PlantInput(
            cmd = cmd,
            wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            faults = Sim.Faults.FaultState(battery_connected = false),
        )
        y_disc = Sim.plant_outputs(model, t, x, u_disc)
        @test y_disc.battery_status.connected == false
        @test isapprox(y_disc.bus_voltage_v, 0.0; atol=1e-12)
        @test isapprox(y_disc.rotors.bus_current_a, 0.0; atol=1e-12)
        @test all(y_disc.rotors.motor_current_a .== 0.0)

        # Estimator freeze: the estimator must not update bus.est while the freeze bit is set.
        bus = Sim.Runtime.SimBus(time_us = UInt64(0))
        plant_rb = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(1.0, 2.0, 3.0))
        bus.est = Sim.Estimators.EstimatedState(
            pos_ned = Sim.Types.vec3(9.0, 9.0, 9.0),
            vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        est = Sim.Sources.LiveEstimatorSource(
            Sim.Estimators.TruthEstimator(),
            MersenneTwister(123),
            0.01,
        )

        bus.faults = Sim.Faults.FaultState(sensor_fault_mask = Sim.Faults.SENSOR_FAULT_EST_FREEZE)
        Sim.Runtime.update!(est, bus, plant_rb, UInt64(0))
        @test bus.est.pos_ned == Sim.Types.vec3(9.0, 9.0, 9.0)

        bus.faults = Sim.Faults.FaultState()
        Sim.Runtime.update!(est, bus, plant_rb, UInt64(0))
        @test bus.est.pos_ned == plant_rb.pos_ned
    end

    @testset "Engine boundary ordering probe test" begin
        # Verify that the canonical boundary protocol ordering is enforced in code:
        # scenario -> wind -> estimator -> autopilot for boundaries where all are due.
        mutable struct ProbeScenarioSource end
        mutable struct ProbeWindSource end
        mutable struct ProbeEstimatorSource end
        mutable struct ProbeAutopilotSource end

        function Sim.Runtime.update!(
            ::ProbeScenarioSource,
            bus::Sim.Runtime.SimBus,
            plant,
            t_us::UInt64,
        )
            bus.wind_ned = Sim.Types.vec3(1.0, 0.0, 0.0)
            return nothing
        end

        function Sim.Runtime.update!(
            ::ProbeWindSource,
            bus::Sim.Runtime.SimBus,
            plant,
            t_us::UInt64,
        )
            @test bus.wind_ned[1] == 1.0
            bus.wind_ned = Sim.Types.vec3(2.0, 0.0, 0.0)
            return nothing
        end

        function Sim.Runtime.update!(
            ::ProbeEstimatorSource,
            bus::Sim.Runtime.SimBus,
            plant,
            t_us::UInt64,
        )
            @test bus.wind_ned[1] == 2.0
            bus.wind_ned = Sim.Types.vec3(3.0, 0.0, 0.0)
            return nothing
        end

        function Sim.Runtime.update!(
            ::ProbeAutopilotSource,
            bus::Sim.Runtime.SimBus,
            plant,
            t_us::UInt64,
        )
            @test bus.wind_ned[1] == 3.0
            bus.wind_ned = Sim.Types.vec3(4.0, 0.0, 0.0)
            return nothing
        end

        struct ZeroRB end
        function (d::ZeroRB)(t_s::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
            return Sim.RigidBody.rb_deriv_zero()
        end

        t_end_us = UInt64(30_000)
        tl = Sim.Runtime.build_timeline(
            UInt64(0),
            t_end_us;
            dt_ap_us = UInt64(10_000),
            dt_wind_us = UInt64(10_000),
            dt_log_us = UInt64(10_000),
        )

        cfg = Sim.Runtime.EngineConfig(mode = Sim.Runtime.MODE_LIVE, enable_derived_outputs = false)
        sim = Sim.Runtime.Engine(
            cfg,
            tl,
            Sim.Runtime.SimBus(time_us = UInt64(0)),
            RB.RigidBodyState(),
            ZeroRB(),
            Sim.Integrators.EulerIntegrator(),
            ProbeAutopilotSource(),
            ProbeWindSource();
            scenario = ProbeScenarioSource(),
            estimator = ProbeEstimatorSource(),
            telemetry = Sim.Runtime.NullTelemetry(),
            recorder = Sim.Recording.NullRecorder(),
        )

        Sim.Runtime.run!(sim)
        @test sim.bus.wind_ned[1] == 4.0
    end
end

@testset "Verification Tier 3 - System regression" begin
    @testset "Iris mission open-loop local defect (record/replay)" begin
        # TODO: implement local defect test for Iris mission recording.
        #
        # Key idea:
        # - Use one recording (Tier0) with held inputs per interval.
        # - For each [t_k, t_{k+1}] interval, start *both* integrators from x_ref(t_k)
        #   (not their own drifted states), integrate to t_{k+1}, and compare.
        #
        # This avoids confusing unstable open-loop divergence with solver accuracy.
        @test_skip true
    end
end
