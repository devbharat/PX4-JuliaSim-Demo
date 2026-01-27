using Test

@testset "AircraftSpec path resolution (must_exist)" begin
    mktempdir() do dir
        toml_extends = """
        schema_version = 1
        extends = ["does_not_exist.toml"]
        """
        path_extends = joinpath(dir, "extends.toml")
        write(path_extends, toml_extends)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_extends; strict = true)

        toml_run = """
        schema_version = 1
        [run]
        mode = "replay"
        recording_in = "missing_recording.bin"
        """
        path_run = joinpath(dir, "run.toml")
        write(path_run, toml_run)
        @test_throws ErrorException Sim.Aircraft.run_spec(
            path_run;
            strict = true,
            base_spec = :default,
        )
    end
end

@testset "AircraftSpec TOML uORB instance validation" begin
    mktempdir() do dir
        toml_pub_ok = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -1
        """
        path_pub_ok = joinpath(dir, "pub_ok.toml")
        write(path_pub_ok, toml_pub_ok)
        _ = Sim.Aircraft.load_spec(path_pub_ok; strict = true)

        toml_pub_default = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        """
        path_pub_default = joinpath(dir, "pub_default.toml")
        write(path_pub_default, toml_pub_default)
        _ = Sim.Aircraft.load_spec(path_pub_default; strict = true)

        toml_pub_bad = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -2
        """
        path_pub_bad = joinpath(dir, "pub_bad.toml")
        write(path_pub_bad, toml_pub_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_pub_bad; strict = true)

        toml_sub_bad = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.subs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -1
        """
        path_sub_bad = joinpath(dir, "sub_bad.toml")
        write(path_sub_bad, toml_sub_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_sub_bad; strict = true)

        toml_sub_default = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.subs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        """
        path_sub_default = joinpath(dir, "sub_default.toml")
        write(path_sub_default, toml_sub_default)
        _ = Sim.Aircraft.load_spec(path_sub_default; strict = true)
    end
end

@testset "AircraftSpec TOML inertia parsing" begin
    mktempdir() do dir
        toml_matrix = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [
          [0.02, 0.001, 0.0],
          [0.001, 0.03, 0.0],
          [0.0, 0.0, 0.04],
        ]
        """
        path_matrix = joinpath(dir, "inertia_matrix.toml")
        write(path_matrix, toml_matrix)
        spec = Sim.Aircraft.load_spec(path_matrix; strict = true, base_spec = :default)
        @test spec.airframe.inertia_diag_kgm2 == Sim.Types.vec3(0.02, 0.03, 0.04)
        @test spec.airframe.inertia_products_kgm2 == Sim.Types.vec3(0.001, 0.0, 0.0)

        toml_flat = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [0.02, 0.001, 0.0,
                       0.001, 0.03, 0.0,
                       0.0, 0.0, 0.04]
        """
        path_flat = joinpath(dir, "inertia_flat.toml")
        write(path_flat, toml_flat)
        spec_flat = Sim.Aircraft.load_spec(path_flat; strict = true, base_spec = :default)
        @test spec_flat.airframe.inertia_diag_kgm2 == Sim.Types.vec3(0.02, 0.03, 0.04)
        @test spec_flat.airframe.inertia_products_kgm2 == Sim.Types.vec3(0.001, 0.0, 0.0)

        toml_nonsym = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [
          [0.02, 0.1, 0.0],
          [0.001, 0.03, 0.0],
          [0.0, 0.0, 0.04],
        ]
        """
        path_nonsym = joinpath(dir, "inertia_nonsym.toml")
        write(path_nonsym, toml_nonsym)
        @test_throws ErrorException Sim.Aircraft.load_spec(
            path_nonsym;
            strict = true,
            base_spec = :default,
        )
    end
end

@testset "AircraftSpec inertia validation (SPD)" begin
    mktempdir() do dir
        toml_bad = """
        schema_version = 1
        [airframe]
        inertia_diag_kgm2 = [1.0, 1.0, 1.0]
        inertia_products_kgm2 = [2.0, 0.0, 0.0]
        """
        path_bad = joinpath(dir, "inertia_bad.toml")
        write(path_bad, toml_bad)
        spec_bad = Sim.Aircraft.load_spec(path_bad; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_bad)
    end
end

@testset "AircraftSpec integrator table parsing" begin
    mktempdir() do dir
        toml_integrator = """
        schema_version = 1
        [plant.integrator]
        kind = "RK45"
        rtol_pos = 1e-5
        atol_omega = 1e-4
        max_substeps = 12345
        quantize_us = false
        """
        path_integrator = joinpath(dir, "integrator.toml")
        write(path_integrator, toml_integrator)
        spec = Sim.Aircraft.load_spec(path_integrator; strict = true)
        integ = spec.plant.integrator
        @test integ isa Sim.Integrators.RK45Integrator
        @test integ.rtol_pos == 1e-5
        @test integ.atol_ω == 1e-4
        @test integ.max_substeps == 12345
        @test integ.quantize_us == false

        toml_bad_kind = """
        schema_version = 1
        [plant.integrator]
        kind = "RK99"
        """
        path_bad = joinpath(dir, "integrator_bad.toml")
        write(path_bad, toml_bad_kind)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_bad; strict = true)
    end
end

@testset "AircraftSpec integrator string normalization" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [plant]
        integrator = "rk45"
        """
        path = joinpath(dir, "integrator_string.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true)
        @test spec.plant.integrator === :RK45
    end
end

@testset "AircraftSpec environment/scenario/estimator parsing" begin
    mktempdir() do dir
        toml_env = """
        schema_version = 1
        [environment]
        wind = "ou"
        wind_mean_ned = [1.0, 2.0, 3.0]
        wind_sigma_ned = [0.1, 0.2, 0.3]
        wind_tau_s = 4.0
        atmosphere = "isa1976"
        gravity = "uniform"
        gravity_mps2 = 9.7

        [scenario]
        arm_time_s = 0.5
        mission_time_s = 1.5

        [estimator]
        kind = "noisy_delayed"
        pos_sigma_m = [0.1, 0.2, 0.3]
        vel_sigma_mps = [0.4, 0.5, 0.6]
        yaw_sigma_rad = 0.02
        rate_sigma_rad_s = [0.01, 0.02, 0.03]
        bias_tau_s = 40.0
        rate_bias_sigma_rad_s = [0.004, 0.005, 0.006]
        delay_s = 0.008
        dt_est_s = 0.004
        """
        path_env = joinpath(dir, "env_estimator.toml")
        write(path_env, toml_env)
        spec = Sim.Aircraft.load_spec(path_env; strict = true)
        @test spec.environment.wind == :ou
        @test spec.environment.wind_mean_ned == Sim.Types.vec3(1.0, 2.0, 3.0)
        @test spec.environment.wind_sigma_ned == Sim.Types.vec3(0.1, 0.2, 0.3)
        @test spec.environment.wind_tau_s == 4.0
        @test spec.environment.atmosphere == :isa1976
        @test spec.environment.gravity == :uniform
        @test spec.environment.gravity_mps2 == 9.7
        @test spec.scenario.arm_time_s == 0.5
        @test spec.scenario.mission_time_s == 1.5
        @test spec.estimator.kind == :noisy_delayed
        @test spec.estimator.pos_sigma_m == Sim.Types.vec3(0.1, 0.2, 0.3)
        @test spec.estimator.vel_sigma_mps == Sim.Types.vec3(0.4, 0.5, 0.6)
        @test spec.estimator.yaw_sigma_rad == 0.02
        @test spec.estimator.rate_sigma_rad_s == Sim.Types.vec3(0.01, 0.02, 0.03)
        @test spec.estimator.bias_tau_s == 40.0
        @test spec.estimator.rate_bias_sigma_rad_s == Sim.Types.vec3(0.004, 0.005, 0.006)
        @test spec.estimator.delay_s == 0.008
        @test spec.estimator.dt_est_s == 0.004

        toml_est_none = """
        schema_version = 1
        [estimator]
        kind = "none"
        """
        path_none = joinpath(dir, "estimator_none.toml")
        write(path_none, toml_est_none)
        spec_none = Sim.Aircraft.load_spec(path_none; strict = true)
        @test spec_none.estimator.kind == :none
    end
end

@testset "AircraftSpec validation errors (environment/scenario/estimator/power)" begin
    mktempdir() do dir
        # OU wind requires tau > 0.
        toml_wind = """
        schema_version = 1
        [environment]
        wind = "ou"
        wind_tau_s = 0.0
        """
        path_wind = joinpath(dir, "wind_bad.toml")
        write(path_wind, toml_wind)
        spec_wind = Sim.Aircraft.load_spec(path_wind; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_wind)

        # Uniform gravity requires g > 0.
        toml_grav = """
        schema_version = 1
        [environment]
        gravity = "uniform"
        gravity_mps2 = 0.0
        """
        path_grav = joinpath(dir, "grav_bad.toml")
        write(path_grav, toml_grav)
        spec_grav = Sim.Aircraft.load_spec(path_grav; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_grav)

        # Scenario times must be microsecond-quantized.
        toml_scn = """
        schema_version = 1
        [scenario]
        arm_time_s = 0.0000005
        mission_time_s = 0.0
        """
        path_scn = joinpath(dir, "scenario_bad.toml")
        write(path_scn, toml_scn)
        spec_scn = Sim.Aircraft.load_spec(path_scn; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_scn)

        # Estimator dt must match dt_autopilot_s.
        toml_est = """
        schema_version = 1
        [timeline]
        dt_autopilot_s = 0.004
        [estimator]
        kind = "noisy_delayed"
        dt_est_s = 0.002
        """
        path_est = joinpath(dir, "estimator_bad.toml")
        write(path_est, toml_est)
        spec_est = Sim.Aircraft.load_spec(path_est; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_est)

        # Power share_mode must be recognized.
        toml_power = """
        schema_version = 1
        [power]
        share_mode = "bogus"
        """
        path_power = joinpath(dir, "power_bad.toml")
        write(path_power, toml_power)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_power; strict = true)

        # Each bus must have at least one battery.
        toml_bus = """
        schema_version = 1
        [power]
        [[power.buses]]
        id = "main"
        battery_ids = []
        motor_ids = ["motor1", "motor2", "motor3", "motor4"]
        servo_ids = []
        avionics_load_w = 0.0
        """
        path_bus = joinpath(dir, "bus_no_battery.toml")
        write(path_bus, toml_bus)
        spec_bus = Sim.Aircraft.load_spec(path_bus; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_bus)
    end
end

@testset "AircraftSpec inertia conflict strict mode" begin
    mktempdir() do dir
        toml_conflict = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [
          [0.02, 0.001, 0.0],
          [0.001, 0.03, 0.0],
          [0.0, 0.0, 0.04],
        ]
        inertia_diag_kgm2 = [0.02, 0.03, 0.04]
        """
        path = joinpath(dir, "inertia_conflict.toml")
        write(path, toml_conflict)
        @test_throws ErrorException Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
    end
end

@testset "AircraftSpec rotor_dir validation" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [airframe]
        rotor_pos_body_m = [[0.1, 0.0, 0.0], [-0.1, 0.0, 0.0]]
        rotor_axis_body_m = [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]
        [actuation]
        [[actuation.motors]]
        id = "m1"
        channel = 1
        [[actuation.motors]]
        id = "m2"
        channel = 2
        [airframe.propulsion]
        rotor_dir = [1.0, 0.5]
        """
        path = joinpath(dir, "rotor_dir_bad.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec)
    end
end

@testset "AircraftSpec propulsion + power share_mode parsing" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [airframe.propulsion]
        km_m = 0.04
        V_nom = 11.5
        rho_nom = 1.1
        rotor_radius_m = 0.2
        inflow_kT = 7.0
        inflow_kQ = 6.0
        thrust_calibration_mult = 2.5

        [airframe.propulsion.esc]
        eta = 0.95
        deadzone = 0.05

        [airframe.propulsion.motor]
        kv_rpm_per_volt = 1000.0
        r_ohm = 0.2
        j_kgm2 = 2.0e-5
        i0_a = 0.4
        viscous_friction_nm_per_rad_s = 1.0e-6
        max_current_a = 55.0

        [power]
        share_mode = "equal"
        """
        path = joinpath(dir, "propulsion_power.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true)
        p = spec.airframe.propulsion
        @test p.km_m == 0.04
        @test p.V_nom == 11.5
        @test p.rho_nom == 1.1
        @test p.rotor_radius_m == 0.2
        @test p.inflow_kT == 7.0
        @test p.inflow_kQ == 6.0
        @test p.thrust_calibration_mult == 2.5
        @test p.esc.eta == 0.95
        @test p.esc.deadzone == 0.05
        @test p.motor.kv_rpm_per_volt == 1000.0
        @test p.motor.r_ohm == 0.2
        @test p.motor.j_kgm2 == 2.0e-5
        @test p.motor.i0_a == 0.4
        @test p.motor.viscous_friction_nm_per_rad_s == 1.0e-6
        @test p.motor.max_current_a == 55.0
        @test spec.power.share_mode == :equal
    end
end

@testset "AircraftSpec hover calibration uses environment gravity" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [environment]
        gravity = "uniform"
        gravity_mps2 = 3.71
        """
        path = joinpath(dir, "gravity.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        veh = Sim.Aircraft._build_vehicle(spec)

        p = spec.airframe.propulsion
        N = length(spec.actuation.motors)
        expected = Sim.Propulsion.default_multirotor_set(
            N = N,
            km_m = p.km_m,
            V_nom = p.V_nom,
            ρ_nom = p.rho_nom,
            thrust_hover_per_rotor_n = spec.airframe.mass_kg * spec.environment.gravity_mps2 / N,
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

        @test isapprox(
            veh.propulsion.units[1].prop.kT,
            expected.units[1].prop.kT;
            rtol = 0.0,
            atol = 1e-12,
        )
        @test isapprox(
            veh.propulsion.units[1].prop.kQ,
            expected.units[1].prop.kQ;
            rtol = 0.0,
            atol = 1e-12,
        )
    end
end
