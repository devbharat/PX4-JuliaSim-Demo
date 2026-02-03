using PrecompileTools: @setup_workload, @compile_workload

@setup_workload begin
    using .Sim
    using .Sim.Verification
    using .Sim.Integrators
    using .Sim.Runtime
    using .Sim.Recording
    using .Sim.Types
    using .Sim.RigidBody
    using .Sim.Environment
    using .Sim.Powertrain
    using .Sim.Plant
    using .Tests
    using Random
    using StaticArrays: SVector

    @compile_workload begin
        Tests.Workloads.precompile_unit()
        # Verification Tier 1: environment + OU wind
        atm = Environment.ISA1976()
        Environment.air_temperature(atm, 0.0)
        Environment.air_pressure(atm, 0.0)
        Environment.air_density(atm, 0.0)

        ou = Environment.OUWind(σ = Types.vec3(1.0, 1.0, 1.0), τ_s = 5.0, v_gust = Types.vec3(0.1, -0.2, 0.0))
        rng = Random.MersenneTwister(0x12345678)
        Environment.step_wind!(ou, Types.vec3(0.0, 0.0, 0.0), 0.0, 0.1, rng)

        # Verification Tier 1: battery stepping + status
        batt = Powertrain.TheveninBattery(
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
        st = Powertrain.battery_state(batt)
        Powertrain.step!(batt, st, 5.0, 0.01)
        Powertrain.r0_ohm(batt.r0_model, st.soc, batt.temp_c)
        Powertrain.status(batt, st)

        # Verification Tier 1: plant power math
        power0 = Plant.PowerState{2}(soc = SVector{2,Float64}(1.0, 0.5), v1 = SVector{2,Float64}(0.1, 0.2))
        x0 = Plant.PlantState{4,2}(power = power0)
        power_dot = Plant.PowerDeriv{2}(soc_dot = SVector{2,Float64}(-0.01, -0.02), v1_dot = SVector{2,Float64}(0.03, 0.04))
        k = Plant.PlantDeriv{4,2}(power = power_dot)
        Plant.plant_add(x0, k, 2.0)
        Plant.plant_scale_add(x0, k, k, k, k, 2.0)
        Plant.plant_lincomb(x0, 2.0, (k, k), (0.25, 0.75))

        case = Verification.SHOCase()
        f = Verification.sho_rhs(case)
        x0 = RigidBody.RigidBodyState(
            pos_ned = Types.vec3(case.x0, 0.0, 0.0),
            vel_ned = Types.vec3(case.v0, 0.0, 0.0),
            q_bn = Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = Types.vec3(0.0, 0.0, 0.0),
        )

        Verification.simulate_trajectory(Integrators.EulerIntegrator(), f, x0, 0.01, 0.1)
        Verification.simulate_trajectory(Integrators.RK4Integrator(), f, x0, 0.01, 0.1)
        Verification.simulate_trajectory(Integrators.RK23Integrator(), f, x0, 0.01, 0.1)
        Verification.simulate_trajectory(Integrators.RK45Integrator(), f, x0, 0.01, 0.1)

        t0_us = UInt64(0)
        t_end_us = UInt64(100_000)
        dt_us = UInt64(20_000)
        tl = Runtime.build_timeline(
            t0_us,
            t_end_us;
            dt_ap_us = dt_us,
            dt_wind_us = dt_us,
            dt_log_us = dt_us,
            dt_phys_us = dt_us,
            scn_times_us = UInt64[],
        )

        rec = Recording.InMemoryRecorder()
        Recording.prepare!(rec, tl; record_estimator = false, record_faults_evt = true)
    end
end
