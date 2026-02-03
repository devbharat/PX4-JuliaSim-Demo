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
const AC = Sim.Aircraft
const RB = Sim.RigidBody
const Scen = Sim.Scenario
const Ev = Sim.Events


@testset "Verification Tier 1 - Subsystems" begin
    @testset "Environment ISA1976 spot checks" begin
        env = PX4Lockstep.Tests.Fixtures.tier1_env_isa1976_spot_checks()
        @test isapprox(env.t0, 288.15; atol = 1e-9)
        @test isapprox(env.p0, 101325.0; atol = 1e-9)
        @test isapprox(env.rho0, 1.225; rtol = 1e-3)
        @test isapprox(env.t11, 216.65; atol = 1e-3)
        @test isapprox(env.p11, 22632.1; rtol = 1e-2)
        @test isapprox(env.rho11, 0.36391; rtol = 1e-2)
        @test env.rho0_gt_rho1k
        @test env.p0_gt_p1k
    end

    @testset "OUWind discrete update contract" begin
        ou = PX4Lockstep.Tests.Fixtures.tier1_ouwind_discrete_update()
        @test isapprox(ou.phi, exp(-0.1 / 5.0); atol = 1e-14)
        @test isapprox(ou.scale, sqrt(1.0 - ou.phi * ou.phi); atol = 1e-14)
        @test isapprox(ou.v_gust, ou.expected; atol = 1e-12)
    end

    @testset "Powertrain Thevenin battery analytic step-load" begin
        batt = PX4Lockstep.Tests.Fixtures.tier1_battery_step_load()
        @test batt.max_soc_err <= 1e-12
        @test batt.max_v1_err <= 1e-11
        @test batt.max_vt_err <= 1e-10
        @test batt.soc < batt.soc0
        @test batt.v1 > 0.0
    end

    @testset "Phase 5.1 - multi-battery PlantState math" begin
        pm = PX4Lockstep.Tests.Fixtures.tier1_multi_battery_plant_state_math()
        @test pm.x1_soc ≈ pm.exp1_soc
        @test pm.x1_v1 ≈ pm.exp1_v1
        @test pm.x2_soc ≈ pm.exp1_soc
        @test pm.x2_v1 ≈ pm.exp1_v1
        @test pm.x3_soc ≈ pm.exp_soc
        @test pm.x3_v1 ≈ pm.exp_v1
    end

    @testset "Phase 5.1 - init_plant_state with multiple batteries" begin
        init = PX4Lockstep.Tests.Fixtures.tier1_init_plant_state_multi_battery()
        @test init.soc == SVector{2,Float64}(0.9, 0.8)
        @test init.v1 == SVector{2,Float64}(0.12, 0.34)
    end

    @testset "Phase 5.1 - when_soc_below uses minimum SOC" begin
        sb = PX4Lockstep.Tests.Fixtures.tier1_when_soc_below_min()
        @test sb.event_count == 1
        @test sb.event_is_when
        @test sb.cond_low == true
        @test sb.cond_high == false
    end

    @testset "Phase 5.2 - power network current sharing + avionics load" begin
        ps = PX4Lockstep.Tests.Fixtures.tier1_power_network_share()
        @test isapprox(ps.I1 + ps.I2, ps.total; rtol = 1e-6, atol = 1e-6)
        @test isapprox(ps.ratio, 2.0; rtol = 1e-2, atol = 1e-2)
        @test isapprox(ps.I1_eq, ps.I2_eq; rtol = 1e-6, atol = 1e-6)
    end

    @testset "Phase 5.2 - multi-bus voltage mapping" begin
        mb = PX4Lockstep.Tests.Fixtures.tier1_multi_bus_voltage()
        @test mb.b1_v < mb.b2_v
        @test isapprox(mb.b2_v, 12.0; atol = 1e-3)
    end

    @testset "Phase 5.2 - back-EMF can zero motor bus load" begin
        be = PX4Lockstep.Tests.Fixtures.tier1_back_emf_zero_bus_load()
        @test isapprox(be.bus_current, 0.0; atol = 1e-6)
        @test isapprox(be.bus_voltage, be.ocv; atol = 1e-6)
    end

    @testset "Phase 4 - Propulsor axis geometry" begin
        pa = PX4Lockstep.Tests.Fixtures.tier1_propulsor_axis_geometry()
        @test isapprox(pa.vel_dot[1], -2.0; atol = 1e-12)
        @test isapprox(pa.vel_dot[2], 0.0; atol = 1e-12)
        @test isapprox(pa.vel_dot[3], 0.0; atol = 1e-12)
        @test isapprox(pa.ω_dot[1], 0.5; atol = 1e-12)
        @test isapprox(pa.ω_dot[2], 0.0; atol = 1e-12)
        @test isapprox(pa.ω_dot[3], 0.0; atol = 1e-12)
    end

    @testset "Phase 4 - Wrench composition (r×F + axis*Q)" begin
        wr = PX4Lockstep.Tests.Fixtures.tier1_wrench_composition()
        F_exp = T.vec3(0.0, -3.0, -2.0)
        τ_exp = T.vec3(0.0, 2.2, 0.5)
        @test isapprox(wr.vel_dot[1], F_exp[1]; atol = 1e-12)
        @test isapprox(wr.vel_dot[2], F_exp[2]; atol = 1e-12)
        @test isapprox(wr.vel_dot[3], F_exp[3]; atol = 1e-12)
        @test isapprox(wr.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(wr.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(wr.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - Vax sign from axis projection" begin
        vax = PX4Lockstep.Tests.Fixtures.tier1_vax_sign_projection()
        @test vax.thrust_pos < 0.0
        @test vax.thrust_neg > 0.0
    end

    @testset "Phase 4 - twin forward props (yaw via differential thrust)" begin
        ty = PX4Lockstep.Tests.Fixtures.tier1_twin_forward_props_yaw()
        T_left = 2.0
        T_right = 5.0
        F_exp = T.vec3(T_left + T_right, 0.0, 0.0)
        τz = 0.5 * (T_right - T_left)
        τ_exp = T.vec3(0.0, 0.0, τz)
        @test isapprox(ty.vel_dot[1], F_exp[1]; atol = 1e-12)
        @test isapprox(ty.vel_dot[2], F_exp[2]; atol = 1e-12)
        @test isapprox(ty.vel_dot[3], F_exp[3]; atol = 1e-12)
        @test isapprox(ty.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(ty.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(ty.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - Twin forward props roll torque from reaction torque" begin
        tr = PX4Lockstep.Tests.Fixtures.tier1_twin_forward_props_roll()
        τ_exp = T.vec3(-(0.4 + 0.6), 0.0, 0.0)
        @test isapprox(tr.ω_dot[1], τ_exp[1]; atol = 1e-12)
        @test isapprox(tr.ω_dot[2], τ_exp[2]; atol = 1e-12)
        @test isapprox(tr.ω_dot[3], τ_exp[3]; atol = 1e-12)
    end

    @testset "Phase 4 - CA axis param sign convention" begin
        ca = PX4Lockstep.Tests.Fixtures.tier1_ca_axis_param_sign()
        @test isapprox(ca.rotor0_az, -1.0; atol = 1e-12)
        @test isapprox(ca.rotor1_ay, -1.0; atol = 1e-12)
    end
end


@testset "plant_outputs purity and RHS consistency" begin
    setup = PX4Lockstep.Tests.Fixtures._iris_fullplant()
    model = setup.model
    x0 = setup.plant0

    # Non-trivial input so we exercise the bus solve + battery currents.
    motors = SVector{12,Float64}(0.4, 0.4, 0.4, 0.4, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors)
    u = Sim.Plant.PlantInput(cmd = cmd, wind_ned = Sim.Types.vec3(3.0, 0.0, 0.0))
    t = 0.0

    # Snapshot a few mutable parameter fields; plant_outputs must not mutate them.
    batt = model.batteries[1]
    batt_snap = (
        soc0 = batt.soc0,
        v1_0 = batt.v1_0,
        ocv_soc = copy(batt.ocv.soc),
        ocv_v = copy(batt.ocv.v),
        r0_model = batt.r0_model,
        r1 = batt.r1,
        c1 = batt.c1,
        min_voltage_v = batt.min_voltage_v,
        temp_c = batt.temp_c,
    )
    prop = model.propulsion
    prop_snap = copy(prop.units)

    y1 = Sim.PlantModels.plant_outputs(model, t, x0, u)
    y2 = Sim.PlantModels.plant_outputs(model, t, x0, u)
    @test y1 == y2

    @test (
        soc0 = batt.soc0,
        v1_0 = batt.v1_0,
        ocv_soc = copy(batt.ocv.soc),
        ocv_v = copy(batt.ocv.v),
        r0_model = batt.r0_model,
        r1 = batt.r1,
        c1 = batt.c1,
        min_voltage_v = batt.min_voltage_v,
        temp_c = batt.temp_c,
    ) == batt_snap
    @test prop.units == prop_snap

    # Consistency: RHS uses the same bus current in the battery SoC derivative.
    dx = model(t, x0, u)
    I_rhs = -dx.power.soc_dot[1] * batt.capacity_c
    @test all(isfinite, y1.bus_current_a)
    @test isfinite(I_rhs)
    @test isapprox(I_rhs, y1.bus_current_a[1]; rtol = 1e-12, atol = 1e-12)

    # Battery disconnected should force zero bus power and a disconnected status.
    u_off = Sim.Plant.PlantInput(
        cmd = cmd,
        wind_ned = u.wind_ned,
        faults = Sim.Faults.FaultState(battery_connected = false),
    )
    y_off = Sim.PlantModels.plant_outputs(model, t, x0, u_off)
    @test all(isapprox.(y_off.bus_current_a, 0.0; atol = 1e-12))
    @test all(isapprox.(y_off.bus_voltage_v, 0.0; atol = 1e-12))
    @test y_off.battery_statuses !== nothing
    @test y_off.battery_statuses[1].connected == false
end

@testset "Verification Tier 2 - Full-plant contract tests" begin
    @testset "Full-plant ballistic free-fall (no thrust, no wind)" begin
        b = PX4Lockstep.Tests.Fixtures.tier2_ballistic_freefall()
        @test b.count > 0
        @test b.max_z_err <= 1e-7
        @test b.max_vz_err <= 1e-7
        @test b.max_qnorm_err <= 1e-12
        @test b.min_rotor >= -1e-12
        @test b.soc_ok
        @test b.v1_ok
    end

    @testset "Hover force-balance RHS check (t=0)" begin
        h = PX4Lockstep.Tests.Fixtures.tier2_hover_force_balance()
        @test isapprox(h.vel_dot[3], 0.0; atol = 1e-6)
        @test isapprox(h.vel_dot[1], 0.0; atol = 1e-9)
        @test isapprox(h.vel_dot[2], 0.0; atol = 1e-9)
    end

    @testset "Bus solve residual sweep" begin
        bus = PX4Lockstep.Tests.Fixtures.tier2_bus_solve_residual_sweep()
        @test bus.min_ok
        @test bus.max_ok
        @test bus.max_resid <= 1e-6
        @test bus.monotonic_ok
    end

    @testset "Quad symmetry torque test" begin
        quad = PX4Lockstep.Tests.Fixtures.tier2_quad_symmetry_torque()
        @test isapprox(quad.bal[1], 0.0; atol = 1e-12)
        @test isapprox(quad.bal[2], 0.0; atol = 1e-12)
        @test isapprox(quad.bal[3], 0.0; atol = 1e-12)
        @test isapprox(quad.yaw_pos[1], 0.0; atol = 1e-12)
        @test isapprox(quad.yaw_pos[2], 0.0; atol = 1e-12)
        @test quad.yaw_pos[3] > 0.0
        @test isapprox(quad.yaw_neg[1], 0.0; atol = 1e-12)
        @test isapprox(quad.yaw_neg[2], 0.0; atol = 1e-12)
        @test quad.yaw_neg[3] < 0.0
    end

    @testset "Rotor_dir sign sensitivity" begin
        @test PX4Lockstep.Tests.Fixtures.tier2_rotor_dir_sign_sensitivity().ok
    end

    @testset "Fault semantics (motor disable / battery disconnect / estimator freeze)" begin
        f = PX4Lockstep.Tests.Fixtures.tier2_fault_semantics()
        @test f.motor_current_disabled == 0.0
        @test f.motor_current_nom > 0.0
        @test f.bus_current_disabled < f.bus_current_nom + 1e-12
        @test f.rotor_ω_dot_disabled < 0.0
        @test f.rotor_ω_dot_disabled < f.rotor_ω_dot_nom + 1e-12
        @test f.battery_connected == false
        @test isapprox(f.bus_voltage, 0.0; atol = 1e-12)
        @test isapprox(f.bus_current_disc, 0.0; atol = 1e-12)
        @test f.motor_current_all_zero
        @test f.est_freeze_ok
        @test f.est_update_ok
    end

    @testset "Engine boundary ordering probe test" begin
        @test PX4Lockstep.Tests.Fixtures.tier2_boundary_order_probe().order_ok
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