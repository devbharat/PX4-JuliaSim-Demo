@testset "PlantModels: bus voltage solve (linear + saturated regimes)" begin
    pset = Sim.Propulsion.default_multirotor_set()
    p = pset  # QuadRotorSet{4}
    ω = SVector{4,Float64}(400.0, 400.0, 400.0, 400.0)

    ocv = 12.6
    v1 = 0.0
    R0 = 0.05
    V_min = 8.0

    # Linear-ish regime: moderate duty and ω so currents are positive but not saturated.
    duty_lin = SVector{4,Float64}(0.2, 0.2, 0.2, 0.2)
    V_lin = Sim.PlantModels._solve_bus_voltage(p, ω, duty_lin, ocv, v1, R0, V_min)
    I_lin = Sim.PlantModels._bus_current_total(p, ω, duty_lin, V_lin)
    @test isfinite(V_lin)
    @test V_lin >= V_min - 1e-9
    @test V_lin <= (ocv - v1) + 1e-9
    @test V_lin > V_min + 1e-6
    @test isapprox(V_lin + R0 * I_lin, ocv - v1; atol = 1e-5)

    # Saturated regime: near-stall at full duty (forces current clamping).
    ω_stall = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0)
    duty_sat = SVector{4,Float64}(1.0, 1.0, 1.0, 1.0)
    V_sat = Sim.PlantModels._solve_bus_voltage(p, ω_stall, duty_sat, ocv, v1, R0, V_min)
    I_sat = Sim.PlantModels._bus_current_total(p, ω_stall, duty_sat, V_sat)
    @test isfinite(V_sat)
    @test V_sat >= V_min - 1e-9
    @test V_sat <= (ocv - v1) + 1e-9
    @test isapprox(V_sat, V_min; atol = 1e-6)
    res_sat = V_sat + R0 * I_sat - (ocv - v1)
    if isapprox(V_sat, V_min; atol = 1e-6)
        @test res_sat >= -1e-6
    else
        @test isapprox(res_sat, 0.0; atol = 1e-3)
    end

    # Current-limited regime (force I_lin >= Imax but avoid V_min clamp).
    p_sat = Sim.Propulsion.default_multirotor_set()
    units_sat = [
        Sim.Propulsion.MotorPropUnit(
            esc = unit.esc,
            motor = Sim.Propulsion.BLDCMotorParams(
                Kv_rpm_per_volt = unit.motor.Kv_rpm_per_volt,
                R_ohm = unit.motor.R_ohm,
                J_kgm2 = unit.motor.J_kgm2,
                I0_a = unit.motor.I0_a,
                viscous_friction_nm_per_rad_s = unit.motor.viscous_friction_nm_per_rad_s,
                max_current_a = 5.0,
            ),
            prop = unit.prop,
        ) for unit in p_sat.units
    ]
    p_sat = Sim.Propulsion.QuadRotorSet(units_sat, p_sat.rotor_dir)

    ocv_sat = 12.6
    R0_sat = 0.02
    V_min_sat = 0.0
    V_cur = Sim.PlantModels._solve_bus_voltage(p_sat, ω_stall, duty_sat, ocv_sat, v1, R0_sat, V_min_sat)
    I_cur = Sim.PlantModels._bus_current_total(p_sat, ω_stall, duty_sat, V_cur)
    @test V_cur > V_min_sat + 1e-6
    @test isapprox(V_cur + R0_sat * I_cur, ocv_sat - v1; atol = 1e-3)
end

@testset "PlantModels: bus voltage clamps to V0 when V0 < V_min" begin
    pset = Sim.Propulsion.default_multirotor_set()
    p = pset
    ω = SVector{4,Float64}(200.0, 200.0, 200.0, 200.0)

    # Force open-circuit voltage below the configured minimum.
    ocv = 9.0
    v1 = 0.0
    R0 = 0.02
    V_min = 10.5

    duty = SVector{4,Float64}(0.3, 0.3, 0.3, 0.3)
    V = Sim.PlantModels._solve_bus_voltage(p, ω, duty, ocv, v1, R0, V_min)
    @test isfinite(V)
    @test isapprox(V, ocv - v1; atol = 1e-9)
    @test V < V_min - 1e-9
end
