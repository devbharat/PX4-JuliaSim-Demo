#!/usr/bin/env julia
"""
Reference-compare: Iris mission PlantSimulation (PX4 lockstep, closed-loop).

This script compares integrators on the Iris + PX4-in-the-loop use case without
requiring analytic solutions.

It runs the same mission twice:

1) **Reference run**: RK45 with tight tolerances (plant-aware error control ON).
2) **Test run**: chosen solver (RK4 / RK23 / RK45 with looser tolerances, etc).

Then it compares the logged trajectories on the *logging grid* (`dt_log`):

* rigid-body truth: position, velocity, attitude angle error, body-rate error
* rotor omega (from log): rotor speed error norm (rad/s)
* battery (from log): |ΔV|, |ΔI|

Notes / limitations
-------------------
* This is a **closed-loop** comparison: different integrators can cause the controller
  to make different decisions (especially over long missions). That's expected.
  Think of this as a "system-level sensitivity to solver choice", not a pure plant-only
  truncation error study.

* A plant-only integrator comparison can be built by recording a baseline actuator
  trace and replaying it via a command-replay autopilot. That isolates solver error
  without closed-loop divergence.

Environment variables
---------------------
Required:
* PX4_LOCKSTEP_MISSION : path to `.waypoints`

Optional:
* PX4_LOCKSTEP_LIB     : path to `libpx4_lockstep` (.so/.dylib). If unset, the script
  falls back to the default build tree search logic.

Optional (defaults shown):
* IRIS_T_END_S       = 20.0
* IRIS_DT_AUTOPILOT  = 0.004
* IRIS_DT_WIND       = 0.002
* IRIS_DT_LOG        = 0.01
* IRIS_SEED          = 1
* IRIS_TEST_SOLVER   = "RK23"   ("RK4", "RK23", "RK45")

Outputs
-------
Writes next to the script:
* `iris_sim_log_ref.csv`
* `iris_sim_log_test.csv`
* `iris_plantsim_reference_compare.csv`  (error vs time)

Run:
    julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/reference_compare_iris_mission_plantsim.jl
"""

using PX4Lockstep
using PX4Lockstep.Sim
using LinearAlgebra
using Printf
using StaticArrays

const V = Sim.Verification

const DEFAULT_HOME = Sim.Autopilots.HomeLocation(lat_deg=47.397742, lon_deg=8.545594, alt_msl_m=488.0)

# Scenario timing.
const ARM_TIME_S = 1.0
const MISSION_TIME_S = 2.0

# Wind configuration.
const WIND_MEAN = Sim.Types.vec3(0.0, 0.0, 0.0)
const WIND_SIGMA = Sim.Types.vec3(0.5, 0.5, 0.2)
const WIND_TAU_S = 3.0

# Estimator noise/delay.
const EST_POS_SIGMA_M = Sim.Types.vec3(0.3, 0.3, 0.5)
const EST_VEL_SIGMA_MPS = Sim.Types.vec3(0.1, 0.1, 0.2)
const EST_YAW_SIGMA_RAD = deg2rad(2.0)
const EST_RATE_SIGMA_RAD_S = Sim.Types.vec3(0.02, 0.02, 0.03)
const EST_BIAS_TAU_S = 20.0
const EST_POS_BIAS_SIGMA_M = Sim.Types.vec3(0.5, 0.5, 0.5)
const EST_DELAY_S = 0.008

# Battery configuration (pack-level values derived from per-cell parameters).
const BATTERY_CELLS = 3
const BATTERY_CAPACITY_AH = 5.0
const BATTERY_OCV_SOC = [0.0, 0.1, 0.5, 0.9, 1.0]
const BATTERY_OCV_CELL_V = [3.0, 3.6, 3.8, 4.1, 4.2]
const BATTERY_R0 = 0.02
const BATTERY_R1 = 0.01
const BATTERY_C1 = 2000.0
const BATTERY_MIN_CELL_V = 3.0
const BATTERY_OCV_V = BATTERY_OCV_CELL_V .* BATTERY_CELLS
const BATTERY_MIN_VOLTAGE_V = BATTERY_MIN_CELL_V * BATTERY_CELLS

function _env_defaults()
    wind = Sim.Environment.OUWind(mean=WIND_MEAN, σ=WIND_SIGMA, τ_s=WIND_TAU_S)
    return Sim.Environment.EnvironmentModel(
        wind = wind,
        origin = DEFAULT_HOME,
    )
end

function _vehicle_defaults()
    model = Sim.Vehicles.IrisQuadrotor()
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0, 0, 0),
        vel_ned = Sim.Types.vec3(0, 0, 0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0, 0, 0),
    )

    hover_T = model.params.mass * 9.80665 / 4.0
    propulsion = Sim.Propulsion.default_iris_quadrotor_set(
        km_m = 0.05,
        thrust_hover_per_rotor_n = hover_T,
    )
    return Sim.Simulation.VehicleInstance(model, motor_act, servo_act, propulsion, x0)
end

function _battery_defaults()
    return Sim.Powertrain.TheveninBattery(
        capacity_ah = BATTERY_CAPACITY_AH,
        ocv_soc = BATTERY_OCV_SOC,
        ocv_v = BATTERY_OCV_V,
        r0 = BATTERY_R0,
        r1 = BATTERY_R1,
        c1 = BATTERY_C1,
        min_voltage_v = BATTERY_MIN_VOLTAGE_V,
    )
end

function _scenario_defaults()
    scenario = Sim.Scenario.EventScenario()
    Sim.Scenario.arm_at!(scenario, ARM_TIME_S)
    Sim.Scenario.mission_start_at!(scenario, MISSION_TIME_S)
    return scenario
end

function _estimator_defaults(dt_autopilot::Float64)
    base_est = Sim.Estimators.NoisyEstimator(
        pos_sigma_m = EST_POS_SIGMA_M,
        vel_sigma_mps = EST_VEL_SIGMA_MPS,
        yaw_sigma_rad = EST_YAW_SIGMA_RAD,
        rate_sigma_rad_s = EST_RATE_SIGMA_RAD_S,
        bias_tau_s = EST_BIAS_TAU_S,
        pos_bias_sigma_m = EST_POS_BIAS_SIGMA_M,
    )
    return Sim.Estimators.DelayedEstimator(
        base_est;
        delay_s = EST_DELAY_S,
        dt_est = dt_autopilot,
    )
end

function _reference_integrator()
    # Tight tolerances, plant-aware.
    return Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-7,
        atol_pos = 1e-3,      # 1 mm
        rtol_vel = 1e-7,
        atol_vel = 1e-3,      # 1 mm/s
        rtol_ω = 1e-7,
        atol_ω = 1e-4,
        atol_att_rad = 1e-4,
        plant_error_control = true,
        rtol_rotor = 1e-7,
        atol_rotor = 5e-2,    # rad/s
        rtol_soc = 1e-7,
        atol_soc = 1e-6,
        rtol_v1 = 1e-7,
        atol_v1 = 1e-3,
        h_min = 1e-6,
        h_max = 0.02,
    )
end

function _test_integrator(name::AbstractString)
    n = uppercase(strip(name))
    if n == "RK4"
        return Sim.Integrators.RK4Integrator()
    elseif n == "RK23"
        return Sim.Integrators.RK23Integrator(
            rtol_pos = 1e-5,
            atol_pos = 5e-3,
            rtol_vel = 1e-5,
            atol_vel = 5e-3,
            rtol_ω = 1e-5,
            atol_ω = 5e-4,
            atol_att_rad = 5e-4,
            plant_error_control = false,
            h_min = 1e-6,
            h_max = 0.05,
        )
    elseif n == "RK45"
        return Sim.Integrators.RK45Integrator(
            rtol_pos = 1e-5,
            atol_pos = 5e-3,
            rtol_vel = 1e-5,
            atol_vel = 5e-3,
            rtol_ω = 1e-5,
            atol_ω = 5e-4,
            atol_att_rad = 5e-4,
            plant_error_control = false,
            h_min = 1e-6,
            h_max = 0.05,
        )
    else
        error("Unknown IRIS_TEST_SOLVER=$(name). Use RK4, RK23, or RK45.")
    end
end

function _run_one(; integrator, t_end::Float64, dt_autopilot::Float64, dt_wind::Float64, dt_log::Float64, seed::Int, out_csv::AbstractString)
    lib_path = get(ENV, PX4Lockstep.LIB_ENV, "")
    mission_path = get(ENV, "PX4_LOCKSTEP_MISSION", "")
    isempty(mission_path) && error("PX4_LOCKSTEP_MISSION is required")

    libpath_arg = isempty(lib_path) ? nothing : lib_path
    if libpath_arg !== nothing && !isfile(libpath_arg)
        error("PX4_LOCKSTEP_LIB path does not exist: $(libpath_arg)")
    end

    px4_cfg = PX4Lockstep.LockstepConfig(
        dataman_use_ram = 1,
        enable_commander = 0,
        enable_control_allocator = 1,
    )

    ap = Sim.Autopilots.init!(
        config = px4_cfg,
        libpath = libpath_arg,
        home = DEFAULT_HOME,
        edge_trigger = false,
    )

    try
        Sim.Autopilots.load_mission!(ap, mission_path)

        env = _env_defaults()
        vehicle = _vehicle_defaults()
        battery = _battery_defaults()
        scenario = _scenario_defaults()
        est = _estimator_defaults(dt_autopilot)

        sim_cfg = Sim.PlantSimulation.PlantSimulationConfig(
            t0 = 0.0,
            t_end = t_end,
            dt_autopilot = dt_autopilot,
            dt_wind = dt_wind,
            dt_log = dt_log,
            seed = seed,
        )

        sim = Sim.PlantSimulation.PlantSimulationInstance(
            cfg = sim_cfg,
            env = env,
            vehicle = vehicle,
            autopilot = ap,
            estimator = est,
            integrator = integrator,
            scenario = scenario,
            battery = battery,
            log = Sim.Logging.SimLog(),
            contact = Sim.Contacts.NoContact(),  # focus on flight
        )

        Sim.PlantSimulation.run!(sim)
        Sim.Logging.write_csv(sim.log, out_csv)

        return sim.log
    finally
        Sim.Autopilots.close!(ap)
    end
end

@inline function _sv3(t::NTuple{3,Float64})
    return Sim.Types.vec3(t[1], t[2], t[3])
end

@inline function _quat(t::NTuple{4,Float64})
    return Sim.Types.Quat(t[1], t[2], t[3], t[4])
end

function _traj_from_log_rb(log::Sim.Logging.SimLog)
    n = length(log.time_us)
    @assert n >= 2
    dt_us = log.time_us[2] - log.time_us[1]
    for i = 3:n
        @assert log.time_us[i] - log.time_us[i-1] == dt_us "non-uniform dt_log detected"
    end
    dt = Float64(dt_us) * 1e-6

    xs = Vector{Sim.RigidBody.RigidBodyState}(undef, n)
    for i = 1:n
        xs[i] = Sim.RigidBody.RigidBodyState(
            pos_ned = _sv3(log.pos_ned[i]),
            vel_ned = _sv3(log.vel_ned[i]),
            q_bn = _quat(log.q_bn[i]),
            ω_body = _sv3(log.ω_body[i]),
        )
    end
    return V.Trajectory{Sim.RigidBody.RigidBodyState}(dt, xs)
end

function _rotor_err_series(log_ref::Sim.Logging.SimLog, log_sol::Sim.Logging.SimLog)
    n = length(log_ref.time_us)
    @assert length(log_sol.time_us) == n
    err = Vector{Float64}(undef, n)
    for i = 1:n
        ωr = SVector{4,Float64}(log_ref.rotor_omega_rad_s[i])
        ωs = SVector{4,Float64}(log_sol.rotor_omega_rad_s[i])
        err[i] = norm(ωs - ωr)
    end
    return err
end

function main()
    t_end = parse(Float64, get(ENV, "IRIS_T_END_S", "20.0"))
    dt_autopilot = parse(Float64, get(ENV, "IRIS_DT_AUTOPILOT", "0.004"))
    dt_wind = parse(Float64, get(ENV, "IRIS_DT_WIND", "0.002"))
    dt_log = parse(Float64, get(ENV, "IRIS_DT_LOG", "0.01"))
    seed = parse(Int, get(ENV, "IRIS_SEED", "1"))
    solver_name = get(ENV, "IRIS_TEST_SOLVER", "RK23")

    ref_integ = _reference_integrator()
    test_integ = _test_integrator(solver_name)

    @info "Running reference (RK45 tight)" t_end=t_end dt_autopilot=dt_autopilot dt_wind=dt_wind dt_log=dt_log seed=seed
    log_ref = _run_one(
        integrator = ref_integ,
        t_end = t_end,
        dt_autopilot = dt_autopilot,
        dt_wind = dt_wind,
        dt_log = dt_log,
        seed = seed,
        out_csv = joinpath(@__DIR__, "iris_sim_log_ref.csv"),
    )

    @info "Running test" solver=solver_name
    log_sol = _run_one(
        integrator = test_integ,
        t_end = t_end,
        dt_autopilot = dt_autopilot,
        dt_wind = dt_wind,
        dt_log = dt_log,
        seed = seed,
        out_csv = joinpath(@__DIR__, "iris_sim_log_test.csv"),
    )

    tr_ref = _traj_from_log_rb(log_ref)
    tr_sol = _traj_from_log_rb(log_sol)

    cmp = V.compare_to_reference(tr_ref, tr_sol)

    rotor_err = _rotor_err_series(log_ref, log_sol)

    V_err = abs.(log_sol.batt_voltage_v .- log_ref.batt_voltage_v)
    I_err = abs.(log_sol.batt_current_a .- log_ref.batt_current_a)

    println()
    println("Iris PlantSimulation integrator compare (closed-loop)")
    println("  reference: RK45 tight, plant-aware")
    println("  test:      $(solver_name)")
    println("  t_end = $(t_end) s, dt_log = $(tr_ref.dt) s, samples = $(length(tr_ref.x))")
    println()
    println("Max error vs reference:")
    @printf("  pos max        = %.3e m\n", cmp.err.max.pos)
    @printf("  vel max        = %.3e m/s\n", cmp.err.max.vel)
    @printf("  att max        = %.3e rad\n", cmp.err.max.att_rad)
    @printf("  bodyrate max   = %.3e rad/s\n", cmp.err.max.ω)
    @printf("  rotor omega max = %.3e rad/s\n", maximum(rotor_err))
    @printf("  batt V max     = %.3e V\n", maximum(V_err))
    @printf("  batt I max     = %.3e A\n", maximum(I_err))

    # Write error CSV.
    out_csv = joinpath(@__DIR__, "iris_plantsim_reference_compare.csv")
    open(out_csv, "w") do io
        println(io, "t,pos_err,vel_err,att_err_rad,omega_err,rotor_omega_err,batt_voltage_err,batt_current_err")
        t = cmp.err.t
        for i in eachindex(t)
            println(io, "$(t[i]),$(cmp.err.pos_err[i]),$(cmp.err.vel_err[i]),$(cmp.err.att_err_rad[i]),$(cmp.err.ω_err[i]),$(rotor_err[i]),$(V_err[i]),$(I_err[i])")
        end
    end
    println()
    println("Wrote comparison CSV: $(out_csv)")
end

main()
