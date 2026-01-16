"""Iris record/replay defaults shared by replay/verification scripts.

This file exists to avoid "silent drift" between scripts (a common source of
confusing record/replay results). The record script, replay script, and the
one-shot integrator comparison script should all construct the **same** vehicle,
environment, battery, scenario, and estimator defaults.

Notes
-----
* These are **example defaults** (not a canonical Iris model).
* Keep this file intentionally dependency-light: it is used from `examples/*`.
* If you change parameters here, you are changing the semantics of the examples.
"""

using PX4Lockstep
using PX4Lockstep.Sim

using Random

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

"""Construct a live environment (with stateful OU wind) for PX4-in-loop runs."""
function iris_env_live(; home=DEFAULT_HOME)
    wind = Sim.Environment.OUWind(mean=WIND_MEAN, σ=WIND_SIGMA, τ_s=WIND_TAU_S)
    return Sim.Environment.EnvironmentModel(wind=wind, origin=home)
end

"""Construct a replay environment (NoWind) for plant-only replays.

Wind is provided via `PlantInput.wind_ned` from recorded traces.
"""
function iris_env_replay(; home=DEFAULT_HOME)
    return Sim.Environment.EnvironmentModel(wind=Sim.Environment.NoWind(), origin=home)
end

"""Construct the Iris vehicle instance with propulsion defaults."""
function iris_vehicle_defaults()
    model = Sim.Vehicles.IrisQuadrotor()
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned=Sim.Types.vec3(0, 0, 0),
        vel_ned=Sim.Types.vec3(0, 0, 0),
        q_bn=Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body=Sim.Types.vec3(0, 0, 0),
    )

    hover_T = model.params.mass * 9.80665 / 4.0
    propulsion = Sim.Propulsion.default_iris_quadrotor_set(
        km_m=0.05,
        thrust_hover_per_rotor_n=hover_T,
    )

    return Sim.Simulation.VehicleInstance(model, motor_act, servo_act, propulsion, x0)
end

"""Construct a pack-level Thevenin battery model."""
function iris_battery_defaults()
    return Sim.Powertrain.TheveninBattery(
        capacity_ah=BATTERY_CAPACITY_AH,
        ocv_soc=BATTERY_OCV_SOC,
        ocv_v=BATTERY_OCV_V,
        r0=BATTERY_R0,
        r1=BATTERY_R1,
        c1=BATTERY_C1,
        min_voltage_v=BATTERY_MIN_VOLTAGE_V,
    )
end

"""Construct the default mission-start scenario (arm then request mission)."""
function iris_scenario_defaults()
    scenario = Sim.Scenario.EventScenario()
    Sim.Scenario.arm_at!(scenario, ARM_TIME_S)
    Sim.Scenario.mission_start_at!(scenario, MISSION_TIME_S)
    return scenario
end

"""Construct the default discrete-time estimator (noisy + delayed)."""
function iris_estimator_defaults(dt_autopilot_s::Float64)
    base_est = Sim.Estimators.NoisyEstimator(
        pos_sigma_m=EST_POS_SIGMA_M,
        vel_sigma_mps=EST_VEL_SIGMA_MPS,
        yaw_sigma_rad=EST_YAW_SIGMA_RAD,
        rate_sigma_rad_s=EST_RATE_SIGMA_RAD_S,
        bias_tau_s=EST_BIAS_TAU_S,
        pos_bias_sigma_m=EST_POS_BIAS_SIGMA_M,
    )
    return Sim.Estimators.DelayedEstimator(
        base_est;
        delay_s=EST_DELAY_S,
        dt_est=dt_autopilot_s,
    )
end

"""Construct the coupled plant RHS functor.

The returned dynfun is suitable for both record and replay engines.
"""
function iris_dynfun(env, vehicle, battery; contact=Sim.Contacts.FlatGroundContact())
    return Sim.PlantSimulation.PlantDynamicsWithContact(
        vehicle.model,
        env,
        contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )
end

"""Reference RK45 integrator config (tight tolerances, plant-aware)."""
function iris_reference_integrator()
    return Sim.Integrators.RK45Integrator(
        rtol_pos=1e-7,
        atol_pos=1e-3,
        rtol_vel=1e-7,
        atol_vel=1e-3,
        rtol_ω=1e-7,
        atol_ω=1e-4,
        atol_att_rad=1e-4,
        plant_error_control=true,
        rtol_rotor=1e-7,
        atol_rotor=5e-2,
        rtol_soc=1e-7,
        atol_soc=1e-6,
        rtol_v1=1e-7,
        atol_v1=1e-3,
        h_min=1e-6,
        h_max=0.02,
    )
end

"""A reasonable default test integrator config by name."""
function iris_test_integrator(name::AbstractString)
    n = uppercase(strip(name))
    if n == "EULER"
        return Sim.Integrators.EulerIntegrator()
    elseif n == "RK4"
        return Sim.Integrators.RK4Integrator()
    elseif n == "RK23"
        return Sim.Integrators.RK23Integrator(
            rtol_pos=1e-5,
            atol_pos=5e-3,
            rtol_vel=1e-5,
            atol_vel=5e-3,
            rtol_ω=1e-5,
            atol_ω=5e-4,
            atol_att_rad=5e-4,
            plant_error_control=false,
            h_min=1e-6,
            h_max=0.05,
        )
    elseif n == "RK45"
        return Sim.Integrators.RK45Integrator(
            rtol_pos=1e-6,
            atol_pos=2e-3,
            rtol_vel=1e-6,
            atol_vel=2e-3,
            rtol_ω=1e-6,
            atol_ω=2e-4,
            atol_att_rad=2e-4,
            plant_error_control=false,
            h_min=1e-6,
            h_max=0.05,
        )
    else
        error("Unknown solver '$name'. Expected Euler, RK4, RK23, or RK45.")
    end
end
