"""Example: PX4-in-the-loop lockstep simulation (Iris quad).

This runs:
* PX4 lockstep (Navigator/Mission + controllers + allocator)
* a minimal Iris rigid-body model in Julia
* a deterministic simulation loop with a selectable integrator (RK4 by default)

Environment variables:
* `PX4_LOCKSTEP_LIB` : path to `libpx4_lockstep` (`.so`/`.dylib`)
* `PX4_LOCKSTEP_MISSION` : path to `.waypoints` file

Outputs:
* `sim_log.csv` and (if Plots.jl is installed) `sim_plot.png`.
"""

using PX4Lockstep
using PX4Lockstep.Sim

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
# IMPORTANT: the sim currently steps the estimator at the *autopilot cadence* (`dt_autopilot`).
# `DelayedEstimator` therefore requires `dt_est == dt_autopilot` and `delay_s` to be an exact
# multiple of that dt to keep the delay behavior deterministic.
const EST_DELAY_S = 0.008
const EST_DT = 0.004

# Simulation rates.
const SIM_DT = 0.002
const SIM_DT_AUTOPILOT = 0.004
const SIM_DT_LOG = 0.01
const SIM_T_END = 90.0
const SIM_SEED = 1

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

function run_example()
    lib_path = get(ENV, PX4Lockstep.LIB_ENV, "")
    mission_path = get(ENV, "PX4_LOCKSTEP_MISSION", "")

    # PX4 side config: Commander lockstep is still stubbed, so keep it disabled.
    px4_cfg = PX4Lockstep.LockstepConfig(
        dataman_use_ram=1,
        enable_commander=0,
        enable_control_allocator=1,
    )

    ap = Sim.Autopilots.init!(
        config=px4_cfg,
        libpath=(isempty(lib_path) ? nothing : lib_path),
        home=DEFAULT_HOME,
        edge_trigger=false,
    )
    try
        if !isempty(mission_path)
            Sim.Autopilots.load_mission!(ap, mission_path)
        end

        # World: ISA atmosphere + (optionally) stateful turbulence.
        wind = Sim.Environment.OUWind(mean=WIND_MEAN, σ=WIND_SIGMA, τ_s=WIND_TAU_S)
        env = Sim.Environment.EnvironmentModel(
            wind=wind,
            origin=DEFAULT_HOME,
        )

        model = Sim.Vehicles.IrisQuadrotor()

        # Actuators: keep direct (no extra lag). Motor dynamics are handled by Propulsion.
        motor_act = Sim.Vehicles.DirectActuators()
        servo_act = Sim.Vehicles.DirectActuators()

        x0 = Sim.RigidBody.RigidBodyState(
            pos_ned=Sim.Types.vec3(0, 0, 0),
            vel_ned=Sim.Types.vec3(0, 0, 0),
            q_bn=Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body=Sim.Types.vec3(0, 0, 0),
        )
        # Propulsion (ESC + BLDC + prop): duty → ω → thrust/torque/current.
        hover_T = model.params.mass * 9.80665 / 4.0
        # `km_m` is a prop calibration convenience (torque/thrust ratio) used by
        # the default quadratic prop model.
        propulsion = Sim.Propulsion.default_iris_quadrotor_set(
            km_m=0.05,
            thrust_hover_per_rotor_n=hover_T,
        )
        vehicle = Sim.Simulation.VehicleInstance(model, motor_act, servo_act, propulsion, x0)

        integrator = Sim.Integrators.RK4Integrator()

        # Battery model: 1st-order Thevenin equivalent (OCV + R0 + RC).
        battery = Sim.Powertrain.TheveninBattery(
            capacity_ah=BATTERY_CAPACITY_AH,
            ocv_soc=BATTERY_OCV_SOC,
            ocv_v=BATTERY_OCV_V,
            r0=BATTERY_R0,
            r1=BATTERY_R1,
            c1=BATTERY_C1,
            min_voltage_v=BATTERY_MIN_VOLTAGE_V,
        )

        scenario = Sim.Scenario.EventScenario()
        Sim.Scenario.arm_at!(scenario, ARM_TIME_S)
        Sim.Scenario.mission_start_at!(scenario, MISSION_TIME_S)

        # Examples of additional event injection:
        # Sim.Scenario.wind_step_at!(scenario, 10.0, Sim.Types.vec3(2.0, 0.0, 0.0); duration_s=3.0)
        # Sim.Scenario.fail_motor_at!(scenario, 15.0, 1)  # disable motor 1

        # Estimated-state injection (noise + fixed delay).
        base_est = Sim.Estimators.NoisyEstimator(
            pos_sigma_m=EST_POS_SIGMA_M,
            vel_sigma_mps=EST_VEL_SIGMA_MPS,
            yaw_sigma_rad=EST_YAW_SIGMA_RAD,
            rate_sigma_rad_s=EST_RATE_SIGMA_RAD_S,
            bias_tau_s=EST_BIAS_TAU_S,
            pos_bias_sigma_m=EST_POS_BIAS_SIGMA_M,
        )
        est = Sim.Estimators.DelayedEstimator(base_est; delay_s=EST_DELAY_S, dt_est=EST_DT)

        sim_cfg = Sim.Simulation.SimulationConfig(
            dt=SIM_DT,
            dt_autopilot=SIM_DT_AUTOPILOT,
            dt_log=SIM_DT_LOG,
            t_end=SIM_T_END,
            seed=SIM_SEED,
        )
        sim = Sim.Simulation.SimulationInstance(
            cfg=sim_cfg,
            env=env,
            vehicle=vehicle,
            autopilot=ap,
            estimator=est,
            integrator=integrator,
            scenario=scenario,
            battery=battery,
            log=Sim.Logging.SimLog(),
            contact=Sim.Contacts.FlatGroundContact(),
        )

        Sim.Simulation.run!(sim)
        Sim.Logging.write_csv(sim.log, "sim_log.csv")

        @info "Done" final_time=sim.t log_samples=length(sim.log.t)
        return sim
    finally
        Sim.Autopilots.close!(ap)
    end
end

run_example()
