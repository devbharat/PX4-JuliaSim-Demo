"""Example: PX4-in-the-loop lockstep simulation (Iris quad, PlantSimulation + RK45).

This runs:
* PX4 lockstep (Navigator/Mission + controllers + allocator)
* a coupled plant model in Julia (rigid body + actuators + rotors + battery)
* an event-driven, adaptive-step simulation loop (RK45)

Environment variables:
* `PX4_LOCKSTEP_LIB` : path to `libpx4_lockstep` (`.so`/`.dylib`)
* `PX4_LOCKSTEP_MISSION` : path to `.waypoints` file

Outputs:
* `sim_log.csv`
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
const EST_DELAY_S = 0.008

# Simulation rates (event boundaries).
const SIM_DT_AUTOPILOT = 0.004
const SIM_DT_WIND = 0.002
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

        # World: ISA atmosphere + stateful turbulence.
        wind = Sim.Environment.OUWind(mean=WIND_MEAN, σ=WIND_SIGMA, τ_s=WIND_TAU_S)
        env = Sim.Environment.EnvironmentModel(
            wind=wind,
            origin=DEFAULT_HOME,
        )

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
        vehicle = Sim.Simulation.VehicleInstance(model, motor_act, servo_act, propulsion, x0)

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

        base_est = Sim.Estimators.NoisyEstimator(
            pos_sigma_m=EST_POS_SIGMA_M,
            vel_sigma_mps=EST_VEL_SIGMA_MPS,
            yaw_sigma_rad=EST_YAW_SIGMA_RAD,
            rate_sigma_rad_s=EST_RATE_SIGMA_RAD_S,
            bias_tau_s=EST_BIAS_TAU_S,
            pos_bias_sigma_m=EST_POS_BIAS_SIGMA_M,
        )
        est = Sim.Estimators.DelayedEstimator(
            base_est;
            delay_s=EST_DELAY_S,
            dt_est=SIM_DT_AUTOPILOT,
        )

        # Adaptive RK45 with physical tolerances (units in meters, m/s, rad, etc.).
        rk45 = Sim.Integrators.RK45Integrator(
            rtol_pos=1e-6,
            atol_pos=1e-3,          # 1 mm
            rtol_vel=1e-6,
            atol_vel=1e-3,          # 1 mm/s
            rtol_ω=1e-6,
            atol_ω=1e-4,            # 0.1 mrad/s
            atol_att_rad=1e-4,      # ~0.006 deg
            plant_error_control=true,
            rtol_act=1e-5,
            atol_act=1e-4,          # actuator command units
            rtol_actdot=1e-5,
            atol_actdot=1e-3,
            rtol_rotor=1e-5,
            atol_rotor=1e-1,        # rad/s
            rtol_soc=1e-6,
            atol_soc=1e-6,
            rtol_v1=1e-5,
            atol_v1=1e-3,           # V
            h_min=1e-6,
            h_max=0.05,
        )

        sim_cfg = Sim.PlantSimulation.PlantSimulationConfig(
            t0=0.0,
            t_end=SIM_T_END,
            dt_autopilot=SIM_DT_AUTOPILOT,
            dt_wind=SIM_DT_WIND,
            dt_log=SIM_DT_LOG,
            seed=SIM_SEED,
        )

        sim = Sim.PlantSimulation.PlantSimulationInstance(
            cfg=sim_cfg,
            env=env,
            vehicle=vehicle,
            autopilot=ap,
            estimator=est,
            integrator=rk45,
            scenario=scenario,
            battery=battery,
            log=Sim.Logging.SimLog(),
            contact=Sim.Contacts.FlatGroundContact(),
        )

        Sim.PlantSimulation.run!(sim)
        Sim.Logging.write_csv(sim.log, "sim_log.csv")

        stats = Sim.Integrators.last_stats(rk45)
        @info "Done" final_time=sim.t_s log_samples=length(sim.log.t) rk45_nfev=stats.nfev
        return sim
    finally
        Sim.Autopilots.close!(ap)
    end
end

run_example()
