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

        gust = Sim.Environment.GustStep(
            Sim.Environment.ConstantWind(Sim.Types.vec3(0.0, 0.0, 0.0)),
            Sim.Types.vec3(2.0, 0.0, 0.0),
            5.0,
            10.0,
        )
        env = Sim.Environment.EnvironmentModel(wind=gust)
        model = Sim.Vehicles.IrisQuadrotor()
        motor_act = Sim.Vehicles.SecondOrderActuators{12}(ωn=40.0, ζ=0.8)
        servo_act = Sim.Vehicles.DirectActuators()

        x0 = Sim.RigidBody.RigidBodyState(
            pos_ned=Sim.Types.vec3(0, 0, 0),
            vel_ned=Sim.Types.vec3(0, 0, 0),
            q_bn=Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body=Sim.Types.vec3(0, 0, 0),
        )
        vehicle = Sim.Simulation.VehicleInstance(model, motor_act, servo_act, x0)

        integrator = Sim.Integrators.RK4Integrator()

        # A lightweight battery model. Replace with your real battery model later.
        # Battery model: 1st-order Thevenin equivalent (OCV + R0 + RC).
        # Replace `current_estimator` with a higher-fidelity ESC/motor current model when available.
        battery = Sim.Powertrain.TheveninBattery(
            capacity_ah=5.0,
            current_estimator=(cmds)->begin
                cmds === nothing && return 0.0
                # crude current estimate: ~20A at full collective
                return 20.0 * (sum(cmds) / length(cmds))
            end,
        )


        scenario = Sim.Scenario.ScriptedScenario(
            arm_time_s=1.0,
            mission_time_s=2.0,
            rtl_time_s=nothing,
        )

        # Estimated-state injection (noise + fixed delay).
        base_est = Sim.Estimators.NoisyEstimator(
            pos_sigma_m=Sim.Types.vec3(0.3, 0.3, 0.5),
            vel_sigma_mps=Sim.Types.vec3(0.1, 0.1, 0.2),
            yaw_sigma_rad=deg2rad(2.0),
            rate_sigma_rad_s=Sim.Types.vec3(0.02, 0.02, 0.03),
            bias_tau_s=20.0,
            pos_bias_sigma_m=Sim.Types.vec3(0.5, 0.5, 0.5),
        )
        est = Sim.Estimators.DelayedEstimator(base_est; delay_s=0.01, dt_est=0.01)

        sim_cfg = Sim.Simulation.SimulationConfig(dt=0.002, dt_autopilot=0.01, dt_log=0.01, t_end=90.0, enable_ground_plane=true)
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
