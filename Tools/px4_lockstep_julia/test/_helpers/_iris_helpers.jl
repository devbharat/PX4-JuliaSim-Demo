const IRIS_SPEC_PATH = PX4Lockstep.Workflows.spec_path(:iris_default)

function iris_spec_for_tests(; strict::Bool = true)
    return Sim.Aircraft.load_spec(IRIS_SPEC_PATH; strict = strict)
end

function iris_home_for_tests()
    return iris_spec_for_tests().home
end

function iris_contact_for_tests()
    return iris_spec_for_tests().plant.contact
end

function iris_env_live_for_tests(; home = nothing)
    spec = iris_spec_for_tests()
    env = Sim.Aircraft._build_env_live(spec)
    if home === nothing
        return env
    end
    return Sim.Environment.EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = env.wind,
        gravity = env.gravity,
        origin = home,
    )
end

function iris_env_replay_for_tests(; home = nothing)
    spec = iris_spec_for_tests()
    env = Sim.Aircraft._build_env_replay(spec)
    if home === nothing
        return env
    end
    return Sim.Environment.EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = env.wind,
        gravity = env.gravity,
        origin = home,
    )
end

function iris_vehicle_for_tests(; x0_override = nothing, x0 = nothing)
    spec = iris_spec_for_tests()
    x0_use = x0 === nothing ? x0_override : x0
    return Sim.Aircraft._build_vehicle(spec; x0_override = x0_use)
end

function iris_batteries_for_tests()
    spec = iris_spec_for_tests()
    return Sim.Aircraft._build_batteries(spec)
end

function iris_battery_for_tests()
    return iris_batteries_for_tests()[1]
end

function iris_dynfun_for_tests(env, vehicle::Sim.Vehicles.VehicleInstance, battery; contact = Sim.Contacts.NoContact())
    return Sim.PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )
end

function iris_scenario_for_tests(; arm_time_s::Float64 = 1.0, mission_time_s::Float64 = 2.0)
    s = Sim.Scenario.EventScenario()
    Sim.Scenario.arm_at!(s, arm_time_s)
    Sim.Scenario.mission_start_at!(s, mission_time_s)
    return s
end

function iris_timeline_for_tests(; t_end_s = nothing, dt_autopilot_s = nothing, dt_wind_s = nothing,
    dt_log_s = nothing, dt_phys_s = nothing, scenario_source = nothing)
    spec = iris_spec_for_tests()
    t_end_s = t_end_s === nothing ? spec.timeline.t_end_s : t_end_s
    dt_autopilot_s = dt_autopilot_s === nothing ? spec.timeline.dt_autopilot_s : dt_autopilot_s
    dt_wind_s = dt_wind_s === nothing ? spec.timeline.dt_wind_s : dt_wind_s
    dt_log_s = dt_log_s === nothing ? spec.timeline.dt_log_s : dt_log_s
    dt_phys_s = dt_phys_s === nothing ? spec.timeline.dt_phys_s : dt_phys_s

    t0_us = UInt64(0)
    t_end_us = Sim.Runtime.dt_to_us(t_end_s)
    dt_ap_us = Sim.Runtime.dt_to_us(dt_autopilot_s)
    dt_wind_us = Sim.Runtime.dt_to_us(dt_wind_s)
    dt_log_us = Sim.Runtime.dt_to_us(dt_log_s)
    dt_phys_us = dt_phys_s === nothing ? nothing : Sim.Runtime.dt_to_us(dt_phys_s)

    return Sim.Runtime.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scenario = scenario_source,
    )
end

function integrator_from_symbol(name::Symbol)
    if name === :Euler
        return Sim.Integrators.EulerIntegrator()
    elseif name === :RK4
        return Sim.Integrators.RK4Integrator()
    elseif name === :RK23
        return Sim.Integrators.RK23Integrator()
    elseif name === :RK45
        return Sim.Integrators.RK45Integrator()
    else
        error("Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)")
    end
end
