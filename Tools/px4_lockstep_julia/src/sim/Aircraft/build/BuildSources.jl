"""Scenario, estimator, and source wiring helpers."""

function _build_scenario(spec::AircraftSpec)
    s = Scenario.EventScenario()
    Scenario.arm_at!(s, spec.scenario.arm_time_s)
    Scenario.mission_start_at!(s, spec.scenario.mission_time_s)
    return s
end

function _build_estimator(spec::AircraftSpec)
    est = spec.estimator
    if est.kind === :none
        return nothing
    end
    base = Estimators.NoisyEstimator(
        pos_sigma_m = est.pos_sigma_m,
        vel_sigma_mps = est.vel_sigma_mps,
        yaw_sigma_rad = est.yaw_sigma_rad,
        rate_sigma_rad_s = est.rate_sigma_rad_s,
        bias_tau_s = est.bias_tau_s,
        rate_bias_sigma_rad_s = est.rate_bias_sigma_rad_s,
    )
    delay_s = est.delay_s === nothing ? 2 * spec.timeline.dt_autopilot_s : est.delay_s
    dt_est = est.dt_est_s === nothing ? spec.timeline.dt_autopilot_s : est.dt_est_s
    return Estimators.DelayedEstimator(base; delay_s = delay_s, dt_est = dt_est)
end

function _build_live_scenario_source(spec::AircraftSpec)
    scenario_obj = _build_scenario(spec)
    return Sources.LiveScenarioSource(scenario_obj)
end

function _build_live_wind_source(spec::AircraftSpec, env)
    return Sources.LiveWindSource(
        env.wind,
        Random.Xoshiro(spec.seed),
        spec.timeline.dt_wind_s,
    )
end

function _build_live_estimator_source(spec::AircraftSpec)
    estimator_obj = _build_estimator(spec)
    return estimator_obj === nothing ? Sources.NullEstimatorSource() :
           Sources.LiveEstimatorSource(
        estimator_obj,
        Random.Xoshiro(spec.seed + 1),
        spec.timeline.dt_autopilot_s,
    )
end

function _build_live_autopilot_source(ap)
    return Sources.LiveAutopilotSource(ap)
end

function _build_live_sources(spec::AircraftSpec, env, scenario_src, ap)
    wind_src = _build_live_wind_source(spec, env)
    estimator_src = _build_live_estimator_source(spec)
    autopilot_src = _build_live_autopilot_source(ap)

    return (
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = scenario_src,
        estimator = estimator_src,
    )
end

function _build_replay_sources(traces, scn_tr)
    wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing
    scenario = Sources.ReplayScenarioSource(
        scn_tr.ap_cmd,
        scn_tr.landed,
        scn_tr.faults;
        wind_dist = wind_dist,
    )

    return (
        autopilot = Sources.ReplayAutopilotSource(traces.cmd),
        wind = Sources.ReplayWindSource(traces.wind_base_ned),
        scenario = scenario,
        estimator = Sources.NullEstimatorSource(),
    )
end
