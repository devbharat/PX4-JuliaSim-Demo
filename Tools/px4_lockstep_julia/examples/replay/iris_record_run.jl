"""Record a Tier-0 trace for an Iris mission (PX4 in the loop).

This runs a **closed-loop** PX4 lockstep mission and records the minimal streams
needed for a **plant-only** replay/integrator sweep:

* `cmd` (actuator commands) sampled on `timeline.ap`
* `wind_ned` sampled on `timeline.wind`
* scenario outputs (`ap_cmd`, `landed`, `faults`) on `timeline.scn`
* *fault transitions* also on `timeline.evt` (so `When(...)` transitions can't be missed)
* `plant` + `battery` snapshots on `timeline.log` (useful sanity checks)

The output is saved as a `Tier0Recording` using Julia `Serialization`.

Usage
-----
```bash
PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib) \
PX4_LOCKSTEP_MISSION=examples/simple_mission.waypoints \
  julia --project -O3 examples/replay/iris_record_run.jl
```

Optional env vars
-----------------
* `IRIS_RECORD_OUT` (default: `iris_tier0_recording.jls`)
* `IRIS_T_END_S` (default: 60.0)
* `IRIS_DT_AUTOPILOT_S` (default: 0.004)
* `IRIS_DT_WIND_S` (default: 0.002)
* `IRIS_DT_LOG_S` (default: 0.01)
* `IRIS_SEED` (default: 1)
* `IRIS_RECORD_SOLVER` in {`Euler`, `RK4`, `RK23`, `RK45`} (default: `RK45`)
"""

using Random

using PX4Lockstep
using PX4Lockstep.Sim

using PX4Lockstep.Sim.RecordReplay

# Keep Iris defaults centralized.
include(joinpath(@__DIR__, "iris_common.jl"))


################################################################################
# Small env-var helpers
################################################################################

_getenv_str(name::AbstractString, default::AbstractString) = get(ENV, name, default)

function _getenv_float(name::AbstractString, default::Float64)
    v = get(ENV, name, nothing)
    v === nothing && return default
    return parse(Float64, v)
end

function _getenv_int(name::AbstractString, default::Int)
    v = get(ENV, name, nothing)
    v === nothing && return default
    return parse(Int, v)
end


################################################################################
# Integrator selector
################################################################################

function _make_record_integrator(name::AbstractString)
    n = uppercase(strip(name))
    if n == "EULER"
        return Sim.Integrators.EulerIntegrator()
    elseif n == "RK4"
        return Sim.Integrators.RK4Integrator()
    elseif n == "RK23"
        # For recording we prefer "safe" tolerances (no missing fast dynamics).
        return Sim.Integrators.RK23Integrator(
            rtol_pos = 1e-6,
            atol_pos = 1e-3,
            rtol_vel = 1e-6,
            atol_vel = 1e-3,
            rtol_ω = 1e-6,
            atol_ω = 1e-4,
            atol_att_rad = 1e-4,
            plant_error_control = true,
            rtol_rotor = 1e-6,
            atol_rotor = 5e-2,
            rtol_soc = 1e-6,
            atol_soc = 1e-6,
            rtol_v1 = 1e-6,
            atol_v1 = 1e-3,
            h_min = 1e-6,
            h_max = 0.02,
        )
    elseif n == "RK45"
        return iris_reference_integrator()
    else
        error("Unknown IRIS_RECORD_SOLVER='$(name)'. Use Euler, RK4, RK23, or RK45.")
    end
end


################################################################################
# Main
################################################################################

mission_path = get(ENV, "PX4_LOCKSTEP_MISSION", "")
isempty(mission_path) && error("Set PX4_LOCKSTEP_MISSION to a mission file (.waypoints/.plan)")

lib_path = get(ENV, PX4Lockstep.LIB_ENV, "")

out_path = _getenv_str("IRIS_RECORD_OUT", "iris_tier0_recording.jls")
t_end_s = _getenv_float("IRIS_T_END_S", 60.0)
dt_ap_s = _getenv_float("IRIS_DT_AUTOPILOT_S", 0.004)
dt_wind_s = _getenv_float("IRIS_DT_WIND_S", 0.002)
dt_log_s = _getenv_float("IRIS_DT_LOG_S", 0.01)
seed = _getenv_int("IRIS_SEED", 1)
solver_name = _getenv_str("IRIS_RECORD_SOLVER", "RK45")

println("[iris_record_run] mission='$(mission_path)'")
println("[iris_record_run] out='$(out_path)'")
println("[iris_record_run] t_end_s=$(t_end_s), dt_log_s=$(dt_log_s)")
println("[iris_record_run] dt_ap_s=$(dt_ap_s), dt_wind_s=$(dt_wind_s)")
println("[iris_record_run] seed=$(seed), solver=$(solver_name)")

# Autopilot (PX4 lockstep)
px4_cfg = PX4Lockstep.LockstepConfig(
    dataman_use_ram = 1,
    enable_commander = 0,
    enable_control_allocator = 1,
)
ap = Sim.Autopilots.init!(
    config = px4_cfg,
    libpath = (isempty(lib_path) ? nothing : lib_path),
    home = DEFAULT_HOME,
    edge_trigger = false,
)

try
    Sim.Autopilots.load_mission!(ap, mission_path)

    # Models
    env = iris_env_live()
    vehicle = iris_vehicle_defaults()
    battery = iris_battery_defaults()
    scenario = iris_scenario_defaults()
    estimator = iris_estimator_defaults(dt_ap_s)

    contact = Sim.Contacts.FlatGroundContact()

    # RHS functor
    dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

    # Initial plant state (full continuous state)
    plant0 = Sim.Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Sources
    scenario_src = RecordReplay.LiveScenarioSource(scenario; env = env, vehicle = vehicle, battery = battery)
    wind_src = RecordReplay.LiveWindSource(env.wind, Random.Xoshiro(seed), dt_wind_s)
    ap_src = RecordReplay.LiveAutopilotSource(ap)
    est_src = RecordReplay.LiveEstimatorSource(estimator, Random.Xoshiro(seed + 1), dt_ap_s)

    # Timeline
    t0_us = UInt64(0)
    t_end_us = RecordReplay.dt_to_us(t_end_s)

    timeline = RecordReplay.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = RecordReplay.dt_to_us(dt_ap_s),
        dt_wind_us = RecordReplay.dt_to_us(dt_wind_s),
        dt_log_us = RecordReplay.dt_to_us(dt_log_s),
        scenario = scenario_src,
    )

    # Integrator + recorder
    integrator = _make_record_integrator(solver_name)
    recorder = RecordReplay.InMemoryRecorder()

    sim = RecordReplay.plant_record_engine(
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integrator,
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario_src,
        estimator = est_src,
        recorder = recorder,
    )

    RecordReplay.run!(sim)

    recording = RecordReplay.Tier0Recording(
        timeline = timeline,
        plant0 = plant0,
        recorder = recorder,
        meta = Dict(
            :mission_path => mission_path,
            :seed => seed,
            :t_end_s => t_end_s,
            :dt_autopilot_s => dt_ap_s,
            :dt_wind_s => dt_wind_s,
            :dt_log_s => dt_log_s,
            :home => DEFAULT_HOME,
            :record_solver => solver_name,
        ),
    )

    RecordReplay.save_recording(out_path, recording)

    println("[iris_record_run] Saved Tier0Recording → $(out_path)")
    println(
        "[iris_record_run] Samples: cmd=$(length(get(recorder.times, :cmd, UInt64[]))), wind=$(length(get(recorder.times, :wind_ned, UInt64[]))), scn=$(length(get(recorder.times, :faults, UInt64[]))), log=$(length(get(recorder.times, :plant, UInt64[])))",
    )
finally
    Sim.Autopilots.close!(ap)
end
