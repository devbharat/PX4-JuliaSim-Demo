"""Runtime.Bus

Signal bus schema for the canonical `Sim.Runtime.Engine`.

The bus is the explicit coupling contract between discrete-time sources
(scenario / wind / estimator / autopilot) and the continuous-time plant.

Constraints
-----------
- **Typed**: fixed meaning + units per field.
- **Versioned**: bump `BUS_SCHEMA_VERSION` if field semantics/units change.
- **Deterministic sampling**: values are treated as piecewise-constant between
  event boundaries unless documented otherwise.

This is intentionally minimal: it contains the signals required to:
- drive the plant (actuator commands, wind, faults)
- feed PX4 (battery status, estimated state)
- record/replay (streams aligned to axes)
"""

using ..Types: Vec3, vec3, Quat
using ..Vehicles: ActuatorCommand
using ..Powertrain: BatteryStatus
using ..Autopilots: AutopilotCommand
using ..Estimators: EstimatedState
using ..Faults: FaultState

"""Bump this when *field meanings or units* change."""
const BUS_SCHEMA_VERSION = 12

"""A minimal atmosphere snapshot for bus-level coupling."""
Base.@kwdef struct EnvSample
    rho_kgm3::Float64 = 1.225
    temp_k::Float64 = 288.15
end

"""Battery telemetry as presented to PX4 and logs."""
const BatteryTelemetry = BatteryStatus

"""Top-level bus state.

Fields
------
- `time_us`: current simulation time (authoritative integer microseconds)
- `cmd`: latest actuator command packet (ZOH between autopilot ticks)
- `wind_ned`: latest wind sample (sample-and-hold between wind ticks)
- `wind_dist_ned`: additive wind disturbance requested by scenario (ZOH between boundaries)
- `faults`: bus-level fault state (scenario publishes; plant consumes)
- `ap_cmd`: high-level autopilot request (arm/mission/RTL)
- `landed`: landed flag for PX4 (scenario/system-level)
- `landed_phy`: physics-derived landed flag (diagnostics/logging)
- `accel_ned`: inertial acceleration sample (NED, m/s^2) at the current boundary
- `spec_force_body`: accelerometer-like specific force sample (body/FRD, m/s^2)
- `impact_dv_ned`: max impact Δv (NED, m/s) since the last log sample
- `impact_accel_est_ned`: estimated impact acceleration (NED, m/s^2) from `impact_dv_ned / dt_autopilot`
- `impact_time_us`: timestamp (us) of the max impact Δv
- `impact_count`: number of impacts since the last log sample
- `est`: estimated state presented to autopilot (truth or injected estimator)
- `env`: minimal atmosphere sample (optional)
- `batteries`: vector of battery telemetry (fixed length after build)
"""
mutable struct SimBus
    schema_version::Int
    time_us::UInt64

    cmd::ActuatorCommand
    wind_ned::Vec3
    wind_dist_ned::Vec3

    # Fault state (piecewise-constant). Scenario publishes this.
    faults::FaultState

    # High-level autopilot command + status (piecewise-constant between boundaries).
    ap_cmd::AutopilotCommand
    landed::Bool

    # Physics-derived landed flag. This does not override `landed` yet.
    landed_phy::Bool

    # Acceleration telemetry (for logging / future sensor synthesis).
    accel_ned::Vec3
    spec_force_body::Vec3

    # Impact telemetry (latched until the next log boundary consumes it).
    impact_dv_ned::Vec3
    impact_accel_est_ned::Vec3
    impact_time_us::UInt64
    impact_count::UInt32

    # Estimated state presented to the autopilot.
    # Contract: updated *before* the autopilot tick at the same `time_us`.
    est::EstimatedState

    env::EnvSample

    # Multi-battery telemetry (battery 1 is the primary).
    batteries::Vector{BatteryTelemetry}
end

"""Construct a `SimBus`.

Parameters
----------
- `time_us`: start time
- `n_batteries`: number of battery telemetry slots to allocate (fixed-length after build)
"""
function SimBus(; time_us::UInt64 = 0, n_batteries::Integer = 1)
    nb = Int(n_batteries)
    nb >= 1 || error("SimBus(n_batteries=$nb) must be >= 1")
    bats = [BatteryTelemetry() for _ = 1:nb]

    return SimBus(
        BUS_SCHEMA_VERSION,
        time_us,
        ActuatorCommand(),
        vec3(0, 0, 0),
        vec3(0, 0, 0),
        FaultState(),
        AutopilotCommand(),
        true,
        true,
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 0.0),
        UInt64(0),
        UInt32(0),
        EstimatedState(
            pos_ned = vec3(0.0, 0.0, 0.0),
            vel_ned = vec3(0.0, 0.0, 0.0),
            q_bn = Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = vec3(0.0, 0.0, 0.0),
        ),
        EnvSample(),
        bats,
    )
end

"""Reset bus to a known baseline at time `t_us`.

This is useful for deterministic replays and tests.
"""
function reset_bus!(bus::SimBus, t_us::UInt64)
    bus.schema_version == BUS_SCHEMA_VERSION || error(
        "BUS_SCHEMA_VERSION mismatch: bus has $(bus.schema_version), expected $(BUS_SCHEMA_VERSION)",
    )

    bus.time_us = t_us
    bus.cmd = ActuatorCommand()
    bus.wind_ned = vec3(0, 0, 0)
    bus.wind_dist_ned = vec3(0, 0, 0)
    bus.faults = FaultState()
    bus.ap_cmd = AutopilotCommand()
    bus.landed = true
    bus.landed_phy = true

    bus.accel_ned = vec3(0.0, 0.0, 0.0)
    bus.spec_force_body = vec3(0.0, 0.0, 0.0)
    bus.impact_dv_ned = vec3(0.0, 0.0, 0.0)
    bus.impact_accel_est_ned = vec3(0.0, 0.0, 0.0)
    bus.impact_time_us = UInt64(0)
    bus.impact_count = UInt32(0)
    bus.est = EstimatedState(
        pos_ned = vec3(0.0, 0.0, 0.0),
        vel_ned = vec3(0.0, 0.0, 0.0),
        q_bn = Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = vec3(0.0, 0.0, 0.0),
    )
    bus.env = EnvSample()

    # Reset batteries in-place (fixed length).
    bats = getfield(bus, :batteries)
    for i in eachindex(bats)
        bats[i] = BatteryTelemetry()
    end
    return nothing
end

export SimBus, BUS_SCHEMA_VERSION, reset_bus!, EnvSample
