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
const BUS_SCHEMA_VERSION = 9

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
- `landed`: landed flag for PX4
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

    # Estimated state presented to the autopilot.
    # Contract: updated *before* the autopilot tick at the same `time_us`.
    est::EstimatedState

    env::EnvSample

    # Phase 5.3: multi-battery telemetry (battery 1 is the primary).
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
    bats = [BatteryTelemetry() for _ in 1:nb]

    return SimBus(
        BUS_SCHEMA_VERSION,
        time_us,
        ActuatorCommand(),
        vec3(0, 0, 0),
        vec3(0, 0, 0),
        FaultState(),
        AutopilotCommand(),
        true,
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
