"""Signal bus definition (Option A).

The bus is the *explicit coupling contract* between components.

Design constraints
------------------
- **Typed**: fields have fixed meanings and units.
- **Versioned**: `BUS_SCHEMA_VERSION` must change if semantics change.
- **Deterministic sampling**: bus values are considered piecewise-constant between
  event boundaries unless otherwise documented.

The bus intentionally does not try to store everything. It stores the signals required
to:
- drive the plant (commands, wind, environment)
- feed PX4 (battery status, pose)
- support record/replay (streams aligned to time axes)

This file defines the schema and helper constructors.

NOTE
----
This is a first-pass schema. Expect to iterate.
"""

using StaticArrays

using ..Types: Vec3, vec3, Quat
using ..Vehicles: ActuatorCommand
using ..Powertrain: BatteryStatus
using ..Autopilots: AutopilotCommand
using ..Estimators: EstimatedState
using ..Faults: FaultState

# Optional: for richer bus fields later.
# using ..Plant: PlantState

"""Bump this when *field meanings or units* change."""
const BUS_SCHEMA_VERSION = 5

"""A minimal environment snapshot for bus-level coupling."""
Base.@kwdef struct EnvSample
    rho_kgm3::Float64 = 1.225
    temp_k::Float64 = 288.15
end

"""Battery telemetry as presented to PX4 and logs.

We intentionally reuse `Sim.Powertrain.BatteryStatus` to avoid duplicate schema drift.
"""
const BatteryTelemetry = BatteryStatus

"""Top-level bus state.

Fields
------
- `time_us`: current simulation time (authoritative)
- `cmd`: latest actuator command packet (ZOH between autopilot ticks)
- `wind_ned`: latest wind sample (sample-and-hold between wind ticks)
- `env`: atmosphere snapshot (typically derived from altitude deterministically)
- `battery`: battery telemetry (updated by plant at boundaries)

Future additions (likely)
------------------------
- PX4 setpoints (trajectory setpoint, thrust setpoint) for logging
- sensor injection streams
- failure/mode flags
"""
mutable struct SimBus
    schema_version::Int
    time_us::UInt64

    cmd::ActuatorCommand
    wind_ned::Vec3

    # Fault state (piecewise-constant). Scenario publishes this.
    faults::FaultState

    # High-level autopilot command + status (piecewise-constant between boundaries).
    ap_cmd::AutopilotCommand
    landed::Bool

    # Estimated state presented to the autopilot (deterministic discrete-time estimator).
    #
    # Contract: this field must be updated *before* the autopilot tick at the same `time_us`.
    # `NullEstimatorSource` publishes truth-as-estimate.
    est::EstimatedState

    env::EnvSample
    battery::BatteryTelemetry
end

function SimBus(; time_us::UInt64 = 0)
    return SimBus(
        BUS_SCHEMA_VERSION,
        time_us,
        ActuatorCommand(),
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
        BatteryTelemetry(),
    )
end

"""Reset bus to a known baseline at time `t_us`.

This is helpful for deterministic replays and tests.
"""
function reset_bus!(bus::SimBus, t_us::UInt64)
    bus.schema_version == BUS_SCHEMA_VERSION || error(
        "BUS_SCHEMA_VERSION mismatch: bus has $(bus.schema_version), expected $(BUS_SCHEMA_VERSION)",
    )
    bus.time_us = t_us
    bus.cmd = ActuatorCommand()
    bus.wind_ned = vec3(0, 0, 0)
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
    bus.battery = BatteryTelemetry()
    return nothing
end
