"""PX4Lockstep.Sim.Faults

First-class *fault / failure* signals.

This module defines a **typed, versionable fault state** that can be published on
the signal bus and treated as a piecewise-constant input to both discrete and
continuous components.

Design constraints
------------------
* Pure data: `FaultState` is immutable.
* Cheap to record: uses compact bitmasks.
* Deterministic: no dictionaries, no dynamic dispatch.

Semantics
---------
Faults are interpreted as **sample-and-hold** between event boundaries.

This means:
* scenario updates can change `FaultState` at a boundary time `t_us`,
* the plant RHS consumes the held fault state for integration over
  `[t_k, t_{k+1})`.
"""
module Faults

export FaultState,
    empty_faults,
    # Motor faults
    is_motor_disabled,
    set_motor_disabled,
    disable_motor,
    enable_motor,
    # Battery faults
    set_battery_connected,
    # Sensor fault mask helpers
    SENSOR_FAULT_GYRO,
    SENSOR_FAULT_ACCEL,
    SENSOR_FAULT_MAG,
    SENSOR_FAULT_BARO,
    SENSOR_FAULT_GPS,
    SENSOR_FAULT_VISION,
    SENSOR_FAULT_EST_FREEZE,
    set_sensor_faults,
    add_sensor_faults,
    clear_sensor_faults

# Max supported motors for the disable bitmask.
const MAX_MOTORS = 32

"""Bitmask-based fault state.

Fields
------
- `motor_disable_mask`: bit i disables motor (i+1) (1-indexed).
- `battery_connected`: when false, bus voltage/current are forced to 0.
- `sensor_fault_mask`: generic sensor/estimator fault bits.

Notes
-----
This definition is intentionally compact. For richer per-motor failure models
(thrust loss fraction, stuck throttle, etc.), add separate bus signals so
schema evolution is explicit.
"""
Base.@kwdef struct FaultState
    motor_disable_mask::UInt32 = 0x00000000
    battery_connected::Bool = true
    sensor_fault_mask::UInt64 = 0x0000000000000000
end

@inline empty_faults() = FaultState()

@inline function _bit(i::Int, ::Type{UInt32})::UInt32
    (i >= 1 && i <= MAX_MOTORS) || error("motor index out of range (1..$MAX_MOTORS): $i")
    return UInt32(1) << UInt32(i - 1)
end

"""Return true if motor `i` is disabled by the fault mask."""
@inline function is_motor_disabled(f::FaultState, i::Int)::Bool
    b = _bit(i, UInt32)
    return (f.motor_disable_mask & b) != 0
end

"""Return a new fault state with motor `i` disabled/enabled."""
@inline function set_motor_disabled(f::FaultState, i::Int, disabled::Bool)::FaultState
    b = _bit(i, UInt32)
    mask = disabled ? (f.motor_disable_mask | b) : (f.motor_disable_mask & ~b)
    return FaultState(
        motor_disable_mask = mask,
        battery_connected = f.battery_connected,
        sensor_fault_mask = f.sensor_fault_mask,
    )
end

@inline disable_motor(f::FaultState, i::Int) = set_motor_disabled(f, i, true)
@inline enable_motor(f::FaultState, i::Int) = set_motor_disabled(f, i, false)

"""Return a new fault state with battery connected/disconnected."""
@inline function set_battery_connected(f::FaultState, connected::Bool)::FaultState
    return FaultState(
        motor_disable_mask = f.motor_disable_mask,
        battery_connected = connected,
        sensor_fault_mask = f.sensor_fault_mask,
    )
end

# Generic sensor/estimator fault bits.
# These are deliberately coarse and can be refined per component later.
const SENSOR_FAULT_GYRO = UInt64(1) << 0
const SENSOR_FAULT_ACCEL = UInt64(1) << 1
const SENSOR_FAULT_MAG = UInt64(1) << 2
const SENSOR_FAULT_BARO = UInt64(1) << 3
const SENSOR_FAULT_GPS = UInt64(1) << 4
const SENSOR_FAULT_VISION = UInt64(1) << 5

# A sim-level fault that freezes the estimator output (truth-as-estimate no longer used).
const SENSOR_FAULT_EST_FREEZE = UInt64(1) << 16

@inline function set_sensor_faults(f::FaultState, mask::UInt64)::FaultState
    return FaultState(
        motor_disable_mask = f.motor_disable_mask,
        battery_connected = f.battery_connected,
        sensor_fault_mask = mask,
    )
end

@inline function add_sensor_faults(f::FaultState, mask::UInt64)::FaultState
    return set_sensor_faults(f, f.sensor_fault_mask | mask)
end

@inline function clear_sensor_faults(f::FaultState, mask::UInt64)::FaultState
    return set_sensor_faults(f, f.sensor_fault_mask & ~mask)
end

end # module Faults
