"""PX4Lockstep.Sim.Powertrain

Battery / powertrain models.

You said you'll own battery models on the Julia side and inject `battery_status` into PX4.
This module provides a small interface so you can swap models without touching:
- the sim engine
- the PX4 lockstep bridge

Included models:

- `IdealBattery`: deterministic coulomb counting + constant voltage (baseline)
- `TheveninBattery`: 1st-order Thevenin equivalent (OCV + R0 + RC polarization)

The Thevenin model is still intentionally lightweight (no thermal yet), but it is a much
better starting point for battery-related RTL / energy studies than a constant voltage.
"""
module Powertrain

using Printf

export BatteryStatus, AbstractBatteryModel, IdealBattery, TheveninBattery, step!, status

############################
# Types
############################

"""Battery status injected into PX4.

Fields match what the lockstep C ABI expects.
"""
Base.@kwdef struct BatteryStatus
    connected::Bool = true
    voltage_v::Float64 = 12.0
    current_a::Float64 = 0.0
    remaining::Float64 = 1.0   # [0,1]
    warning::Int32 = 0         # PX4 battery_status warning enum
end

abstract type AbstractBatteryModel end

############################
# Helpers
############################

@inline function _warning_from_remaining(
    rem::Float64,
    low::Float64,
    crit::Float64,
    emerg::Float64,
)::Int32
    if rem <= emerg
        return Int32(3)  # EMERGENCY
    elseif rem <= crit
        return Int32(2)  # CRITICAL
    elseif rem <= low
        return Int32(1)  # LOW
    else
        return Int32(0)  # NONE
    end
end

@inline function _clamp01(x::Float64)
    x < 0.0 && return 0.0
    x > 1.0 && return 1.0
    return x
end

############################
# Ideal battery (baseline)
############################

"""Ideal coulomb-counted battery model.

- SOC integrates from a fixed capacity.
- Voltage is constant (or can be set externally).
- Current draw is estimated from motor commands via a user-provided callback.

This is intentionally simple and deterministic.
"""
mutable struct IdealBattery <: AbstractBatteryModel
    capacity_c::Float64          # coulombs
    soc::Float64
    voltage_v::Float64
    last_current_a::Float64
    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
    current_estimator::Function
end

function IdealBattery(;
    capacity_ah::Float64 = 5.0,
    soc0::Float64 = 1.0,
    voltage_v::Float64 = 12.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
    current_estimator::Function = (cmds)->0.0,
)
    capacity_c = capacity_ah * 3600.0
    return IdealBattery(
        capacity_c,
        _clamp01(soc0),
        voltage_v,
        0.0,
        low_thr,
        crit_thr,
        emerg_thr,
        current_estimator,
    )
end

"""Advance the battery model by `dt` seconds using a bus current draw.

This is the preferred stepping API once you have an explicit motor/ESC model.
"""
function step!(b::IdealBattery, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    b.last_current_a = I
    b.soc = _clamp01(b.soc - (I * dt) / b.capacity_c)
    return nothing
end

"""Advance the battery model by `dt` seconds using motor commands.

This legacy helper is kept for convenience when you don't have a motor model yet.
"""
function step!(b::IdealBattery, motor_cmds, dt::Float64)
    I = Float64(b.current_estimator(motor_cmds))
    return step!(b, I, dt)
end

function status(b::IdealBattery)::BatteryStatus
    return BatteryStatus(
        connected = true,
        voltage_v = b.voltage_v,
        current_a = b.last_current_a,
        remaining = b.soc,
        warning = _warning_from_remaining(b.soc, b.low_thr, b.crit_thr, b.emerg_thr),
    )
end

############################
# Thevenin battery (OCV + R0 + RC)
############################

"""Piecewise-linear OCV curve for battery open-circuit voltage.

Given two equal-length vectors `(soc_pts, v_pts)`, returns `v(soc)` via clamped linear
interpolation.
"""
function _interp_ocv(soc_pts::Vector{Float64}, v_pts::Vector{Float64}, soc::Float64)
    n = length(soc_pts)
    n == length(v_pts) || throw(ArgumentError("ocv_soc and ocv_v must have same length"))
    n >= 2 || throw(ArgumentError("ocv curve must have at least 2 points"))

    s = _clamp01(soc)

    # Clamp to end points.
    if s <= soc_pts[1]
        return v_pts[1]
    elseif s >= soc_pts[end]
        return v_pts[end]
    end

    # Find segment (linear scan; small n).
    for i = 1:(n-1)
        s0 = soc_pts[i]
        s1 = soc_pts[i+1]
        if s >= s0 && s <= s1
            α = (s - s0) / (s1 - s0)
            return (1-α)*v_pts[i] + α*v_pts[i+1]
        end
    end

    # Fallback (shouldn't hit if curve monotonic in soc).
    return v_pts[end]
end

"""1st-order Thevenin equivalent battery model.

Terminal voltage:
  V = OCV(SOC) - I*R0 - V1

Polarization voltage state:
  dV1/dt = -(1/(R1*C1)) * V1 + (1/C1) * I

SOC:
  dSOC/dt = -I / Q
"""
mutable struct TheveninBattery <: AbstractBatteryModel
    capacity_c::Float64
    soc::Float64

    ocv_soc::Vector{Float64}
    ocv_v::Vector{Float64}

    r0::Float64
    r1::Float64
    c1::Float64
    v1::Float64

    last_current_a::Float64

    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64

    current_estimator::Function
end

function TheveninBattery(;
    capacity_ah::Float64 = 5.0,
    soc0::Float64 = 1.0,
    # Default ~3S LiPo-ish open circuit curve (very rough).
    ocv_soc::Vector{Float64} = [0.0, 0.1, 0.5, 0.9, 1.0],
    ocv_v::Vector{Float64} = [9.0, 10.8, 11.4, 12.3, 12.6],
    r0::Float64 = 0.02,
    r1::Float64 = 0.01,
    c1::Float64 = 2000.0,
    v1_0::Float64 = 0.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
    current_estimator::Function = (cmds)->0.0,
)
    capacity_c = capacity_ah * 3600.0
    return TheveninBattery(
        capacity_c,
        _clamp01(soc0),
        ocv_soc,
        ocv_v,
        r0,
        r1,
        c1,
        v1_0,
        0.0,
        low_thr,
        crit_thr,
        emerg_thr,
        current_estimator,
    )
end

function step!(b::TheveninBattery, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    b.last_current_a = I

    # SOC coulomb counting.
    b.soc = _clamp01(b.soc - (I * dt) / b.capacity_c)

    # Polarization voltage dynamics.
    if b.r1 > 0.0 && b.c1 > 0.0
        τ = b.r1 * b.c1
        b.v1 += dt * (-(b.v1 / τ) + (I / b.c1))
    end

    return nothing
end

function step!(b::TheveninBattery, motor_cmds, dt::Float64)
    I = Float64(b.current_estimator(motor_cmds))
    return step!(b, I, dt)
end

function status(b::TheveninBattery)::BatteryStatus
    ocv = _interp_ocv(b.ocv_soc, b.ocv_v, b.soc)
    V = ocv - b.last_current_a * b.r0 - b.v1
    return BatteryStatus(
        connected = true,
        voltage_v = V,
        current_a = b.last_current_a,
        remaining = b.soc,
        warning = _warning_from_remaining(b.soc, b.low_thr, b.crit_thr, b.emerg_thr),
    )
end

end # module Powertrain
