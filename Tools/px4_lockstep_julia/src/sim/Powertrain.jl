"""PX4Lockstep.Sim.Powertrain

Battery / powertrain models.

Battery models for sim-side powertrain and PX4 `battery_status` injection.
This module provides a small interface so models can be swapped without touching:
- the sim engine
- the PX4 lockstep bridge

Included models:

- `IdealBattery`: deterministic coulomb counting + constant voltage (baseline)
- `TheveninBattery`: 1st-order Thevenin equivalent (OCV + R0 + RC polarization)

The Thevenin model is still intentionally lightweight (no thermal yet), but it is a
useful starting point for battery-related RTL and energy studies.
"""
module Powertrain

export BatteryStatus, AbstractBatteryModel, IdealBattery, TheveninBattery, step!, status

# NOTE
# ----
# In the canonical runtime engine, battery *state* (SOC/V1) is integrated as part of
# `Plant.PlantState`. Battery model objects in this module are therefore intended to
# be **parameter-only**.
#
# For stand-alone stepping (outside the integrated plant), use the
# `BatteryState` helper below and call `step!(model, state, I_bus, dt)`.

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

"""Battery state for stand-alone battery stepping.

The canonical multirotor plant integrates battery state (SOC/V1) inside
`Plant.PlantState`, so the battery model objects in this module are parameter-only.

This helper exists for any workflows that still want to step a battery model outside
the integrated plant.
"""
Base.@kwdef mutable struct BatteryState
    soc::Float64 = 1.0
    v1::Float64 = 0.0
    last_current_a::Float64 = 0.0
end

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
- Current draw comes from the modeled bus current.

This is intentionally simple and deterministic.
"""
struct IdealBattery <: AbstractBatteryModel
    capacity_c::Float64          # coulombs
    soc0::Float64                # initial SOC used by `Plant.init_plant_state`
    voltage_v::Float64
    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
end

function IdealBattery(;
    capacity_ah::Float64 = 5.0,
    soc0::Float64 = 1.0,
    voltage_v::Float64 = 12.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
)
    capacity_c = capacity_ah * 3600.0
    return IdealBattery(capacity_c, _clamp01(soc0), voltage_v, low_thr, crit_thr, emerg_thr)
end

"""Construct a battery state from model parameters."""
@inline function battery_state(b::IdealBattery)
    return BatteryState(soc = _clamp01(b.soc0), v1 = 0.0, last_current_a = 0.0)
end

"""Advance the ideal battery state by `dt` seconds given bus current draw."""
function step!(b::IdealBattery, st::BatteryState, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    st.last_current_a = I
    st.soc = _clamp01(st.soc - (I * dt) / b.capacity_c)
    # Ideal battery has no polarization state.
    st.v1 = 0.0
    return nothing
end

function status(b::IdealBattery, st::BatteryState)::BatteryStatus
    rem = _clamp01(st.soc)
    return BatteryStatus(
        connected = true,
        voltage_v = b.voltage_v,
        current_a = st.last_current_a,
        remaining = rem,
        warning = _warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr),
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
struct TheveninBattery <: AbstractBatteryModel
    capacity_c::Float64
    soc0::Float64                # initial SOC used by `Plant.init_plant_state`

    ocv_soc::Vector{Float64}
    ocv_v::Vector{Float64}

    r0::Float64
    r1::Float64
    c1::Float64
    v1_0::Float64                # initial polarization voltage used by `Plant.init_plant_state`

    min_voltage_v::Float64

    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
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
    min_voltage_v::Float64 = 0.0,
    low_thr::Float64 = 0.15,
    crit_thr::Float64 = 0.10,
    emerg_thr::Float64 = 0.05,
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
        min_voltage_v,
        low_thr,
        crit_thr,
        emerg_thr,
    )
end

"""Construct a battery state from model parameters."""
@inline function battery_state(b::TheveninBattery)
    return BatteryState(soc = _clamp01(b.soc0), v1 = Float64(b.v1_0), last_current_a = 0.0)
end

function step!(b::TheveninBattery, st::BatteryState, I_bus_a::Float64, dt::Float64)
    I = max(0.0, Float64(I_bus_a))
    st.last_current_a = I

    # SOC coulomb counting.
    st.soc = _clamp01(st.soc - (I * dt) / b.capacity_c)

    # Polarization voltage dynamics.
    if b.r1 > 0.0 && b.c1 > 0.0
        τ = b.r1 * b.c1
        if τ < 1e-9
            st.v1 = I * b.r1
        else
            α = exp(-dt / τ)
            st.v1 = st.v1 * α + I * b.r1 * (1.0 - α)
        end
    else
        st.v1 = 0.0
    end

    return nothing
end

function status(b::TheveninBattery, st::BatteryState)::BatteryStatus
    rem = _clamp01(st.soc)
    ocv = _interp_ocv(b.ocv_soc, b.ocv_v, rem)
    V = ocv - st.last_current_a * b.r0 - st.v1
    V = max(b.min_voltage_v, V)
    return BatteryStatus(
        connected = true,
        voltage_v = V,
        current_a = st.last_current_a,
        remaining = rem,
        warning = _warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr),
    )
end

end # module Powertrain
