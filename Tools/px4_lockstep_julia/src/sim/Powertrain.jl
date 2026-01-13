"""PX4Lockstep.Sim.Powertrain

Battery / powertrain models.

Long-term target (from Gradient proposal):
* 1st-order Thevenin equivalent circuit (OCV + R + RC branch)
* OCV and resistance temperature dependence
* simple thermal model with convection cooling

For now, we provide a small interface plus a minimal deterministic battery...

The intent is that you can swap the battery model without changing the sim engine or the
PX4 lockstep bridge.
"""
module Powertrain

export BatteryStatus,
       AbstractBatteryModel,
       IdealBattery,
       step!, status

"""Battery status to be injected into PX4 (`battery_status` uORB).

Fields match what we typically care about in closed-loop testing.
"""
Base.@kwdef struct BatteryStatus
    connected::Bool = true
    voltage_v::Float64 = 12.0
    current_a::Float64 = 0.0
    remaining::Float64 = 1.0   # [0,1]
    warning::Int32 = 0         # PX4 battery_status warning enum
end

abstract type AbstractBatteryModel end

"""Ideal coulomb-counted battery model.

* SOC integrates from a constant capacity.
* Voltage is constant (or can be set externally).
* Current draw can be estimated from motor commands via a user-provided callback.

This is intentionally simple and stable. Replace it with Thevenin/thermal later.
"""
mutable struct IdealBattery <: AbstractBatteryModel
    capacity_c::Float64         # coulombs
    soc::Float64                # [0,1]
    voltage_v::Float64
    last_current_a::Float64
    # Thresholds (fraction remaining) that map to PX4 warning states.
    low_thr::Float64
    crit_thr::Float64
    emerg_thr::Float64
    # User hook: current estimator I = f(cmds)
    current_estimator::Function
end

function IdealBattery(; capacity_ah::Float64=5.0,
                        soc0::Float64=1.0,
                        voltage_v::Float64=12.0,
                        low_thr::Float64=0.15,
                        crit_thr::Float64=0.10,
                        emerg_thr::Float64=0.05,
                        current_estimator::Function=(cmds)->0.0)
    capacity_c = capacity_ah * 3600.0
    return IdealBattery(capacity_c, soc0, voltage_v, 0.0, low_thr, crit_thr, emerg_thr, current_estimator)
end

"""Advance the battery model by `dt`.

`motor_cmds` is an actuator-level command vector (typically normalized [0..1] per motor).
"""
function step!(b::IdealBattery, motor_cmds, dt::Float64)
    I = Float64(b.current_estimator(motor_cmds))
    b.last_current_a = I
    # Coulomb counting: dSOC = -I*dt / Q
    b.soc = clamp(b.soc - (I * dt) / b.capacity_c, 0.0, 1.0)
    return nothing
end

function _warning_from_soc(b::IdealBattery)
    soc = b.soc
    if soc <= b.emerg_thr
        return Int32(3)  # EMERGENCY
    elseif soc <= b.crit_thr
        return Int32(2)  # CRITICAL
    elseif soc <= b.low_thr
        return Int32(1)  # LOW
    else
        return Int32(0)  # NONE
    end
end

function status(b::IdealBattery)::BatteryStatus
    return BatteryStatus(
        connected=true,
        voltage_v=b.voltage_v,
        current_a=b.last_current_a,
        remaining=b.soc,
        warning=_warning_from_soc(b),
    )
end

end # module Powertrain
