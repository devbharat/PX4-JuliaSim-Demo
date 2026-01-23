"""PX4Lockstep.Sim.Autopilots.UORBInjection

Multi-rate uORB *injection* helpers.

Why this exists
---------------
The long-term goal is to run **PX4 EKF2 in the loop** while generating raw sensor
data on the Julia side at different rates (IMU, mag, baro, GPS, ...).

To keep the interface clean and deterministic, we centralize a simple boundary rule:

1. Queue all uORB publishes that are *due* at the current `time_us`.
2. Call `step_uorb!(handle, time_us)` exactly once.
3. Read back configured uORB subscriptions.

This file implements:

* a `PX4StepContext` (inputs + common derived values used by multiple messages)
* periodic injection sources (`PeriodicUORBInjection`) that can run at their own
  integer microsecond periods
* a small `PX4UORBInjector` that owns a set of injection sources and applies them
  at each PX4 step.

Note: this does *not* prescribe how sensor models are implemented. The intent is
that future sensor components produce values on the Julia side, and the injector
turns those into uORB messages at the appropriate rates.
"""

# ----------------------------------------------------------------------------
# Context
# ----------------------------------------------------------------------------

"""Common context for one PX4 step.

The fields here are intentionally *plain data* (no handle pointers) so that
message builders stay deterministic and easy to test.
"""
Base.@kwdef struct PX4StepContext
    # Primary inputs
    time_us::UInt64
    pos_ned::Vec3
    vel_ned::Vec3
    q_bn::Quat
    ω_body::Vec3
    cmd::AutopilotCommand
    landed::Bool
    battery::BatteryStatus
    # Phase 5.3: full battery vector (deterministic order).
    batteries::Vector{BatteryStatus} = BatteryStatus[]

    # Derived values used across multiple topics
    yaw_rad::Float64
    lat_deg::Float64
    lon_deg::Float64
    alt_msl_m::Float64

    ref_lat_deg::Float64
    ref_lon_deg::Float64
    ref_alt_m::Float64

    auto_mode::Bool
    nav_state::UInt8
    arming_state::UInt8
    control_allocator_enabled::Bool
end

# ----------------------------------------------------------------------------
# Injection sources
# ----------------------------------------------------------------------------

"""Abstract uORB injection source.

An injection source may publish one uORB topic instance at a configurable rate.

Interface:

* `uorb_period_us(src)` → `UInt64`
    * `0` means "every PX4 step"
* `inject_due!(src, handle, ctx, t0_us)`
"""
abstract type AbstractUORBInjectionSource end

"""Publish period of the injection source.

`0` means publish every PX4 step.
"""
uorb_period_us(::AbstractUORBInjectionSource) = UInt64(0)

"""Human-friendly name for debugging."""
uorb_name(::AbstractUORBInjectionSource) = :uorb_injection

const EVERY_STEP_US = UInt64(0)

@inline function _is_due(time_us::UInt64, t0_us::UInt64, period_us::UInt64, phase_us::UInt64)
    period_us == 0 && return true
    time_us < t0_us + phase_us && return false
    return ((time_us - t0_us - phase_us) % period_us) == 0
end

"""Generic periodic uORB injection.

`builder` must be callable as `builder(ctx)::T` and should be allocation-free.
"""
Base.@kwdef mutable struct PeriodicUORBInjection{T<:UORBMsg,F} <: AbstractUORBInjectionSource
    name::Symbol = :uorb_injection
    pub::UORBPublisher{T}
    period_us::UInt64 = UInt64(0)
    phase_us::UInt64 = UInt64(0)
    builder::F
end

uorb_period_us(src::PeriodicUORBInjection) = src.period_us
uorb_name(src::PeriodicUORBInjection) = src.name

@inline function inject_due!(
    src::PeriodicUORBInjection{T,F},
    handle::LockstepHandle,
    ctx::PX4StepContext,
    t0_us::UInt64,
) where {T,F}
    _is_due(ctx.time_us, t0_us, src.period_us, src.phase_us) || return nothing
    msg = src.builder(ctx)::T
    publish!(handle, src.pub, msg)
    return nothing
end

"""Home position injection.

Home position needs a monotonically increasing `update_count`.
We keep that counter inside the injection source to avoid coupling it to the
autopilot step implementation.
"""
Base.@kwdef mutable struct HomePositionUORBInjection <: AbstractUORBInjectionSource
    name::Symbol = :home_position
    pub::UORBPublisher{HomePositionMsg}
    home::HomeLocation
    period_us::UInt64 = UInt64(0)
    phase_us::UInt64 = UInt64(0)
    update_count::UInt32 = UInt32(0)
end

uorb_period_us(src::HomePositionUORBInjection) = src.period_us
uorb_name(src::HomePositionUORBInjection) = src.name

@inline function inject_due!(
    src::HomePositionUORBInjection,
    handle::LockstepHandle,
    ctx::PX4StepContext,
    t0_us::UInt64,
)
    _is_due(ctx.time_us, t0_us, src.period_us, src.phase_us) || return nothing
    # Home is considered valid if lat/lon are finite.
    if isfinite(src.home.lat_deg) && isfinite(src.home.lon_deg)
        src.update_count += UInt32(1)
        msg = _home_position_msg(
            ctx.time_us,
            src.home.lat_deg,
            src.home.lon_deg,
            src.home.alt_msl_m,
            src.update_count,
        )
        publish!(handle, src.pub, msg)
    end
    return nothing
end

# ----------------------------------------------------------------------------
# Injector
# ----------------------------------------------------------------------------

"""A collection of uORB injection sources.

The injector also captures `t0_us` (first time it is called) to make schedules
relative to sim start, rather than absolute wall-clock timestamps.
"""
mutable struct PX4UORBInjector
    initialized::Bool
    t0_us::UInt64
    sources::Vector{AbstractUORBInjectionSource}
end

PX4UORBInjector() = PX4UORBInjector(false, UInt64(0), AbstractUORBInjectionSource[])

"""Add a source to an injector."""
function add_source!(inj::PX4UORBInjector, src::AbstractUORBInjectionSource)
    push!(inj.sources, src)
    return inj
end

"""Inject all uORB messages that are due at `ctx.time_us` (queueing only)."""
function inject_due!(inj::PX4UORBInjector, handle::LockstepHandle, ctx::PX4StepContext)
    if !inj.initialized
        inj.t0_us = ctx.time_us
        inj.initialized = true
    end
    for src in inj.sources
        inject_due!(src, handle, ctx, inj.t0_us)
    end
    return nothing
end

"""Return all non-zero publish periods (microseconds)."""
function injection_periods_us(inj::PX4UORBInjector)
    periods = UInt64[]
    for src in inj.sources
        p = uorb_period_us(src)
        p > 0 && push!(periods, p)
    end
    return periods
end

"""Recommended PX4 step dt based on injection periods.

Returns `nothing` if no source declares a non-zero period (i.e. everything is
"publish every step").
"""
function recommended_step_dt_us(inj::PX4UORBInjector)
    periods = injection_periods_us(inj)
    isempty(periods) && return nothing
    return minimum(periods)
end


# ----------------------------------------------------------------------------
# Default injector: "state injection" topics
# ----------------------------------------------------------------------------

"""Build the default *state injection* uORB injector.

This reproduces the previous behavior where the Julia simulator publishes a set
of state/command topics directly into PX4 via uORB (battery, attitude, position,
status...).

All default topics publish **every PX4 step** (period = 0), so the injector
itself does not constrain the PX4 step frequency.
"""
function build_state_injection_injector(bridge::UORBBridge, home::HomeLocation)
    inj = PX4UORBInjector()

    # --------------------------
    # Battery status (multi-instance)
    # --------------------------
    if haskey(bridge.pubs, :battery_status)
        entries = bridge.pubs[:battery_status]

        # Back-compat: if only one publisher is configured, publish the legacy primary battery.
        if length(entries) == 1
            pub = entries[1][1]::UORBPublisher{BatteryStatusMsg}
            inst = entries[1][2]
            inst >= -1 || error("battery_status instance must be -1 or >=0 (got $(inst))")
            bidx0 = inst >= 0 ? Int(inst) : 0
            add_source!(inj, PeriodicUORBInjection(
                name = :battery_status,
                pub = pub,
                period_us = EVERY_STEP_US,
                builder = ctx -> begin
                    length(ctx.batteries) > bidx0 ||
                        error("battery_status instance $(bidx0) requires ctx.batteries length >= $(bidx0 + 1)")
                    _battery_status_msg(ctx.time_us, ctx.batteries[bidx0 + 1]; id = UInt8(bidx0))
                end,
            ))
        else
            # Phase 5.3: publish one battery_status per configured publisher instance.
            bidx0s = Int[]
            for (j, (_pub_any, inst)) in enumerate(entries)
                inst >= -1 || error("battery_status instance must be -1 or >=0 (got $(inst))")
                bidx0 = inst >= 0 ? Int(inst) : (j - 1)  # 0-based battery index
                push!(bidx0s, bidx0)
            end
            length(unique(bidx0s)) == length(bidx0s) ||
                error("battery_status publishers map to duplicate battery indices: $(bidx0s)")

            for (j, (pub_any, inst)) in enumerate(entries)
                pub = pub_any::UORBPublisher{BatteryStatusMsg}
                bidx0 = inst >= 0 ? Int(inst) : (j - 1)  # 0-based battery index
                let bidx0 = bidx0
                    add_source!(inj, PeriodicUORBInjection(
                        name = Symbol("battery_status_$(bidx0)"),
                        pub = pub,
                        period_us = EVERY_STEP_US,
                        builder = ctx -> begin
                            length(ctx.batteries) > bidx0 ||
                                error("battery_status instance $(bidx0) requires ctx.batteries length >= $(bidx0 + 1)")
                            _battery_status_msg(
                                ctx.time_us,
                                ctx.batteries[bidx0 + 1];
                                id = UInt8(bidx0),
                            )
                        end,
                    ))
                end
            end
        end
    end

    # --------------------------
    # State injection (single-instance topics)
    # --------------------------

    if haskey(bridge.pubs, :vehicle_attitude)
        pub = bridge.pubs[:vehicle_attitude][1][1]::UORBPublisher{VehicleAttitudeMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_attitude,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_attitude_msg(ctx.time_us, ctx.q_bn),
        ))
    end

    if haskey(bridge.pubs, :vehicle_local_position)
        pub = bridge.pubs[:vehicle_local_position][1][1]::UORBPublisher{VehicleLocalPositionMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_local_position,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_local_position_msg(
                ctx.time_us,
                ctx.pos_ned,
                ctx.vel_ned,
                ctx.yaw_rad,
                ctx.ref_lat_deg,
                ctx.ref_lon_deg,
                ctx.ref_alt_m,
            ),
        ))
    end

    if haskey(bridge.pubs, :vehicle_global_position)
        pub = bridge.pubs[:vehicle_global_position][1][1]::UORBPublisher{VehicleGlobalPositionMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_global_position,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_global_position_msg(
                ctx.time_us,
                ctx.lat_deg,
                ctx.lon_deg,
                ctx.alt_msl_m,
            ),
        ))
    end

    if haskey(bridge.pubs, :vehicle_angular_velocity)
        pub = bridge.pubs[:vehicle_angular_velocity][1][1]::UORBPublisher{VehicleAngularVelocityMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_angular_velocity,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_angular_velocity_msg(ctx.time_us, ctx.ω_body),
        ))
    end

    if haskey(bridge.pubs, :vehicle_land_detected)
        pub = bridge.pubs[:vehicle_land_detected][1][1]::UORBPublisher{VehicleLandDetectedMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_land_detected,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_land_detected_msg(ctx.time_us, ctx.landed),
        ))
    end

    if haskey(bridge.pubs, :vehicle_status)
        pub = bridge.pubs[:vehicle_status][1][1]::UORBPublisher{VehicleStatusMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_status,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_status_msg(ctx.time_us, ctx.nav_state, ctx.arming_state),
        ))
    end

    if haskey(bridge.pubs, :vehicle_control_mode)
        pub = bridge.pubs[:vehicle_control_mode][1][1]::UORBPublisher{VehicleControlModeMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :vehicle_control_mode,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _vehicle_control_mode_msg(
                ctx.time_us,
                ctx.cmd,
                ctx.auto_mode,
                ctx.nav_state,
                ctx.control_allocator_enabled,
            ),
        ))
    end

    if haskey(bridge.pubs, :actuator_armed)
        pub = bridge.pubs[:actuator_armed][1][1]::UORBPublisher{ActuatorArmedMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :actuator_armed,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _actuator_armed_msg(ctx.time_us, ctx.cmd),
        ))
    end

    if haskey(bridge.pubs, :home_position)
        pub = bridge.pubs[:home_position][1][1]::UORBPublisher{HomePositionMsg}
        add_source!(inj, HomePositionUORBInjection(pub = pub, home = home))
    end

    if haskey(bridge.pubs, :geofence_status)
        pub = bridge.pubs[:geofence_status][1][1]::UORBPublisher{GeofenceStatusMsg}
        add_source!(inj, PeriodicUORBInjection(
            name = :geofence_status,
            pub = pub,
            period_us = EVERY_STEP_US,
            builder = ctx -> _geofence_status_msg(ctx.time_us),
        ))
    end

    return inj
end
