"""PX4Lockstep.Sim.PlantSimulation

Event-driven variable-step simulation engine.

Overview
--------
This engine integrates a *full* continuous-time plant state (rigid body + actuator
states + rotor speed states + battery states) with either fixed-step (Euler/RK4) or
adaptive (RK23/RK45) ODE solvers.

Key architectural idea
---------------------
* Discrete event boundaries define when inputs may change:
  - PX4/autopilot tick (new actuator commands)
  - wind/turbulence tick (new disturbance sample)
  - log tick (sample-and-hold logging snapshot)
  - scenario/failure events (non-periodic boundaries)
* Between boundaries, inputs are held constant and the plant RHS is a pure function of
  (t, x, u) with **no RNG** and no hidden mutation, so adaptive substeps remain
  deterministic.

Current limitations (non-blocking for flight)
--------------------------------------------
* Scenario `AtTime` events are treated as true time boundaries (hybrid system style).
  `When` events are evaluated when `process_events!` is called at boundaries.
* Battery↔bus↔motor coupling uses an analytic linear-region solve with deterministic
  fallback (region-classified iteration + fixed-iteration bisection).
* Contact uses a penalty force without event detection (acceptable for "in-air" focus).
"""
module PlantSimulation

using Random
using StaticArrays

using ..Types: Vec3, Quat, WorldOrigin, vec3, quat_rotate_inv
using ..RigidBody: RigidBodyState, RigidBodyDeriv
using ..Environment:
    EnvironmentModel, SampledWind, step_wind!, sample_wind!, air_density
import ..Vehicles
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, mass
import ..Propulsion
import ..Simulation: VehicleInstance, _sync_world_origin
using ..Autopilots:
    AbstractAutopilot,
    AutopilotCommand,
    autopilot_step,
    autopilot_output_type,
    max_internal_rate_hz,
    PX4LockstepAutopilot
using ..Estimators: AbstractEstimator, TruthEstimator, estimate!
using ..Scenario:
    AbstractScenario,
    ScriptedScenario,
    EventScenario,
    scenario_step,
    process_events!,
    next_event_us
import ..Powertrain
using ..Powertrain: AbstractBatteryModel, IdealBattery, BatteryStatus, status
using ..Logging: AbstractLogSink, SimLog, log!, close!, reserve!
using ..Contacts: AbstractContactModel, NoContact
using ..Contacts: contact_force_ned
using ..Integrators: AbstractIntegrator, step_integrator
using ..Plant: PlantState, PlantInput, PlantDeriv, PlantOutputs, init_plant_state, sync_components_from_plant!

export PlantSimulationConfig, PlantSimulationInstance, time_s, time_us, step_to_next_event!, run!

############################
# Config
############################

"""Configuration for the event-driven/variable-step engine.

Semantics
---------
* `dt_autopilot`: lockstep PX4 step period. Must be microsecond-quantized.
* `dt_wind`: disturbance update period (OU turbulence, gust scheduler). Must be microsecond-quantized.
* `dt_log`: logging period. Must be microsecond-quantized.

Notes
-----
* This is deliberately separate from `Simulation.SimulationConfig` to avoid breaking the
  legacy engine while this refactor is in progress.
* The long-term goal is to unify configs once the plant ODE is fully implemented.
"""
Base.@kwdef struct PlantSimulationConfig
    t0::Float64 = 0.0
    t_end::Float64 = 20.0

    dt_autopilot::Float64 = 0.01
    dt_wind::Float64 = 0.002
    dt_log::Float64 = 0.01

    seed::UInt64 = 0

    strict_lockstep_rates::Bool = true
end

############################
# Timebase helpers
############################

@inline function _dt_to_us(dt::Float64; name::AbstractString = "dt")::UInt64
    dt > 0 || throw(ArgumentError("$name must be > 0"))
    dt_us = UInt64(round(dt * 1e6))
    dt_back = Float64(dt_us) * 1e-6
    abs(dt - dt_back) <= 1e-12 || throw(
        ArgumentError("$name must be an integer number of microseconds (got dt=$dt => $dt_us μs)"),
    )
    dt_us > 0 || throw(ArgumentError("$name is too small (rounded to 0 μs)"))
    return dt_us
end

@inline function _time_to_us(t::Float64; name::AbstractString = "t")::UInt64
    t >= 0 || throw(ArgumentError("$name must be >= 0"))
    t_us = UInt64(round(t * 1e6))
    t_back = Float64(t_us) * 1e-6
    abs(t - t_back) <= 1e-12 || throw(
        ArgumentError("$name must be an integer number of microseconds (got $t => $t_us μs)"),
    )
    return t_us
end

"""Stable splitmix64 for deriving independent RNG seeds."""
@inline function _splitmix64(x::UInt64)::UInt64
    z = x + 0x9E3779B97F4A7C15
    z = (z ⊻ (z >> 30)) * 0xBF58476D1CE4E5B9
    z = (z ⊻ (z >> 27)) * 0x94D049BB133111EB
    return z ⊻ (z >> 31)
end

@inline function _seed_to_int(s::UInt64)::Int
    return Int(s % UInt64(typemax(Int)))
end

function _make_rngs(seed::UInt64)
    s_wind = _splitmix64(seed ⊻ 0x57494E445F534545)  # "WIND_SEE"
    s_est = _splitmix64(seed ⊻ 0x4553545F53454544)   # "EST_SEED"
    s_misc = _splitmix64(seed ⊻ 0x4D4953435F534545)  # "MISC_SEE"
    return MersenneTwister(_seed_to_int(s_wind)),
    MersenneTwister(_seed_to_int(s_est)),
    MersenneTwister(_seed_to_int(s_misc))
end

############################
# Microsecond periodic triggers
############################

mutable struct PeriodicTriggerUs
    period_us::UInt64
    next_us::UInt64
end

@inline function _init_trigger(dt_us::UInt64, t0_us::UInt64)
    return PeriodicTriggerUs(dt_us, t0_us)
end

@inline function _advance!(trig::PeriodicTriggerUs)
    trig.next_us += trig.period_us
    return nothing
end

############################
# Plant dynamics wrapper
############################

"""Coupled plant dynamics (shell).

This functor is meant to be the single RHS used by ODE integrators:

    f(t, x::PlantState, u::PlantInput) -> PlantDeriv

Coupled plant RHS implemented here:
* actuator ODEs (first/second order, saturation/rate limits)
* motor/ESC electrical torque/current
* rotor ω dynamics + thrust/drag torque (including inflow corrections)
* battery SOC + polarization voltage + terminal voltage (algebraic bus solve)
* rigid-body forces/torques + contact forces

Determinism requirements:
* Must be a pure function: no RNG, no global state, no mutation of `sim.env` / `sim.vehicle`.
"""
struct PlantDynamicsWithContact{M,E,C,AM,AS,P,B}
    model::M
    env::E
    contact::C
    motor_actuators::AM
    servo_actuators::AS
    propulsion::P
    battery::B
end

############################
# Pure subsystem dynamics helpers
############################

@inline function _actuator_derivs(
    ::Vehicles.DirectActuators,
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    # Direct actuators are algebraic; their state must be snapped at event boundaries.
    return zero(y), zero(ydot)
end

@inline function _actuator_derivs(
    a::Vehicles.FirstOrderActuators{N},
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    τ = a.τ
    if !(isfinite(τ) && τ > 0.0)
        # Degenerate: treat as direct.
        return zero(y), zero(ydot)
    end
    return (u_cmd - y) / τ, zero(ydot)
end

@inline function _actuator_derivs(
    a::Vehicles.SecondOrderActuators{N},
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    ωn = a.ωn
    ζ = a.ζ

    # Rate limiting semantics:
    # * The legacy discrete model clamps `ydot` after each update.
    # * In continuous time this is a differential inclusion; we approximate it by:
    #   - computing dynamics using a clamped effective `ydot` (prevents runaway), and
    #   - projecting `ydot` back into bounds after each accepted integration interval
    #     (see `step_to_next_event!`).
    rl = a.rate_limit
    ydot_eff = ydot
    if isfinite(rl)
        ydot_eff = map(v -> clamp(v, -rl, rl), ydot)
    end

    # ẏ = ydot
    y_dot = ydot_eff

    # ÿ = ωn²(u - y) - 2ζωn ẏ
    ydd = (ωn * ωn) .* (u_cmd .- y) .- (2.0 * ζ * ωn) .* ydot_eff

    # If at the limit and accelerating further into saturation, freeze acceleration.
    if isfinite(rl)
        ydd = SVector{N,Float64}(ntuple(i -> begin
            yi = ydot_eff[i]
            ai = ydd[i]
            if yi >= rl && ai > 0.0
                0.0
            elseif yi <= -rl && ai < 0.0
                0.0
            else
                ai
            end
        end, N))
    end

    return y_dot, ydd
end

@inline function _battery_ocv(b::Powertrain.IdealBattery, soc::Float64)
    return b.voltage_v
end

@inline function _battery_ocv(b::Powertrain.TheveninBattery, soc::Float64)
    # Use the internal OCV curve helper for now.
    return Powertrain._interp_ocv(b.ocv_soc, b.ocv_v, soc)
end

@inline function _battery_capacity_c(b::Powertrain.IdealBattery)
    return b.capacity_c
end
@inline function _battery_capacity_c(b::Powertrain.TheveninBattery)
    return b.capacity_c
end

@inline function _battery_min_voltage(b::Powertrain.IdealBattery)
    return 0.0
end
@inline function _battery_min_voltage(b::Powertrain.TheveninBattery)
    return b.min_voltage_v
end

@inline function _battery_r0(b::Powertrain.IdealBattery)
    return 0.0
end
@inline function _battery_r0(b::Powertrain.TheveninBattery)
    return b.r0
end

@inline function _battery_v1_dot(::Powertrain.IdealBattery, v1::Float64, I_bus::Float64)
    return 0.0
end

@inline function _battery_v1_dot(b::Powertrain.TheveninBattery, v1::Float64, I_bus::Float64)
    if b.r1 > 0.0 && b.c1 > 0.0
        return -(v1 / (b.r1 * b.c1)) + (I_bus / b.c1)
    else
        return 0.0
    end
end

@inline function _battery_status_from_state(
    b::Powertrain.IdealBattery,
    soc::Float64,
    v1::Float64,
    I_bus::Float64,
    V_bus::Float64,
)::BatteryStatus
    rem = clamp(soc, 0.0, 1.0)
    warn = Powertrain._warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr)
    return BatteryStatus(
        connected = true,
        voltage_v = V_bus,
        current_a = I_bus,
        remaining = rem,
        warning = warn,
    )
end

@inline function _battery_status_from_state(
    b::Powertrain.TheveninBattery,
    soc::Float64,
    v1::Float64,
    I_bus::Float64,
    V_bus::Float64,
)::BatteryStatus
    rem = clamp(soc, 0.0, 1.0)
    warn = Powertrain._warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr)
    return BatteryStatus(
        connected = true,
        voltage_v = V_bus,
        current_a = I_bus,
        remaining = rem,
        warning = warn,
    )
end

@inline function _motorprop_unit_eval(
    unit::Propulsion.MotorPropUnit,
    ω::Float64,
    duty::Float64,
    V_bus::Float64,
    ρ::Float64,
    Vax::Float64,
)
    if !unit.enabled
        b = unit.motor.viscous_friction_nm_per_rad_s
        J = unit.motor.J_kgm2
        ω_dot = (-(b * ω)) / J
        if ω <= 0.0 && ω_dot < 0.0
            ω_dot = 0.0
        end
        return 0.0, 0.0, ω_dot, 0.0, 0.0
    end

    esc = unit.esc
    motor = unit.motor
    prop = unit.prop

    d = clamp(duty, 0.0, 1.0)
    if d < esc.deadzone
        d = 0.0
    end

    V_m = d * V_bus
    Ke = Propulsion.motor_Ke(motor)
    Kt = Propulsion.motor_Kt(motor)
    R = motor.R_ohm

    # Quasi-static motor current (clamp to [0,Imax]).
    I_m = (V_m - Ke * ω) / R
    I_m = clamp(I_m, 0.0, motor.max_current_a)

    τ_e = Kt * max(0.0, I_m - motor.I0_a)
    τ_load = Propulsion.prop_torque(prop, ρ, ω, Vax)

    b = motor.viscous_friction_nm_per_rad_s
    J = motor.J_kgm2
    ω_dot = (τ_e - τ_load - b * ω) / J

    # Prevent unphysical negative spin due to numerical overshoot.
    if ω <= 0.0 && ω_dot < 0.0
        ω_dot = 0.0
    end

    # Aerodynamic outputs at current ω.
    T = Propulsion.prop_thrust(prop, ρ, ω, Vax)
    Q = Propulsion.prop_torque(prop, ρ, ω, Vax)

    I_bus = (d * I_m) / max(1e-6, esc.η)
    return T, Q, ω_dot, I_m, I_bus
end

############################
# Battery↔bus↔motor coupling helpers
############################

@inline function _bus_current_total(
    p::Propulsion.QuadRotorSet{N},
    ω::SVector{N,Float64},
    duty::SVector{N,Float64},
    V_bus::Float64,
) where {N}
    I_bus_total = 0.0
    @inbounds for i = 1:N
        unit = p.units[i]
        unit.enabled || continue

        d = clamp(duty[i], 0.0, 1.0)
        if d < unit.esc.deadzone
            continue
        end

        motor = unit.motor
        Ke = Propulsion.motor_Ke(motor)
        R = motor.R_ohm
        η = max(1e-6, unit.esc.η)

        # Quasi-static motor current.
        I_m = (d * V_bus - Ke * ω[i]) / R
        I_m = clamp(I_m, 0.0, motor.max_current_a)

        I_bus_total += (d * I_m) / η
    end
    return I_bus_total
end


@inline function _try_solve_bus_voltage_linear(
    p::Propulsion.QuadRotorSet{N},
    ω::SVector{N,Float64},
    duty::SVector{N,Float64},
    V0::Float64,
    R0::Float64,
    V_min::Float64,
)::Union{Nothing,Float64} where {N}
    # Fast path: assume all active motors are in the linear (unsaturated, positive-current) region.
    #
    # Under that assumption, the total bus current is affine in V:
    #   I_bus(V) = A*V + B
    # and the Thevenin+R0 bus equation has the analytic solution:
    #   (1 + R0*A) V = V0 - R0*B
    #
    # We validate the region assumption at the resulting V; if it fails, caller falls back to the
    # robust region-classified iteration + bisection.
    A = 0.0
    B = 0.0
    any_active = false
    @inbounds for i = 1:N
        unit = p.units[i]
        unit.enabled || continue

        d = clamp(duty[i], 0.0, 1.0)
        if d < unit.esc.deadzone
            continue
        end
        any_active = true

        motor = unit.motor
        Ke = Propulsion.motor_Ke(motor)
        R = motor.R_ohm
        η = max(1e-6, unit.esc.η)

        A += (d * d) / (η * R)
        B += (-d * Ke * ω[i]) / (η * R)
    end

    # No active load → open-circuit voltage (clamped).
    if !any_active
        return V0
    end

    denom = 1.0 + R0 * A
    denom = max(denom, 1e-12)
    V = (V0 - R0 * B) / denom
    V = clamp(V, V_min, V0)
    isfinite(V) || return nothing

    # Validate that every active motor is actually in the linear region at this V.
    @inbounds for i = 1:N
        unit = p.units[i]
        unit.enabled || continue

        d = clamp(duty[i], 0.0, 1.0)
        if d < unit.esc.deadzone
            continue
        end

        motor = unit.motor
        Ke = Propulsion.motor_Ke(motor)
        R = motor.R_ohm
        Imax = motor.max_current_a

        I_lin = (d * V - Ke * ω[i]) / R
        # Strict inequalities: if we hit the clamp boundaries, the affine model is invalid.
        if !(I_lin > 0.0 && I_lin < Imax)
            return nothing
        end
    end

    return V
end


"""Solve the bus voltage V_bus for the instantaneous (ω, duty, SOC, V1) state.

We solve the scalar fixed point implied by the Thevenin+R0 model:

    V = max(V_min, (OCV - V1) - R0 * I_bus(V))

where I_bus(V) is monotone nondecreasing in V for the quasi-static motor current model.

Strategy (deterministic):
1) Try a fast analytic solve assuming all active motors are in the linear (unsaturated) region.
   Validate the region assumption; if it holds, this is a one-shot exact solve.
2) Otherwise, try a region-classified analytic solve by iterating region classification a few times.
3) If residual is not small, fall back to a fixed-iteration bisection on the monotone
   equation g(V)=V + R0*I(V) - (OCV - V1).

This is robust to deadzones and current clamping and remains deterministic.
"""
function _solve_bus_voltage(
    p::Propulsion.QuadRotorSet{N},
    ω::SVector{N,Float64},
    duty::SVector{N,Float64},
    ocv::Float64,
    v1::Float64,
    R0::Float64,
    V_min::Float64,
) where {N}
    # Defensive programming: the bus solve is a scalar root find inside the plant RHS.
    # If it ever goes non-finite, adaptive integrators will explode in hard-to-debug ways.
    if !isfinite(ocv) || !isfinite(v1) || !isfinite(R0) || !isfinite(V_min)
        error("non-finite battery parameters in bus solve: ocv=$(ocv) v1=$(v1) R0=$(R0) V_min=$(V_min)")
    end

    V0 = ocv - v1
    if !isfinite(V0)
        error("non-finite open-circuit voltage in bus solve: ocv=$(ocv) v1=$(v1) V0=$(V0)")
    end
    V0 = max(V_min, V0)

    # Ideal/no-R0 case.
    if !(isfinite(R0) && R0 > 0.0)
        return V0
    end

    # If already clamped, nothing to solve.
    if V0 <= V_min + 1e-12
        return V_min
    end


    # Fast path: if all active motors remain in the unsaturated linear region, we can solve for V_bus analytically.
    V_lin = _try_solve_bus_voltage_linear(p, ω, duty, V0, R0, V_min)
    if V_lin !== nothing
        return V_lin
    end

    V = V0

    # Region-classified analytic iteration (small fixed count, deterministic).
    for _ = 1:6
        A = 0.0
        B = 0.0
        @inbounds for i = 1:N
            unit = p.units[i]
            unit.enabled || continue

            d = clamp(duty[i], 0.0, 1.0)
            if d < unit.esc.deadzone
                continue
            end

            motor = unit.motor
            Ke = Propulsion.motor_Ke(motor)
            R = motor.R_ohm
            Imax = motor.max_current_a
            η = max(1e-6, unit.esc.η)

            I_lin = (d * V - Ke * ω[i]) / R
            if I_lin <= 0.0
                continue
            elseif I_lin >= Imax
                # Current-saturated region: constant I_bus.
                B += (d * Imax) / η
            else
                # Linear region: I_bus(V) = α V + β.
                A += (d * d) / (η * R)
                B += (-d * Ke * ω[i]) / (η * R)
            end
        end

        denom = 1.0 + R0 * A
        denom = max(denom, 1e-12)
        V_new = (V0 - R0 * B) / denom
        V_new = clamp(V_new, V_min, V0)

        if !isfinite(V_new)
            # Break to the bisection fallback with a valid bracket.
            V = V0
            break
        end

        if abs(V_new - V) < 1e-9
            V = V_new
            break
        end
        V = V_new
    end

    # Check residual; if poor, use deterministic bisection fallback.
    I = _bus_current_total(p, ω, duty, V)
    if !isfinite(I) || !isfinite(V)
        error(
            "non-finite intermediate bus solve value: V=$(V) I=$(I) ocv=$(ocv) v1=$(v1) R0=$(R0) V_min=$(V_min) ω=$(ω) duty=$(duty)",
        )
    end
    res = abs(V + R0 * I - V0)
    if res <= 1e-6
        return V
    end

    V_low = V_min
    V_high = V0

    I_low = _bus_current_total(p, ω, duty, V_low)
    I_high = _bus_current_total(p, ω, duty, V_high)
    if !isfinite(I_low) || !isfinite(I_high)
        error(
            "non-finite bus current during voltage bracket: I_low=$(I_low) I_high=$(I_high) V_low=$(V_low) V_high=$(V_high) ocv=$(ocv) v1=$(v1) R0=$(R0) ω=$(ω) duty=$(duty)",
        )
    end

    # Monotonicity contract required by bisection.
    if I_high + 1e-9 < I_low
        error(
            "bus current is not monotone in V_bus; bisection assumptions violated: I_low=$(I_low) I_high=$(I_high) V_low=$(V_low) V_high=$(V_high) ω=$(ω) duty=$(duty)",
        )
    end

    g_low = V_low + R0 * I_low - V0
    if g_low >= 0.0
        return V_low
    end

    g_high = V_high + R0 * I_high - V0
    if !isfinite(g_high)
        error("non-finite g_high in bus voltage solve: g_high=$(g_high) V_high=$(V_high)")
    end
    # If g(V_high) <= 0, the root is at the upper bound (typically I(V_high)=0).
    if g_high <= 0.0
        return V_high
    end
    if g_high + 1e-9 < g_low
        error(
            "g(V) is not monotone on bracket; bisection invalid: g_low=$(g_low) g_high=$(g_high) V_low=$(V_low) V_high=$(V_high) ω=$(ω) duty=$(duty)",
        )
    end

    # Fixed-iteration bisection (deterministic regardless of FP noise).
    for _ = 1:25
        V_mid = 0.5 * (V_low + V_high)
        I_mid = _bus_current_total(p, ω, duty, V_mid)
        if !isfinite(I_mid) || !isfinite(V_mid)
            error(
                "non-finite midpoint during bus voltage bisection: V_mid=$(V_mid) I_mid=$(I_mid) V_low=$(V_low) V_high=$(V_high) ocv=$(ocv) v1=$(v1) R0=$(R0) V0=$(V0) V_min=$(V_min) ω=$(ω) duty=$(duty)",
            )
        end
        g_mid = V_mid + R0 * I_mid - V0
        if g_mid > 0.0
            V_high = V_mid
        else
            V_low = V_mid
        end
    end

    V_sol = 0.5 * (V_low + V_high)
    if !isfinite(V_sol)
        error(
            "non-finite final bus voltage: V_sol=$(V_sol) V_low=$(V_low) V_high=$(V_high) ocv=$(ocv) v1=$(v1) R0=$(R0) V0=$(V0) V_min=$(V_min) ω=$(ω) duty=$(duty)",
        )
    end
    return V_sol
end

function _eval_propulsion_and_bus(
    p::Propulsion.QuadRotorSet{N},
    b::Powertrain.AbstractBatteryModel,
    env::EnvironmentModel,
    t::Float64,
    x::PlantState{N},
    u::PlantInput,
) where {N}
    # Atmosphere expects MSL altitude.
    alt_msl_m = env.origin.alt_msl_m - x.rb.pos_ned[3]
    ρ = air_density(env.atmosphere, alt_msl_m)

    # Air-relative velocity in body frame.
    v_air_ned = u.wind_ned - x.rb.vel_ned
    v_air_body = quat_rotate_inv(x.rb.q_bn, v_air_ned)
    Vax = -Float64(v_air_body[3])

    duties = SVector{N,Float64}(ntuple(i -> clamp(x.motors_y[i], 0.0, 1.0), N))

    soc = x.batt_soc
    v1 = x.batt_v1

    ocv = _battery_ocv(b, soc)
    R0 = _battery_r0(b)
    V_min = _battery_min_voltage(b)

    # Solve terminal/bus voltage with deterministic robust coupling.
    V_bus = _solve_bus_voltage(p, x.rotor_ω, duties, ocv, v1, R0, V_min)
    if !isfinite(V_bus)
        error(
            "non-finite V_bus from bus solver: V_bus=$(V_bus) ocv=$(ocv) v1=$(v1) R0=$(R0) V_min=$(V_min) soc=$(soc) ω=$(x.rotor_ω) duties=$(duties)",
        )
    end

    thrust = MVector{N,Float64}(undef)
    torque = MVector{N,Float64}(undef)
    omega_dot = MVector{N,Float64}(undef)
    imotor = MVector{N,Float64}(undef)
    I_bus_total = 0.0

    @inbounds for i = 1:N
        Ti, Qi, ωdot_i, Ii, Ibus_i = _motorprop_unit_eval(
            p.units[i],
            x.rotor_ω[i],
            duties[i],
            V_bus,
            ρ,
            Vax,
        )
        thrust[i] = Ti
        # Own yaw reaction sign here (single source of truth).
        torque[i] = p.rotor_dir[i] * Qi
        omega_dot[i] = ωdot_i
        imotor[i] = Ii
        I_bus_total += Ibus_i
    end

    if !isfinite(I_bus_total)
        error("non-finite I_bus_total in propulsion eval: I_bus_total=$(I_bus_total) V_bus=$(V_bus) ω=$(x.rotor_ω) duties=$(duties)")
    end

    rot_out = Propulsion.RotorOutput{N}(
        thrust_n = SVector{N,Float64}(thrust),
        shaft_torque_nm = SVector{N,Float64}(torque),
        ω_rad_s = x.rotor_ω,
        motor_current_a = SVector{N,Float64}(imotor),
        bus_current_a = I_bus_total,
    )

    if !isfinite(I_bus_total)
        error(
            "non-finite I_bus_total after rotor evaluation: I_bus_total=$(I_bus_total) V_bus=$(V_bus) ρ=$(ρ) Vax=$(Vax) ω=$(x.rotor_ω) duties=$(duties)",
        )
    end

    return rot_out, SVector{N,Float64}(omega_dot), I_bus_total, V_bus, ρ, v_air_body
end

"""Evaluate algebraic plant outputs at (t, x, u).

This is intended for boundary-time logging and PX4 injection (battery_status). It must be
pure and deterministic.
"""
function plant_outputs(f::PlantDynamicsWithContact, t::Float64, x::PlantState{N}, u::PlantInput) where {N}
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} || error("PlantOutputs currently supports QuadRotorSet{$N} propulsion")

    rot_out, _ωdot, I_bus, V_bus, _ρ, _v_air_body = _eval_propulsion_and_bus(p, f.battery, f.env, t, x, u)
    batt = _battery_status_from_state(f.battery, x.batt_soc, x.batt_v1, I_bus, V_bus)
    return PlantOutputs{N}(
        rotors = rot_out,
        bus_current_a = I_bus,
        bus_voltage_v = V_bus,
        battery_status = batt,
    )
end

@inline function _plant_replace_actuators(
    x::PlantState{N};
    motors_y::SVector{12,Float64} = x.motors_y,
    motors_ydot::SVector{12,Float64} = x.motors_ydot,
    servos_y::SVector{8,Float64} = x.servos_y,
    servos_ydot::SVector{8,Float64} = x.servos_ydot,
) where {N}
    return PlantState{N}(
        rb = x.rb,
        motors_y = motors_y,
        motors_ydot = motors_ydot,
        servos_y = servos_y,
        servos_ydot = servos_ydot,
        rotor_ω = x.rotor_ω,
        batt_soc = x.batt_soc,
        batt_v1 = x.batt_v1,
    )
end

@inline function _set_battery_last_current!(b::Powertrain.IdealBattery, I::Float64, V::Float64)
    b.last_current_a = I
    # Ideal battery stores voltage explicitly.
    b.voltage_v = V
    return nothing
end

@inline function _set_battery_last_current!(b::Powertrain.TheveninBattery, I::Float64, V::Float64)
    b.last_current_a = I
    return nothing
end

# Fallback (unknown battery model): no-op.
@inline function _set_battery_last_current!(::Powertrain.AbstractBatteryModel, I::Float64, V::Float64)
    return nothing
end


function (f::PlantDynamicsWithContact)(t::Float64, x::PlantState{N}, u::PlantInput) where {N}
    # Actuator dynamics (pure; no mutation of actuator model objects).
    my_dot, mydot_dot = _actuator_derivs(f.motor_actuators, x.motors_y, x.motors_ydot, u.cmd.motors)
    sy_dot, sydot_dot = _actuator_derivs(f.servo_actuators, x.servos_y, x.servos_ydot, u.cmd.servos)

    # Propulsion + bus solve.
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} || error("PlantDynamicsWithContact currently supports QuadRotorSet{$N} propulsion")

    rot_out, ω_dot, I_bus, V_bus, _ρ, _v_air_body = _eval_propulsion_and_bus(p, f.battery, f.env, t, x, u)

    # Rigid-body dynamics.
    d_rb = Vehicles.dynamics(f.model, f.env, t, x.rb, rot_out)

    # Contact forces (NED) applied at COM.
    F_contact = contact_force_ned(f.contact, x.rb, t)
    if (F_contact[1] != 0.0) || (F_contact[2] != 0.0) || (F_contact[3] != 0.0)
        d_rb = RigidBodyDeriv(
            pos_dot = d_rb.pos_dot,
            vel_dot = d_rb.vel_dot + F_contact / mass(f.model),
            q_dot = d_rb.q_dot,
            ω_dot = d_rb.ω_dot,
        )
    end

    # Battery dynamics from bus current.
    cap_c = _battery_capacity_c(f.battery)
    I = max(0.0, I_bus)
    soc = x.batt_soc
    soc_dot = -I / cap_c
    if soc <= 0.0 && soc_dot < 0.0
        soc_dot = 0.0
    end
    v1_dot = _battery_v1_dot(f.battery, x.batt_v1, I)

    return PlantDeriv{N}(
        rb = d_rb,
        motors_y_dot = my_dot,
        motors_ydot_dot = mydot_dot,
        servos_y_dot = sy_dot,
        servos_ydot_dot = sydot_dot,
        rotor_ω_dot = ω_dot,
        batt_soc_dot = soc_dot,
        batt_v1_dot = v1_dot,
    )
end

############################
# Simulation instance
############################

mutable struct PlantSimulationInstance{E,V,A,EST,I,S,B,L,C,R,O,PS,D}
    cfg::PlantSimulationConfig
    env::E
    vehicle::V
    autopilot::A
    estimator::EST
    integrator::I
    scenario::S
    battery::B
    log::L
    contact::C

    # Continuous plant state (integrator-owned).
    plant::PS

    # Deterministic RNG streams.
    rng_wind::R
    rng_est::R
    rng_misc::R

    # Timebase.
    t_us::UInt64
    t_s::Float64

    # Stop time (microseconds). Stored so stepping never integrates past end time.
    t_end_us::UInt64

    # Periodic triggers (next event times).
    trig_wind::PeriodicTriggerUs
    trig_ap::PeriodicTriggerUs
    trig_log::PeriodicTriggerUs

    # Sample-and-hold input and PX4 output.
    input::PlantInput
    last_out::Union{Nothing,O}
    ap_uses_time_us::Bool

    # Cached RHS functor (avoids per-step closures).
    dynfun::D
end

@inline time_s(sim::PlantSimulationInstance) = sim.t_s
@inline time_us(sim::PlantSimulationInstance) = sim.t_us

function _wrap_sampled_wind(env::EnvironmentModel, pos_ned::Vec3, t::Float64)
    if env.wind isa SampledWind
        sample_wind!(env.wind, pos_ned, t)
        return env
    end

    sampled = SampledWind(env.wind)
    sample_wind!(sampled, pos_ned, t)
    return EnvironmentModel(
        atmosphere = env.atmosphere,
        wind = sampled,
        gravity = env.gravity,
        origin = env.origin,
    )
end

function PlantSimulationInstance(;
    cfg::PlantSimulationConfig = PlantSimulationConfig(),
    env::EnvironmentModel = EnvironmentModel(),
    vehicle::VehicleInstance,
    autopilot::AbstractAutopilot,
    estimator::AbstractEstimator = TruthEstimator(),
    integrator::AbstractIntegrator,
    scenario::AbstractScenario = ScriptedScenario(),
    battery::AbstractBatteryModel = IdealBattery(),
    log::AbstractLogSink = SimLog(),
    contact::AbstractContactModel = NoContact(),
)
    cfg.t_end >= cfg.t0 || throw(ArgumentError("t_end must be >= t0"))

    # Enforce microsecond-quantized schedules.
    t0_us = _time_to_us(cfg.t0; name = "t0")
    t_end_us = _time_to_us(cfg.t_end; name = "t_end")
    dt_ap_us = _dt_to_us(cfg.dt_autopilot; name = "dt_autopilot")
    dt_wind_us = _dt_to_us(cfg.dt_wind; name = "dt_wind")
    dt_log_us = _dt_to_us(cfg.dt_log; name = "dt_log")

    t_end_us >= t0_us || throw(ArgumentError("t_end must be >= t0"))

    # Guard against conceptually-inconsistent PX4 lockstep cadence.
    max_hz = max_internal_rate_hz(autopilot)
    if max_hz !== nothing
        dt_req = 1.0 / Float64(max_hz)
        if cfg.dt_autopilot > dt_req + 1e-12
            msg = "dt_autopilot is larger than the fastest configured autopilot task period; internal loops will run too slowly"
            if cfg.strict_lockstep_rates
                throw(
                    ArgumentError(
                        "$(msg). Set strict_lockstep_rates=false to override (dt_autopilot=$(cfg.dt_autopilot) s, required_dt=$(dt_req) s for max_task_rate_hz=$(max_hz)).",
                    ),
                )
            else
                @warn msg dt_autopilot=cfg.dt_autopilot required_dt=dt_req max_task_rate_hz=max_hz strict_lockstep_rates=cfg.strict_lockstep_rates
            end
        end
    end

    rng_wind, rng_est, rng_misc = _make_rngs(cfg.seed)

    # Align PX4 home/origin before wrapping wind.
    env = _sync_world_origin(env, autopilot)

    # Hold wind constant over adaptive substeps.
    env = _wrap_sampled_wind(env, vehicle.state.pos_ned, cfg.t0)

    # Preallocate in-memory log capacity.
    if log isa SimLog
        n = Int(floor((cfg.t_end - cfg.t0) / cfg.dt_log)) + 2
        reserve!(log, n)
    end

    # Initialize continuous plant state from legacy mutable components.
    plant0 = init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Initial held wind sample and command.
    wind0 = sample_wind!(env.wind, vehicle.state.pos_ned, cfg.t0)
    input0 = PlantInput(cmd = ActuatorCommand(), wind_ned = wind0)

    # Cache RHS functor (currently throws until implemented).
    dynfun = PlantDynamicsWithContact(
        vehicle.model,
        env,
        contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Autopilot output type for sample-and-hold.
    O = autopilot_output_type(autopilot)
    ap_uses_time_us = hasmethod(
        autopilot_step,
        Tuple{typeof(autopilot),UInt64,Vec3,Vec3,Quat,Vec3,AutopilotCommand},
    )

    trig_wind = _init_trigger(dt_wind_us, t0_us)
    trig_ap = _init_trigger(dt_ap_us, t0_us)
    trig_log = _init_trigger(dt_log_us, t0_us)

    E = typeof(env)
    V = typeof(vehicle)
    A = typeof(autopilot)
    EST = typeof(estimator)
    I = typeof(integrator)
    S = typeof(scenario)
    B = typeof(battery)
    L = typeof(log)
    C = typeof(contact)
    R = typeof(rng_wind)
    PS = typeof(plant0)
    D = typeof(dynfun)

    t0_s = Float64(t0_us) * 1e-6

    return PlantSimulationInstance{E,V,A,EST,I,S,B,L,C,R,O,PS,D}(
        cfg,
        env,
        vehicle,
        autopilot,
        estimator,
        integrator,
        scenario,
        battery,
        log,
        contact,
        plant0,
        rng_wind,
        rng_est,
        rng_misc,
        t0_us,
        t0_s,
        t_end_us,
        trig_wind,
        trig_ap,
        trig_log,
        input0,
        nothing,
        ap_uses_time_us,
        dynfun,
    )
end

############################
# Main stepping (event-driven)
############################

"""Process all due discrete events at the *current* simulation time.

Ordering contract (deterministic):
1) scenario events (may mutate env/vehicle, update high-level command state)
2) wind update (RNG)
3) autopilot tick (scenario command + estimator injection + autopilot step)
4) logging snapshot

Notes
-----
* Scenario one-off events are currently processed at the autopilot tick, which means
  event timing resolution is bounded by `dt_autopilot` unless you make it smaller.
"""
function _process_events_at_current_time!(sim::PlantSimulationInstance)
    # Scenario events (AtTime/When).
    #
    # NOTE: `AtTime` events are treated as true event boundaries by including their
    # next fire time in `step_to_next_event!`. That means when we arrive at exactly
    # `t_fire`, calling `process_events!` here causes the actions to fire at the
    # correct physical time, independent of the autopilot tick cadence.
    process_events!(sim.scenario, sim, sim.t_s)

    # Refresh held wind sample in case scenario events mutated the wind model.
    wind_now = sample_wind!(sim.env.wind, sim.plant.rb.pos_ned, sim.t_s)
    sim.input = PlantInput(cmd = sim.input.cmd, wind_ned = wind_now)

    # Wind update.
    if sim.t_us == sim.trig_wind.next_us
        # Wind OU process updates once per wind tick.
        step_wind!(sim.env.wind, sim.plant.rb.pos_ned, sim.t_s, sim.cfg.dt_wind, sim.rng_wind)
        wind = sample_wind!(sim.env.wind, sim.plant.rb.pos_ned, sim.t_s)
        sim.input = PlantInput(cmd = sim.input.cmd, wind_ned = wind)
        _advance!(sim.trig_wind)
    end

    # Autopilot tick.
    if sim.t_us == sim.trig_ap.next_us
        x = sim.plant.rb

        # Scenario generates high-level command and approximate landed flag.
        cmd, landed = scenario_step(sim.scenario, sim.t_s, x, sim)

        # Battery status injected into PX4 (sampled at the boundary time).
        # Use the algebraic bus solve at the current plant state and held inputs.
        outs = plant_outputs(sim.dynfun, sim.t_s, sim.plant, sim.input)
        batt = outs.battery_status === nothing ? status(sim.battery) : outs.battery_status
        _set_battery_last_current!(sim.battery, batt.current_a, batt.voltage_v)

        # Estimator update (RNG should live *here*, not inside ODE substeps).
        est = estimate!(sim.estimator, sim.rng_est, sim.t_s, x, sim.cfg.dt_autopilot)

        if sim.ap_uses_time_us
            sim.last_out = autopilot_step(
                sim.autopilot,
                sim.t_us,
                est.pos_ned,
                est.vel_ned,
                est.q_bn,
                est.ω_body,
                cmd;
                landed = landed,
                battery = batt,
            )
        else
            sim.last_out = autopilot_step(
                sim.autopilot,
                sim.t_s,
                est.pos_ned,
                est.vel_ned,
                est.q_bn,
                est.ω_body,
                cmd;
                landed = landed,
                battery = batt,
            )
        end

        out = sim.last_out
        out === nothing && error("autopilot produced no output")

        # Convert PX4 outputs to a held actuator command.
        # NOTE: command sanitization logic should match `Simulation.step!`.
        m_raw = getproperty(out, :actuator_motors)
        s_raw = getproperty(out, :actuator_servos)
        motors = SVector{12,Float64}(ntuple(i -> isfinite(Float64(m_raw[i])) ? clamp(Float64(m_raw[i]), 0.0, 1.0) : 0.0, 12))
        servos = SVector{8,Float64}(ntuple(i -> isfinite(Float64(s_raw[i])) ? clamp(Float64(s_raw[i]), -1.0, 1.0) : 0.0, 8))

        sim.input = PlantInput(cmd = ActuatorCommand(motors = motors, servos = servos), wind_ned = sim.input.wind_ned)

        # If using `DirectActuators`, snap the algebraic outputs at the event boundary.
        # For dynamic actuators, keep the integrated states and let the ODE evolve them.
        if sim.vehicle.motor_actuators isa Vehicles.DirectActuators
            sim.plant = _plant_replace_actuators(sim.plant; motors_y = motors, motors_ydot = zero(SVector{12,Float64}))
        end
        if sim.vehicle.servo_actuators isa Vehicles.DirectActuators
            sim.plant = _plant_replace_actuators(sim.plant; servos_y = servos, servos_ydot = zero(SVector{8,Float64}))
        end

        _advance!(sim.trig_ap)
    end

    # Logging.
    if sim.t_us == sim.trig_log.next_us
        if sim.last_out !== nothing
            out = sim.last_out

            # Environment sampled at the log time/state.
            wind = sim.input.wind_ned
            alt_msl_m_log = sim.env.origin.alt_msl_m - sim.plant.rb.pos_ned[3]
            rho = air_density(sim.env.atmosphere, alt_msl_m_log)
            v_air_ned = wind - sim.plant.rb.vel_ned
            v_air_body = quat_rotate_inv(sim.plant.rb.q_bn, v_air_ned)

            # Plant outputs for logging (rotors + bus + battery status).
            outs = plant_outputs(sim.dynfun, sim.t_s, sim.plant, sim.input)
            batt = outs.battery_status === nothing ? status(sim.battery) : outs.battery_status
            _set_battery_last_current!(sim.battery, batt.current_a, batt.voltage_v)

            # PX4 setpoints (if present).
            pos_sp = (
                Float64(getproperty(out, :trajectory_setpoint_position)[1]),
                Float64(getproperty(out, :trajectory_setpoint_position)[2]),
                Float64(getproperty(out, :trajectory_setpoint_position)[3]),
            )
            vel_sp = (
                Float64(getproperty(out, :trajectory_setpoint_velocity)[1]),
                Float64(getproperty(out, :trajectory_setpoint_velocity)[2]),
                Float64(getproperty(out, :trajectory_setpoint_velocity)[3]),
            )
            acc_sp = (
                Float64(getproperty(out, :trajectory_setpoint_acceleration)[1]),
                Float64(getproperty(out, :trajectory_setpoint_acceleration)[2]),
                Float64(getproperty(out, :trajectory_setpoint_acceleration)[3]),
            )
            yaw_sp = Float64(getproperty(out, :trajectory_setpoint_yaw))
            yawspeed_sp = Float64(getproperty(out, :trajectory_setpoint_yawspeed))

            # Convenience rotor logging for quads.
            rotor_ω = (NaN, NaN, NaN, NaN)
            rotor_T = (NaN, NaN, NaN, NaN)
            if length(sim.plant.rotor_ω) >= 4 && outs.rotors !== nothing
                rotor_ω = (
                    sim.plant.rotor_ω[1],
                    sim.plant.rotor_ω[2],
                    sim.plant.rotor_ω[3],
                    sim.plant.rotor_ω[4],
                )
                rotor_T = (
                    outs.rotors.thrust_n[1],
                    outs.rotors.thrust_n[2],
                    outs.rotors.thrust_n[3],
                    outs.rotors.thrust_n[4],
                )
            end

            cmd_log = ActuatorCommand(
                motors = clamp.(sim.plant.motors_y, 0.0, 1.0),
                servos = clamp.(sim.plant.servos_y, -1.0, 1.0),
            )

            log!(
                sim.log,
                sim.t_s,
                sim.plant.rb,
                cmd_log;
                time_us = sim.t_us,
                wind_ned = wind,
                rho = rho,
                air_vel_body = (v_air_body[1], v_air_body[2], v_air_body[3]),
                battery = batt,
                nav_state = Int32(getproperty(out, :nav_state)),
                arming_state = Int32(getproperty(out, :arming_state)),
                mission_seq = Int32(getproperty(out, :mission_seq)),
                mission_count = Int32(getproperty(out, :mission_count)),
                mission_finished = Int32(getproperty(out, :mission_finished)),
                pos_sp = pos_sp,
                vel_sp = vel_sp,
                acc_sp = acc_sp,
                yaw_sp = yaw_sp,
                yawspeed_sp = yawspeed_sp,
                rotor_omega = rotor_ω,
                rotor_thrust = rotor_T,
            )
        end
        _advance!(sim.trig_log)
    end

    return nothing
end

"""Advance simulation to the next discrete event boundary.

This function:
1) processes all events due at the current time (may update `sim.input`), then
2) integrates the plant ODE from `t` to the next event time with `sim.input` held constant.

`PlantDynamicsWithContact` provides the coupled plant RHS used for integration.
"""
function step_to_next_event!(sim::PlantSimulationInstance)
    # 1) Process current-time events (deterministic ordering).
    _process_events_at_current_time!(sim)

    # 2) Find next event time.
    # Include `t_end_us` so we never integrate past the requested stop time.
    # Include the next scenario `AtTime` event so scenario one-offs are true time boundaries.
    next_scn = next_event_us(sim.scenario, sim)
    next_us = if next_scn === nothing
        min(sim.trig_wind.next_us, sim.trig_ap.next_us, sim.trig_log.next_us, sim.t_end_us)
    else
        min(sim.trig_wind.next_us, sim.trig_ap.next_us, sim.trig_log.next_us, sim.t_end_us, next_scn)
    end
    next_us > sim.t_us || error("event scheduler produced non-increasing time")

    dt_us = next_us - sim.t_us
    dt = Float64(dt_us) * 1e-6

    # 3) Integrate plant over (t, t_next) with inputs held constant.
    x0 = sim.plant
    x1 = step_integrator(sim.integrator, sim.dynfun, sim.t_s, x0, sim.input, dt)

    # Contact discontinuity handling (best-effort): if we cross z=0 within this interval,
    # split the interval at an estimated crossing time to reduce deep penetration.
    if !(sim.contact isa NoContact)
        z0 = x0.rb.pos_ned[3]
        z1 = x1.rb.pos_ned[3]
        if z0 <= 0.0 && z1 > 0.0 && dt_us >= UInt64(2)
            # Linear interpolation in z to estimate crossing fraction.
            α = -z0 / (z1 - z0)
            α = clamp(α, 0.0, 1.0)

            dt1_us = UInt64(round(α * Float64(dt_us)))
            # Ensure both sub-intervals are non-zero.
            if dt1_us == 0
                dt1_us = UInt64(1)
            elseif dt1_us >= dt_us
                dt1_us = dt_us - UInt64(1)
            end

            dt1 = Float64(dt1_us) * 1e-6
            dt2 = Float64(dt_us - dt1_us) * 1e-6

            x_mid = step_integrator(sim.integrator, sim.dynfun, sim.t_s, x0, sim.input, dt1)
            x1 = step_integrator(sim.integrator, sim.dynfun, sim.t_s + dt1, x_mid, sim.input, dt2)
        end
    end

    sim.plant = x1

    # Post-step deterministic projections (keep states in physical bounds).
    # These are last-line-of-defense clamps to prevent NaN cascades from occasional RK overshoot.
    ω_clamped = map(w -> max(0.0, w), sim.plant.rotor_ω)
    soc_clamped = clamp(sim.plant.batt_soc, 0.0, 1.0)

    # Actuator output bounds (match lockstep ABI expectations).
    motors_y_clamped = map(u -> clamp(u, 0.0, 1.0), sim.plant.motors_y)
    servos_y_clamped = map(u -> clamp(u, -1.0, 1.0), sim.plant.servos_y)

    # Rate limits for 2nd-order actuators (projection semantics).
    motors_ydot_proj = sim.plant.motors_ydot
    if sim.vehicle.motor_actuators isa Vehicles.SecondOrderActuators
        rl = sim.vehicle.motor_actuators.rate_limit
        if isfinite(rl)
            motors_ydot_proj = map(v -> clamp(v, -rl, rl), motors_ydot_proj)
        end
    else
        motors_ydot_proj = zero(SVector{12,Float64})
    end

    servos_ydot_proj = sim.plant.servos_ydot
    if sim.vehicle.servo_actuators isa Vehicles.SecondOrderActuators
        rl = sim.vehicle.servo_actuators.rate_limit
        if isfinite(rl)
            servos_ydot_proj = map(v -> clamp(v, -rl, rl), servos_ydot_proj)
        end
    else
        servos_ydot_proj = zero(SVector{8,Float64})
    end

    if (ω_clamped != sim.plant.rotor_ω) ||
       (soc_clamped != sim.plant.batt_soc) ||
       (motors_y_clamped != sim.plant.motors_y) ||
       (servos_y_clamped != sim.plant.servos_y) ||
       (motors_ydot_proj != sim.plant.motors_ydot) ||
       (servos_ydot_proj != sim.plant.servos_ydot)
        T = typeof(sim.plant)
        sim.plant = T(
            rb = sim.plant.rb,
            motors_y = motors_y_clamped,
            motors_ydot = motors_ydot_proj,
            servos_y = servos_y_clamped,
            servos_ydot = servos_ydot_proj,
            rotor_ω = ω_clamped,
            batt_soc = soc_clamped,
            batt_v1 = sim.plant.batt_v1,
        )
    end

    # Optional: keep legacy mutable components in sync for tooling/logging.
    sync_components_from_plant!(
        sim.plant,
        sim.vehicle.motor_actuators,
        sim.vehicle.servo_actuators,
        sim.vehicle.propulsion,
        sim.battery,
    )

    # Keep the legacy vehicle state mirror up-to-date.
    sim.vehicle.state = sim.plant.rb

    # Keep time representations consistent by deriving Float64 time from the
    # authoritative integer microsecond clock.
    sim.t_us = next_us
    sim.t_s = Float64(sim.t_us) * 1e-6
    return nothing
end

"""Run until `cfg.t_end`.

This uses event-driven stepping. If you need a fixed-step loop, use `Simulation.run!`.
"""
function run!(sim::PlantSimulationInstance; max_events::Int = typemax(Int), close_log::Bool = true)
    ne = 0
    while sim.t_us < sim.t_end_us && ne < max_events
        step_to_next_event!(sim)
        ne += 1
    end

    # Process any events that are due exactly at the stop time (notably logging).
    if sim.t_us == sim.t_end_us
        _process_events_at_current_time!(sim)
    end

    if close_log
        close!(sim.log)
    end

    return sim
end

end # module PlantSimulation
