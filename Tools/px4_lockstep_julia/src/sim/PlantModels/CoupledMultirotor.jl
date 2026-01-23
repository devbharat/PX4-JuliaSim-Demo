"""PlantModels.CoupledMultirotor

Coupled multirotor plant model that is suitable for fixed-step *and* adaptive-step
integration.

This is the canonical full-plant model used by `Sim.Runtime.Engine`.

Included physics (coupled inside the RHS)
----------------------------------------
* actuator dynamics (motor/servo)
* propulsion (motor/ESC electrical torque/current + prop inflow corrections)
* battery Thevenin (SOC + polarization) with **analytic bus solve** + deterministic
  fallback when region assumptions are violated
* rigid-body 6DOF dynamics
* optional contact penalty forces (primarily for in-air + simple ground)

Determinism contract
--------------------
* The RHS must be a pure function of `(t, x, u)`.
* No RNG, no mutation of shared state, no dependence on container iteration order.
* Discrete faults and commands are sample-and-hold via `PlantInput`.

Post-step projection
--------------------
Some state variables have hard physical bounds (e.g. ω ≥ 0, SOC ∈ [0,1]).
The canonical engine calls the `plant_project(model, x)` protocol after each
integrated interval (if implemented) so multiple engines do not duplicate clamp
logic.
"""

using StaticArrays

using ..Types: Vec3, Quat, quat_rotate_inv
using ..RigidBody: RigidBodyDeriv
using ..Environment: EnvironmentModel, air_density, air_temperature
import ..Vehicles
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, mass
import ..Propulsion
import ..Powertrain
using ..Powertrain: BatteryStatus
using ..Contacts: AbstractContactModel, NoContact, contact_force_ned
using ..Faults: is_motor_disabled
using ..Plant: PlantState, PlantInput, PlantDeriv, PlantOutputs, PowerState, PowerDeriv

import ..plant_outputs
import ..plant_project
import ..plant_on_autopilot_tick


"""Coupled multirotor model (RHS functor + algebraic outputs + projection).

This model owns references to the immutable vehicle parameters and the mutable
component objects used elsewhere in the codebase (actuators, propulsion, battery).

The canonical truth during integration is the `PlantState` passed to the RHS; the
mutable component objects are treated as *parameters*.
"""
struct CoupledMultirotorModel{M,E,C,AM,AS,P,BAT,NET,MM,SM}
    model::M
    env::E
    contact::C
    motor_actuators::AM
    servo_actuators::AS
    propulsion::P
    batteries::BAT
    power_net::NET
    motor_map::MM
    servo_map::SM
end

"""Convenience constructor with default identity motor mapping.

This preserves backwards compatibility with the Phase 0/1 call sites where the
physical propulsor index `i` corresponded to PX4 motor output channel `i`.

For configurable airframes, prefer passing an explicit `motor_map` derived from the
aircraft spec.
"""
function CoupledMultirotorModel(
    model,
    env,
    contact,
    motor_actuators,
    servo_actuators,
    propulsion::Propulsion.QuadRotorSet{N},
    battery;
    motor_map::Vehicles.MotorMap{N} = Vehicles.MotorMap{N}(
        SVector{N,Int}(ntuple(i -> i, N)),
    ),
    servo_map = nothing,
) where {N}
    # Phase 5.2: preserve the legacy single-battery call sites by implicitly
    # constructing a trivial one-bus power network.
    batteries = (battery,)
    net = PowerNetwork{N,1,1}(
        bus_for_motor = SVector{N,Int}(ntuple(_ -> 1, N)),
        bus_for_battery = SVector{1,Int}(1),
        avionics_load_w = SVector{1,Float64}(0.0),
        share_mode = :inv_r0,
        primary_bus = 1,
        primary_battery = 1,
    )

    return CoupledMultirotorModel(
        model,
        env,
        contact,
        motor_actuators,
        servo_actuators,
        propulsion,
        batteries,
        net,
        motor_map,
        servo_map,
    )
end

"""Power-network constructor (Phase 5.2).

Prefer using this constructor for multi-battery / multi-bus aircraft. The
single-battery constructor remains for backwards compatibility.
"""
function CoupledMultirotorModel(
    model,
    env,
    contact,
    motor_actuators,
    servo_actuators,
    propulsion::Propulsion.QuadRotorSet{N},
    batteries::NTuple{B,<:Powertrain.AbstractBatteryModel},
    power_net::PowerNetwork{N,B,K};
    motor_map::Vehicles.MotorMap{N} = Vehicles.MotorMap{N}(
        SVector{N,Int}(ntuple(i -> i, N)),
    ),
    servo_map = nothing,
) where {N,B,K}
    return CoupledMultirotorModel(
        model,
        env,
        contact,
        motor_actuators,
        servo_actuators,
        propulsion,
        batteries,
        power_net,
        motor_map,
        servo_map,
    )
end

"""Legacy name retained for incremental migration.

Do not introduce new dependencies on this name; prefer `CoupledMultirotorModel`.
"""
const PlantDynamicsWithContact = CoupledMultirotorModel

export CoupledMultirotorModel, PlantDynamicsWithContact


############################
# Actuator ODE helpers
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
    # * In continuous time this is a differential inclusion; it is approximated by:
    #   - computing dynamics using a clamped effective `ydot` (prevents runaway), and
    #   - projecting `ydot` back into bounds after each accepted integration interval
    #     (via `plant_project`).
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


############################
# Battery model helpers
############################

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
    V_bus::Float64;
    connected::Bool = true,
)::BatteryStatus
    rem = clamp(soc, 0.0, 1.0)
    warn = Powertrain._warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr)
    if !connected
        return BatteryStatus(
            connected = false,
            voltage_v = 0.0,
            current_a = 0.0,
            remaining = rem,
            warning = warn,
        )
    end
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
    V_bus::Float64;
    connected::Bool = true,
)::BatteryStatus
    rem = clamp(soc, 0.0, 1.0)
    warn = Powertrain._warning_from_remaining(rem, b.low_thr, b.crit_thr, b.emerg_thr)
    if !connected
        return BatteryStatus(
            connected = false,
            voltage_v = 0.0,
            current_a = 0.0,
            remaining = rem,
            warning = warn,
        )
    end
    return BatteryStatus(
        connected = true,
        voltage_v = V_bus,
        current_a = I_bus,
        remaining = rem,
        warning = warn,
    )
end


############################
# Motor/prop + electrical coupling helpers
############################

@inline function _motorprop_unit_eval(
    unit::Propulsion.MotorPropUnit,
    ω::Float64,
    duty::Float64,
    V_bus::Float64,
    ρ::Float64,
    Vax::Float64,
)

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
    # The region assumption is validated at the resulting V; if it fails, the caller falls back
    # to the robust region-classified iteration + bisection.
    A = 0.0
    B = 0.0
    any_active = false
    @inbounds for i = 1:N
        unit = p.units[i]

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

        d = clamp(duty[i], 0.0, 1.0)
        if d < unit.esc.deadzone
            continue
        end

        motor = unit.motor
        Ke = Propulsion.motor_Ke(motor)
        R = motor.R_ohm
        Imax = motor.max_current_a

        I_lin = (d * V - Ke * ω[i]) / R
        # Strict inequalities: if the clamp boundaries are hit, the affine model is invalid.
        if !(I_lin > 0.0 && I_lin < Imax)
            return nothing
        end
    end

    return V
end


"""Solve the bus voltage V_bus for the instantaneous (ω, duty, SOC, V1) state.

The solver computes the scalar fixed point implied by the Thevenin+R0 model:

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
        error(
            "non-finite battery parameters in bus solve: ocv=$(ocv) v1=$(v1) R0=$(R0) V_min=$(V_min)",
        )
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

    # Fast path: if all active motors remain in the unsaturated linear region, V_bus can be
    # solved analytically.
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


function _eval_propulsion_and_power_network(
    p::Propulsion.QuadRotorSet{N},
    batteries::NTuple{B,<:Powertrain.AbstractBatteryModel},
    net::PowerNetwork{N,B,K},
    env::EnvironmentModel,
    t::Float64,
    x::PlantState{N,B},
    u::PlantInput,
    motor_map::Vehicles.MotorMap{N},
    axis_b::SVector{N,Vec3},
) where {N,B,K}
    # Atmosphere expects MSL altitude.
    alt_msl_m = env.origin.alt_msl_m - x.rb.pos_ned[3]
    ρ = air_density(env.atmosphere, alt_msl_m)

    # Air-relative velocity in body frame.
    v_air_ned = u.wind_ned - x.rb.vel_ned
    v_air_body = quat_rotate_inv(x.rb.q_bn, v_air_ned)

    # Apply sample-and-hold motor disable faults by forcing duty to 0.
    #
    # Important: motor channel mapping is explicit so that "motor #i" is a physical
    # propulsor index (1..N), not a PX4 output channel.
    fstate = u.faults
    duties_cmd = Vehicles.map_motors(motor_map, x.motors_y)
    duties = SVector{N,Float64}(
        ntuple(i -> (is_motor_disabled(fstate, i) ? 0.0 : clamp(duties_cmd[i], 0.0, 1.0)), N),
    )

    # Phase 5.2: multi-bus, multi-battery algebraic power network.
    #
    # Strategy:
    # 1) Solve one bus voltage per bus using an equivalent Thevenin source
    #    (parallel R0 combination + conductance-weighted average of (OCV - V1)).
    # 2) Evaluate motors at their respective bus voltages to get per-bus load currents.
    # 3) Add per-bus avionics constant-power load current (P/V) and share across
    #    batteries on that bus (simple rule: :inv_r0 or :equal).
    #
    # Note: `avionics_load_w` is treated as constant power for SOC integration.
    # To keep the bus solve monotone/deterministic, we approximate its effect during
    # the solve as a constant current computed at V0 (first-order approximation).

    # Numerical guard for ideal/near-ideal batteries (R0≈0).
    R0_EPS = 1e-6

    V_bus = MVector{K,Float64}(undef)
    if !fstate.battery_connected
        @inbounds for k = 1:K
            V_bus[k] = 0.0
        end
    else
        @inbounds for k = 1:K
            # Mask duties to motors on this bus.
            duties_k = SVector{N,Float64}(
                ntuple(i -> (net.bus_for_motor[i] == k ? duties[i] : 0.0), N),
            )

            # Equivalent source for batteries on this bus.
            w_sum = 0.0
            wv_sum = 0.0
            V_min_bus = 0.0
            any_bat = false
            all_ideal = true
            for bi = 1:B
                if net.bus_for_battery[bi] == k
                    any_bat = true
                    ocv_i = _battery_ocv(batteries[bi], x.power.soc[bi])
                    v1_i = x.power.v1[bi]
                    V0_i = ocv_i - v1_i

                    R0_i = _battery_r0(batteries[bi])
                    all_ideal &= !(isfinite(R0_i) && R0_i > 0.0)
                    R0_eff = (isfinite(R0_i) && R0_i > 0.0) ? R0_i : R0_EPS
                    w = 1.0 / R0_eff
                    w_sum += w
                    wv_sum += w * V0_i

                    V_min_bus = max(V_min_bus, _battery_min_voltage(batteries[bi]))
                end
            end

            if !any_bat || !(isfinite(w_sum) && w_sum > 0.0)
                V_bus[k] = 0.0
                continue
            end

            V0_eq = wv_sum / w_sum
            # If the bus is fed only by ideal (R0≈0) sources, treat the equivalent
            # source as ideal too (exactly no droop).
            R_eq = all_ideal ? 0.0 : (1.0 / w_sum)

            # Approximate avionics draw during the solve as a constant current at V0.
            P_av = net.avionics_load_w[k]
            I_av_assumed = (P_av > 0.0 && V0_eq > 1e-6) ? (P_av / V0_eq) : 0.0
            V0_eff = V0_eq - R_eq * I_av_assumed

            V_k = _solve_bus_voltage(p, x.rotor_ω, duties_k, V0_eff, 0.0, R_eq, V_min_bus)
            if !isfinite(V_k)
                error(
                    "non-finite V_bus[$k] from bus solver: V=$(V_k) V0_eq=$(V0_eq) V0_eff=$(V0_eff) R_eq=$(R_eq) V_min=$(V_min_bus) ω=$(x.rotor_ω) duties=$(duties_k)",
                )
            end
            V_bus[k] = V_k
        end
    end

    thrust = MVector{N,Float64}(undef)
    torque = MVector{N,Float64}(undef)
    omega_dot = MVector{N,Float64}(undef)
    imotor = MVector{N,Float64}(undef)
    I_bus_motors_total = 0.0
    I_bus_motors = MVector{K,Float64}(ntuple(_ -> 0.0, K))

    @inbounds for i = 1:N
        # Axial inflow component along the propulsor thrust direction (Phase 4).
        # Convention: axis_b points along the propulsor axis such that F = -T * axis_b.
        ai = axis_b[i]
        Vax_i = -Float64(v_air_body[1] * ai[1] + v_air_body[2] * ai[2] + v_air_body[3] * ai[3])
        k = net.bus_for_motor[i]
        V_i = (1 <= k <= K) ? V_bus[k] : 0.0
        Ti, Qi, ωdot_i, Ii, Ibus_i =
            _motorprop_unit_eval(p.units[i], x.rotor_ω[i], duties[i], V_i, ρ, Vax_i)
        thrust[i] = Ti
        # Own yaw reaction sign here (single source of truth).
        torque[i] = p.rotor_dir[i] * Qi
        omega_dot[i] = ωdot_i
        imotor[i] = Ii
        I_bus_motors_total += Ibus_i
        if 1 <= k <= K
            I_bus_motors[k] += Ibus_i
        end
    end

    if !isfinite(I_bus_motors_total)
        error(
            "non-finite I_bus_total in propulsion eval: I_bus_total=$(I_bus_motors_total) ω=$(x.rotor_ω) duties=$(duties)",
        )
    end

    rot_out = Propulsion.RotorOutput{N}(
        thrust_n = SVector{N,Float64}(thrust),
        shaft_torque_nm = SVector{N,Float64}(torque),
        ω_rad_s = x.rotor_ω,
        motor_current_a = SVector{N,Float64}(imotor),
        bus_current_a = I_bus_motors_total,
    )

    # Per-bus total current = motors + avionics constant-power load (P/V).
    I_bus_total = MVector{K,Float64}(undef)
    @inbounds for k = 1:K
        V_k = V_bus[k]
        I_av = (fstate.battery_connected && V_k > 1e-6 && net.avionics_load_w[k] > 0.0) ?
               (net.avionics_load_w[k] / V_k) : 0.0
        I_bus_total[k] = I_bus_motors[k] + I_av
    end

    # Share per-bus current draw across batteries on that bus.
    I_batt = MVector{B,Float64}(ntuple(_ -> 0.0, B))
    @inbounds for k = 1:K
        I_k = I_bus_total[k]
        if !(isfinite(I_k) && I_k > 0.0)
            continue
        end

        wsum = 0.0
        for bi = 1:B
            if net.bus_for_battery[bi] == k
                if net.share_mode === :equal
                    wsum += 1.0
                else
                    R0_i = _battery_r0(batteries[bi])
                    R0_eff = (isfinite(R0_i) && R0_i > 0.0) ? R0_i : R0_EPS
                    wsum += 1.0 / R0_eff
                end
            end
        end
        wsum > 0.0 || continue

        for bi = 1:B
            if net.bus_for_battery[bi] == k
                w = if net.share_mode === :equal
                    1.0
                else
                    R0_i = _battery_r0(batteries[bi])
                    R0_eff = (isfinite(R0_i) && R0_i > 0.0) ? R0_i : R0_EPS
                    1.0 / R0_eff
                end
                I_batt[bi] = I_k * w / wsum
            end
        end
    end

    return (
        rot_out,
        SVector{N,Float64}(omega_dot),
        I_bus_motors_total,
        SVector{K,Float64}(V_bus),
        SVector{K,Float64}(I_bus_total),
        SVector{B,Float64}(I_batt),
        ρ,
        v_air_body,
    )
end


############################
# Protocol: algebraic outputs
############################

"""Evaluate algebraic plant outputs at (t, x, u).

Intended for boundary-time logging and PX4 injection (battery_status). Must be pure.
"""
function plant_outputs(
    f::CoupledMultirotorModel,
    t::Float64,
    x::PlantState{N,B},
    u::PlantInput,
) where {N,B}
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} ||
        error("PlantOutputs currently supports QuadRotorSet{$N} propulsion")

    rot_out, _ωdot, _I_motors_total, V_bus, I_bus_total, I_batt, _ρ, _v_air_body =
        _eval_propulsion_and_power_network(
            p,
            f.batteries,
            f.power_net,
            f.env,
            t,
            x,
            u,
            f.motor_map,
            f.model.params.rotor_axis_body,
        )

    # Compute per-battery telemetry (Phase 5.3).
    batt_all = SVector{B,BatteryStatus}(ntuple(i -> begin
        k = f.power_net.bus_for_battery[i]
        V_i = (1 <= k <= length(V_bus)) ? V_bus[k] : 0.0
        I_i = I_batt[i]
        _battery_status_from_state(
            f.batteries[i],
            x.power.soc[i],
            x.power.v1[i],
            I_i,
            V_i;
            connected = u.faults.battery_connected,
        )
    end, B))

    # Legacy outputs/injection: expose a single "primary" bus and battery.
    pb = f.power_net.primary_bus
    pi = f.power_net.primary_battery
    Vp = (1 <= pb <= length(V_bus)) ? V_bus[pb] : 0.0
    Ib = (1 <= pb <= length(I_bus_total)) ? I_bus_total[pb] : 0.0
    batt_primary = (1 <= pi <= length(batt_all)) ? batt_all[pi] : BatteryStatus()

    temp_k = air_temperature(f.env.atmosphere, -x.rb.pos_ned[3])
    return PlantOutputs{N,B}(
        rotors = rot_out,
        bus_current_a = Ib,
        bus_voltage_v = Vp,
        rho_kgm3 = _ρ,
        temp_k = temp_k,
        air_vel_body = _v_air_body,
        battery_status = batt_primary,
        battery_statuses = batt_all,
    )
end


############################
# RHS functor
############################

function (f::CoupledMultirotorModel)(t::Float64, x::PlantState{N,B}, u::PlantInput) where {N,B}
    # Actuator dynamics (pure; no mutation of actuator model objects).
    my_dot, mydot_dot =
        _actuator_derivs(f.motor_actuators, x.motors_y, x.motors_ydot, u.cmd.motors)
    sy_dot, sydot_dot =
        _actuator_derivs(f.servo_actuators, x.servos_y, x.servos_ydot, u.cmd.servos)

    # Propulsion + bus solve.
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} ||
        error("CoupledMultirotorModel currently supports QuadRotorSet{$N} propulsion")

    rot_out, ω_dot, _I_motors_total, _V_bus, _I_bus_total, I_batt, _ρ, _v_air_body =
        _eval_propulsion_and_power_network(
            p,
            f.batteries,
            f.power_net,
            f.env,
            t,
            x,
            u,
            f.motor_map,
            f.model.params.rotor_axis_body,
        )

    # Rigid-body dynamics.
    d_rb = Vehicles.dynamics(f.model, f.env, t, x.rb, rot_out, u.wind_ned)

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

    # Battery dynamics from per-battery current draw (Phase 5.2).
    soc_dot = MVector{B,Float64}(undef)
    v1_dot = MVector{B,Float64}(undef)
    @inbounds for i = 1:B
        cap_c = _battery_capacity_c(f.batteries[i])
        I = max(0.0, I_batt[i])
        sdot = -I / cap_c
        if x.power.soc[i] <= 0.0 && sdot < 0.0
            sdot = 0.0
        end
        soc_dot[i] = sdot
        v1_dot[i] = _battery_v1_dot(f.batteries[i], x.power.v1[i], I)
    end

    power_dot = PowerDeriv{B}(
        soc_dot = SVector{B,Float64}(soc_dot),
        v1_dot = SVector{B,Float64}(v1_dot),
    )

    return PlantDeriv{N,B}(
        rb = d_rb,
        motors_y_dot = my_dot,
        motors_ydot_dot = mydot_dot,
        servos_y_dot = sy_dot,
        servos_ydot_dot = sydot_dot,
        rotor_ω_dot = ω_dot,
        power = power_dot,
    )
end


############################
# Protocol: post-step projection
############################

"""Project plant state into physical bounds.

This is intentionally conservative: it is a last-line-of-defense clamp to prevent
NaN cascades from occasional overshoot in adaptive solvers.

Important: this is not meant to hide modeling bugs. If projection is frequently
active, tighten tolerances or fix the stiffness/discontinuity.
"""
function plant_project(f::CoupledMultirotorModel, x::PlantState{N,B}) where {N,B}
    # Rotor speeds nonnegative.
    ω_clamped = map(w -> max(0.0, w), x.rotor_ω)

    # SOC in [0,1] (vectorized for B batteries).
    soc_clamped = map(s -> clamp(s, 0.0, 1.0), x.power.soc)

    # Actuator outputs in ABI-consistent ranges.
    motors_y_clamped = map(u -> clamp(u, 0.0, 1.0), x.motors_y)
    servos_y_clamped = map(u -> clamp(u, -1.0, 1.0), x.servos_y)

    # Rate limits for 2nd-order actuators (projection semantics).
    motors_ydot_proj = x.motors_ydot
    if f.motor_actuators isa Vehicles.SecondOrderActuators
        rl = f.motor_actuators.rate_limit
        if isfinite(rl)
            motors_ydot_proj = map(v -> clamp(v, -rl, rl), motors_ydot_proj)
        end
    else
        motors_ydot_proj = zero(SVector{12,Float64})
    end

    servos_ydot_proj = x.servos_ydot
    if f.servo_actuators isa Vehicles.SecondOrderActuators
        rl = f.servo_actuators.rate_limit
        if isfinite(rl)
            servos_ydot_proj = map(v -> clamp(v, -rl, rl), servos_ydot_proj)
        end
    else
        servos_ydot_proj = zero(SVector{8,Float64})
    end

    if (ω_clamped != x.rotor_ω) ||
       (soc_clamped != x.power.soc) ||
       (motors_y_clamped != x.motors_y) ||
       (servos_y_clamped != x.servos_y) ||
       (motors_ydot_proj != x.motors_ydot) ||
       (servos_ydot_proj != x.servos_ydot)
        power = PowerState{B}(soc = soc_clamped, v1 = x.power.v1)
        return PlantState{N,B}(
            rb = x.rb,
            motors_y = motors_y_clamped,
            motors_ydot = motors_ydot_proj,
            servos_y = servos_y_clamped,
            servos_ydot = servos_ydot_proj,
            rotor_ω = ω_clamped,
            power = power,
        )
    end

    return x
end


"""Apply boundary-time updates at an autopilot tick.

This is used to keep hybrid semantics correct for `DirectActuators`:
`motors_y/servos_y` are algebraic outputs and must be snapped to the newly
published command at the tick boundary.

For non-direct actuators, this is a no-op.
"""
function plant_on_autopilot_tick(
    f::CoupledMultirotorModel,
    x::PlantState{N,B},
    cmd::ActuatorCommand,
) where {N,B}
    x2 = x

    if f.motor_actuators isa Vehicles.DirectActuators
        x2 = PlantState{N,B}(
            rb = x2.rb,
            motors_y = cmd.motors,
            motors_ydot = zero(SVector{12,Float64}),
            servos_y = x2.servos_y,
            servos_ydot = x2.servos_ydot,
            rotor_ω = x2.rotor_ω,
            power = x2.power,
        )
    end

    if f.servo_actuators isa Vehicles.DirectActuators
        x2 = PlantState{N,B}(
            rb = x2.rb,
            motors_y = x2.motors_y,
            motors_ydot = x2.motors_ydot,
            servos_y = cmd.servos,
            servos_ydot = zero(SVector{8,Float64}),
            rotor_ω = x2.rotor_ω,
            power = x2.power,
        )
    end

    return x2
end
