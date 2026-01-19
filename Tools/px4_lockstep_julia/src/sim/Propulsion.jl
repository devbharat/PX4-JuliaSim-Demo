"""PX4Lockstep.Sim.Propulsion

Motor / ESC / propeller models.

This module provides the next "high signal" realism layer beyond directly mapping PX4
motor commands to thrust:

    ESC duty  →  motor electrical torque/current  →  rotor speed ω  →  thrust/drag torque

Design goals:
* Deterministic, fixed-step, single-thread friendly.
* Minimal dependencies (only Julia stdlib + StaticArrays).
* Extensible: the same interfaces should support fixed-wing propulsors later.

Notes / intentional simplifications:
* Electrical inductance is ignored (quasi-static current). This is typically fine for
  flight dynamics timescales.
* The propeller is modeled as quadratic in ω (hover-ish). It can be extended with an
  inflow/advance-ratio model when table-based aero is added.
"""
module Propulsion

using ..Types: Vec3
using StaticArrays

export ESCParams,
    BLDCMotorParams,
    QuadraticPropParams,
    MotorPropUnit,
    RotorOutput,
    QuadRotorSet,
    default_iris_quadrotor_set,
    step_propulsion!

############################
# Type hierarchy
############################

"""Abstract motor parameter type."""
abstract type MotorParams end

"""Abstract propeller parameter type."""
abstract type AbstractPropParams end

############################
# Low-level component models
############################

"""ESC model parameters.

The ESC is modeled as an ideal PWM stage (DC-DC) with optional efficiency.

* `η`      : efficiency (0,1]
* `deadzone`: duty below which the ESC outputs 0V (helps avoid tiny idle torques)
"""
Base.@kwdef struct ESCParams
    η::Float64 = 0.98
    deadzone::Float64 = 0.0
end

"""Brushless DC motor parameters (simple quasi-static electrical model).

Units:
* Kv in RPM/V
* R in Ohms
* J in kg*m^2 (rotor inertia)
"""
Base.@kwdef struct BLDCMotorParams <: MotorParams
    Kv_rpm_per_volt::Float64 = 920.0
    R_ohm::Float64 = 0.25
    J_kgm2::Float64 = 1.0e-5
    I0_a::Float64 = 0.5              # no-load current (approx)
    viscous_friction_nm_per_rad_s::Float64 = 1.0e-6
    max_current_a::Float64 = 60.0
end

"""Convert Kv (RPM/V) to Ke (V/(rad/s))."""
@inline function motor_Ke(p::BLDCMotorParams)
    # Ke = 60 / (2π Kv)
    return 60.0 / (2π * p.Kv_rpm_per_volt)
end

"""Torque constant Kt (Nm/A) for an ideal BLDC in SI units."""
@inline motor_Kt(p::BLDCMotorParams) = motor_Ke(p)

"""Quadratic propeller model parameters.

Model:
    T = kT * ρ * ω^2
    Q = kQ * ρ * ω^2

where:
* ω is rad/s
* ρ is air density (kg/m^3)

The ratio kQ/kT has units of meters and corresponds to "torque per thrust" (similar to
the `km` term in many quad models).
"""
Base.@kwdef struct QuadraticPropParams <: AbstractPropParams
    kT::Float64 = 3.0e-6
    kQ::Float64 = 1.5e-7

    # Rotor geometry used for inflow sensitivity (meters).
    radius_m::Float64 = 0.127

    # Inflow sensitivity (dimensionless). 0 disables inflow effects.
    # A simple correction factor is applied: f = 1/(1 + k*mu^2)
    # where mu = Vax / (|ω|*R).
    inflow_kT::Float64 = 8.0
    inflow_kQ::Float64 = 8.0
end

@inline function _inflow_mu(p::QuadraticPropParams, ω::Float64, Vax::Float64)
    tip = max(1e-3, abs(ω) * p.radius_m)
    return Vax / tip
end

@inline function _inflow_factor(k::Float64, mu::Float64)
    k <= 0.0 && return 1.0
    return 1.0 / (1.0 + k * mu * mu)
end

@inline function prop_thrust(p::QuadraticPropParams, ρ::Float64, ω::Float64, Vax::Float64)
    mu = _inflow_mu(p, ω, Vax)
    f = _inflow_factor(p.inflow_kT, mu)
    return p.kT * ρ * ω * ω * f
end

@inline function prop_torque(p::QuadraticPropParams, ρ::Float64, ω::Float64, Vax::Float64)
    mu = _inflow_mu(p, ω, Vax)
    f = _inflow_factor(p.inflow_kQ, mu)
    return p.kQ * ρ * ω * ω * f
end


############################
# Combined unit
############################

"""A combined ESC + motor + propeller unit.

This is a **parameter-only** container. Failure injection is expressed via
`Faults.FaultState` (a bus signal), not by mutating propulsion objects.
"""
struct MotorPropUnit{M<:MotorParams,P<:AbstractPropParams}
    esc::ESCParams
    motor::M
    prop::P
end

function MotorPropUnit(;
    esc::ESCParams = ESCParams(),
    motor::MotorParams = BLDCMotorParams(),
    prop::AbstractPropParams = QuadraticPropParams(),
)
    return MotorPropUnit(esc, motor, prop)
end

"""Per-rotor outputs produced by a propulsion step."""
Base.@kwdef struct RotorOutput{N}
    thrust_n::SVector{N,Float64}
    shaft_torque_nm::SVector{N,Float64}
    ω_rad_s::SVector{N,Float64}
    motor_current_a::SVector{N,Float64}
    bus_current_a::Float64
end

############################
# Quad set (N rotors)
############################

"""A set of N motor/propulsor units for a multirotor.

This is kept separate from the rigid-body model: the vehicle model defines geometry
(rotor positions, directions), and the propulsion set defines the motor/prop dynamics.
"""
mutable struct QuadRotorSet{N}
    units::Vector{MotorPropUnit}   # length N
    rotor_dir::SVector{N,Float64}  # +1/-1 for yaw reaction torque
end

"""A reasonable Iris-like default motor+prop set.

This is *not* meant to be a perfect Iris model; it is a calibrated baseline that:
* produces the correct hover thrust at the previous default hover throttle,
* has plausible motor time constants,
* produces plausible current draw.

Replace these parameters when modeling other airframes.
"""
function default_iris_quadrotor_set(;
    N::Int = 4,
    km_m::Float64 = 0.05,
    V_nom::Float64 = 12.0,
    ρ_nom::Float64 = 1.225,
    thrust_hover_per_rotor_n::Float64 = 3.7,
    rotor_radius_m::Float64 = 0.127,
    inflow_kT::Float64 = 8.0,
    inflow_kQ::Float64 = 8.0,
)

    esc = ESCParams(η = 0.98, deadzone = 0.02)
    motor = BLDCMotorParams(
        Kv_rpm_per_volt = 920.0,
        R_ohm = 0.25,
        J_kgm2 = 1.2e-5,
        I0_a = 0.6,
        viscous_friction_nm_per_rad_s = 2.0e-6,
        max_current_a = 60.0,
    )

    # Calibrate kT so that at ~hover the thrust is reasonable at nominal density.
    prop = _calibrate_quadratic_prop(
        motor,
        V_nom,
        ρ_nom,
        thrust_hover_per_rotor_n*2.0,
        km_m;
        radius_m = rotor_radius_m,
        inflow_kT = inflow_kT,
        inflow_kQ = inflow_kQ,
    )

    units = [MotorPropUnit(esc = esc, motor = motor, prop = prop) for _ = 1:N]
    # Iris typically has alternating directions; use 1,1,-1,-1 as a common pattern.
    rotor_dir =
        N == 4 ? SVector(1.0, 1.0, -1.0, -1.0) :
        SVector{N,Float64}(ntuple(i -> (isodd(i) ? 1.0 : -1.0), N))
    return QuadRotorSet{N}(units, rotor_dir)
end

"""Solve for a QuadraticPropParams that hits a target thrust at duty=1.

Assumptions:
* kQ = km_m * kT
* steady-state ω is from DC motor + quadratic load (closed-form root)

The target is typically set to roughly "2× hover thrust" so full throttle has
headroom.
"""
function _calibrate_quadratic_prop(
    motor::BLDCMotorParams,
    V_nom::Float64,
    ρ_nom::Float64,
    target_thrust_n::Float64,
    km_m::Float64;
    radius_m::Float64 = 0.127,
    inflow_kT::Float64 = 8.0,
    inflow_kQ::Float64 = 8.0,
)::QuadraticPropParams

    Ke = motor_Ke(motor)
    Kt = motor_Kt(motor)
    R = motor.R_ohm
    b = motor.viscous_friction_nm_per_rad_s

    # Compute thrust given kT (with kQ = km*kT) and duty=1 at V_nom.
    function thrust_at_kT(kT::Float64)
        kQ = km_m * kT
        # Solve quadratic: (kQ*ρ) ω^2 + (b + Kt*Ke/R) ω - (Kt*V/R) = 0
        a = kQ * ρ_nom
        bq = b + (Kt*Ke)/R
        c = -(Kt*V_nom)/R
        if a < 1e-12
            ω = max(0.0, -c / bq)
        else
            disc = bq*bq - 4.0*a*c
            ω = max(0.0, (-bq + sqrt(max(0.0, disc))) / (2.0*a))
        end
        return kT * ρ_nom * ω * ω
    end

    # Bisection for kT in a plausible range.
    lo = 1e-8
    hi = 1e-4
    f_lo = thrust_at_kT(lo) - target_thrust_n
    f_hi = thrust_at_kT(hi) - target_thrust_n
    # Expand if needed.
    it = 0
    while f_hi < 0.0 && it < 20
        hi *= 2.0
        f_hi = thrust_at_kT(hi) - target_thrust_n
        it += 1
    end

    for _ = 1:80
        mid = 0.5*(lo+hi)
        f_mid = thrust_at_kT(mid) - target_thrust_n
        if f_mid > 0.0
            hi = mid
        else
            lo = mid
        end
    end

    kT = 0.5*(lo+hi)
    kQ = km_m * kT
    return QuadraticPropParams(
        kT = kT,
        kQ = kQ,
        radius_m = radius_m,
        inflow_kT = inflow_kT,
        inflow_kQ = inflow_kQ,
    )
end

############################
# Stepping
############################

"""Step a single motor/prop unit.

Returns (thrust_N, shaft_torque_Nm, ω_rad_s, motor_current_A, bus_current_A).
"""
@inline function _step_unit(
    u::MotorPropUnit,
    ω::Float64,
    duty::Float64,
    V_bus::Float64,
    ρ::Float64,
    Vax::Float64,
    dt::Float64,
)
    esc = u.esc
    motor = u.motor
    prop = u.prop

    d = clamp(duty, 0.0, 1.0)
    if d < esc.deadzone
        d = 0.0
    end

    # Motor electrical model: current set by applied voltage and back-EMF.
    V_m = d * V_bus
    Ke = motor_Ke(motor)
    Kt = motor_Kt(motor)
    R = motor.R_ohm

    I_m = (V_m - Ke * ω) / R
    I_m = clamp(I_m, 0.0, motor.max_current_a)

    τ_e = Kt * max(0.0, I_m - motor.I0_a)
    τ_load = prop_torque(prop, ρ, ω, Vax)

    b = motor.viscous_friction_nm_per_rad_s
    J = motor.J_kgm2
    ω_dot = (τ_e - τ_load - b * ω) / J

    ω_new = ω + ω_dot * dt
    ω_new = max(0.0, ω_new)

    # Use mid-step ω for thrust/torque outputs to reduce Euler bias.
    ω_mid = 0.5 * (ω + ω_new)
    T = prop_thrust(prop, ρ, ω_mid, Vax)
    Q = prop_torque(prop, ρ, ω_mid, Vax)

    I_bus = (d * I_m) / max(1e-6, esc.η)
    return T, Q, ω_new, I_m, I_bus
end

"""Step a multirotor propulsion set.


Inputs:
* `ω_rad_s` : current rotor speeds (rad/s) per rotor (SVector length N)
* `duties`  : normalized [0..1] per rotor (SVector length N)
* `V_bus`   : battery bus voltage (V)
* `ρ`       : air density at vehicle location (kg/m^3)
* `v_air_body` : relative air velocity in body frame (m/s)
* `dt`      : integration step (s)

Returns a `RotorOutput{N}` with the updated `ω_rad_s`.
"""
function step_propulsion!(
    p::QuadRotorSet{N},
    ω_rad_s::SVector{N,Float64},
    duties::SVector{N,Float64},
    V_bus::Float64,
    ρ::Float64,
    v_air_body::Vec3,
    dt::Float64,
) where {N}
    # Avoid per-tick heap allocations: use stack-friendly static buffers.
    thrust = MVector{N,Float64}(undef)
    torque = MVector{N,Float64}(undef)
    omega = MVector{N,Float64}(undef)
    imotor = MVector{N,Float64}(undef)
    Ibus_total = 0.0

    # Axial inflow along rotor thrust direction (body -Z).
    # Body axes are X fwd, Y right, Z down, so thrust is along -Z and Vax = -v_air_body.z.
    Vax = -Float64(v_air_body[3])

    @inbounds for i = 1:N
        Ti, Qi, ωi, Ii, Ibus =
            _step_unit(p.units[i], ω_rad_s[i], duties[i], V_bus, ρ, Vax, dt)
        thrust[i] = Ti
        # Own yaw reaction torque sign here (single source of truth).
        torque[i] = p.rotor_dir[i] * Qi
        omega[i] = ωi
        imotor[i] = Ii
        Ibus_total += Ibus
    end

    return RotorOutput{N}(
        thrust_n = SVector{N,Float64}(thrust),
        shaft_torque_nm = SVector{N,Float64}(torque),
        ω_rad_s = SVector{N,Float64}(omega),
        motor_current_a = SVector{N,Float64}(imotor),
        bus_current_a = Ibus_total,
    )
end

end # module Propulsion
