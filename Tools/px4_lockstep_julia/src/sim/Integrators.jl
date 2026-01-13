"""PX4Lockstep.Sim.Integrators

Fixed-step ODE integrators used by the sim engine.

We intentionally keep this lightweight and dependency-free (no DifferentialEquations.jl)
because we want deterministic behavior, easy review, and minimal dependency surface.

Design:

* Integrators operate on a `RigidBodyState` and a dynamics function...
* Control inputs are treated as piecewise constant across a sim step. In closed-loop
  simulation this is the standard assumption: the controller produces a command at the
  start of the interval and the physical system evolves under that command until the next
  tick.

The blog you linked strongly emphasizes solver choice and time step stability; this file
provides Euler (for debugging) and RK4 (default).
"""
module Integrators

using ..RigidBody: RigidBodyState, RigidBodyDeriv, rb_add, rb_scale_add

export AbstractIntegrator, EulerIntegrator, RK4Integrator, step_integrator

abstract type AbstractIntegrator end

"""Forward Euler integrator (1st order).

Useful for debugging but generally not recommended for production sims unless the step size
is very small.
"""
struct EulerIntegrator <: AbstractIntegrator end

"""Runge-Kutta 4th order integrator (fixed-step).

Good accuracy/compute tradeoff for flight dynamics and the common default in many aircraft
simulations.
"""
struct RK4Integrator <: AbstractIntegrator end

"""Advance state one step.

Arguments:
* `integrator` : `EulerIntegrator()` or `RK4Integrator()`
* `f`          : dynamics function `f(t, x, u) -> RigidBodyDeriv`
* `t`          : current time (s)
* `x`          : current state
* `u`          : control input (piecewise-constant over dt)
* `dt`         : step size (s)
"""
@inline function step_integrator(
    ::EulerIntegrator,
    f,
    t::Float64,
    x::RigidBodyState,
    u,
    dt::Float64,
)
    k1 = f(t, x, u)
    return rb_add(x, k1, dt)
end

@inline function step_integrator(
    ::RK4Integrator,
    f,
    t::Float64,
    x::RigidBodyState,
    u,
    dt::Float64,
)
    k1 = f(t, x, u)
    x2 = rb_add(x, k1, 0.5*dt)
    k2 = f(t + 0.5*dt, x2, u)
    x3 = rb_add(x, k2, 0.5*dt)
    k3 = f(t + 0.5*dt, x3, u)
    x4 = rb_add(x, k3, dt)
    k4 = f(t + dt, x4, u)
    return rb_scale_add(x, k1, k2, k3, k4, dt)
end

end # module Integrators
