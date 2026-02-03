using StaticArrays

using ..Types: Vec3, Quat, quat_rotate_inv, quat_normalize
using ..RigidBody: RigidBodyState, RigidBodyDeriv
using ..Environment: EnvironmentModel, air_density, air_temperature
import ..Vehicles
using ..Vehicles: AbstractVehicleModel, ActuatorCommand, mass
import ..Propulsion
import ..Powertrain
using ..Powertrain: BatteryStatus
using ..Contacts:
    AbstractContactModel,
    NoContact,
    FlatGroundConstraintContact,
    contact_force_ned,
    contact_info
using ..Faults: is_motor_disabled
using ..Plant: PlantState, PlantInput, PlantDeriv, PlantOutputs, PowerState, PowerDeriv
using ..Integrators: step_integrator, reset!

import ..plant_outputs
import ..plant_project
import ..plant_on_autopilot_tick
import ..plant_integrate_interval

# If true, always run the fixed iteration count for cross-platform determinism.
const DETERMINISTIC_SOLVE = true


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

This preserves backwards compatibility with legacy call sites where the
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
    # Preserve legacy single-battery call sites by implicitly constructing a
    # trivial one-bus power network.
    batteries = (battery,)
    net = PowerNetwork{N,1,1}(
        bus_for_motor = SVector{N,Int}(ntuple(_ -> 1, N)),
        bus_for_battery = SVector{1,Int}(1),
        avionics_load_w = SVector{1,Float64}(0.0),
        share_mode = :inv_r0,
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

"""Power-network constructor.

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
