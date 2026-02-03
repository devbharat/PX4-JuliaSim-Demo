"""Return the geodesic rotation error (rad) between two quaternions.

Quaternions are treated as equivalent up to sign (q == -q).
"""
function quat_angle_error(q::Sim.Types.Quat, q_ref::Sim.Types.Quat)
    d = abs(sum(q .* q_ref))
    d = clamp(d, 0.0, 1.0)
    return 2.0 * acos(d)
end

struct ZeroRB end

function (f::ZeroRB)(t::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
    return Sim.RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
end
