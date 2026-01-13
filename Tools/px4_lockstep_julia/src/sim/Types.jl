"""PX4Lockstep.Sim.Types

Low-allocation math types and helpers used throughout the simulation framework.

Conventions (chosen to match the existing lockstep examples and PX4 SITL defaults):

* World frame is **NED** (x North, y East, z Down).
* Body frame is aircraft body axes.
* Quaternions are stored as `(w, x, y, z)` and represent the rotation **Body → NED**.
"""
module Types

using LinearAlgebra
using StaticArrays

export Vec3,
    Mat3,
    Quat,
    vec3,
    quat_normalize,
    quat_conj,
    quat_mul,
    quat_from_axis_angle,
    quat_to_dcm,
    quat_rotate,
    quat_integrate,
    yaw_from_quat,
    wrap_pi

"""3D vector (Float64) used for NED/body vectors."""
const Vec3 = SVector{3,Float64}

"""3x3 rotation matrix (Float64)."""
const Mat3 = SMatrix{3,3,Float64,9}

"""Quaternion `(w,x,y,z)` representing rotation Body → NED."""
const Quat = SVector{4,Float64}

@inline vec3(x::Real, y::Real, z::Real) = Vec3(Float64(x), Float64(y), Float64(z))

@inline function wrap_pi(θ::Float64)
    # Wrap to (-π, π]
    ϕ = mod(θ + π, 2π)
    return ϕ <= 0 ? (ϕ + π) : (ϕ - π)
end

@inline function quat_normalize(q::Quat)
    n = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2)
    n == 0 && return Quat(1.0, 0.0, 0.0, 0.0)
    return q / n
end

@inline quat_conj(q::Quat) = Quat(q[1], -q[2], -q[3], -q[4])

"""Quaternion product `q ⊗ p` (Hamilton product)."""
@inline function quat_mul(q::Quat, p::Quat)
    qw, qx, qy, qz = q
    pw, px, py, pz = p
    return Quat(
        qw*pw - qx*px - qy*py - qz*pz,
        qw*px + qx*pw + qy*pz - qz*py,
        qw*py - qx*pz + qy*pw + qz*px,
        qw*pz + qx*py - qy*px + qz*pw,
    )
end

"""Quaternion from axis-angle.

`axis` does not need to be unit length.

The returned quaternion is `(w, x, y, z)` and represents the rotation of `angle_rad`
about `axis` (right-hand rule).
"""
@inline function quat_from_axis_angle(axis::Vec3, angle_rad::Float64)
    ax, ay, az = axis
    n = sqrt(ax*ax + ay*ay + az*az)
    if n == 0.0
        return Quat(1.0, 0.0, 0.0, 0.0)
    end
    s = sin(0.5 * angle_rad) / n
    return quat_normalize(Quat(cos(0.5 * angle_rad), ax*s, ay*s, az*s))
end


"""Direction cosine matrix for rotation Body → NED."""
@inline function quat_to_dcm(q::Quat)
    qw, qx, qy, qz = q
    # Matches the formula used in the existing example (row-major literal).
    return @SMatrix [
        1.0 - 2.0*(qy*qy + qz*qz) 2.0*(qx*qy - qw*qz) 2.0*(qx*qz + qw*qy)
        2.0*(qx*qy + qw*qz) 1.0 - 2.0*(qx*qx + qz*qz) 2.0*(qy*qz - qw*qx)
        2.0*(qx*qz - qw*qy) 2.0*(qy*qz + qw*qx) 1.0 - 2.0*(qx*qx + qy*qy)
    ]
end

"""Rotate a body-frame vector into NED using quaternion `q` (Body → NED)."""
@inline quat_rotate(q::Quat, v_body::Vec3) = quat_to_dcm(q) * v_body

"""Yaw angle (rad) from quaternion (Body → NED)."""
@inline function yaw_from_quat(q::Quat)
    qw, qx, qy, qz = q
    return atan(2.0 * (qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
end

"""Integrate quaternion given body rates `ω_body` over `dt` using first-order integration.

This matches the existing example and is used as the primitive inside Euler and RK4.
"""
@inline function quat_integrate(q::Quat, ω_body::Vec3, dt::Float64)
    qw, qx, qy, qz = q
    wx, wy, wz = ω_body
    dq = Quat(
        -0.5 * (qx*wx + qy*wy + qz*wz),
        0.5 * (qw*wx + qy*wz - qz*wy),
        0.5 * (qw*wy - qx*wz + qz*wx),
        0.5 * (qw*wz + qx*wy - qy*wx),
    )
    return quat_normalize(q + dq * dt)
end

end # module Types
