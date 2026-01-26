using Test
using StaticArrays
using LinearAlgebra

using PX4Lockstep
const Sim = PX4Lockstep.Sim

const V = Sim.Vehicles
const T = Sim.Types
const RB = Sim.RigidBody

"""Build a minimal 1-rotor GenericMultirotor for unit tests."""
function _test_model_1rotor(; I::T.Mat3, rotor_J::Float64, rotor_dir::Float64 = 1.0)
    N = 1
    params = V.QuadrotorParams{N}(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = inv(I),
        rotor_pos_body = SVector{N,T.Vec3}(T.vec3(0.0, 0.0, 0.0)),
        rotor_axis_body = SVector{N,T.Vec3}(T.vec3(0.0, 0.0, 1.0)),
        rotor_inertia_kgm2 = SVector{N,Float64}(rotor_J),
        rotor_dir = SVector{N,Float64}(rotor_dir),
        linear_drag = 0.0,
        angular_damping = T.vec3(0.0, 0.0, 0.0),
    )
    return V.GenericMultirotor{N}(params)
end

"""Build a minimal RotorOutput{1} for unit tests."""
function _test_rotor_u_1(; thrust::Float64 = 0.0, torque::Float64 = 0.0, ω::Float64 = 0.0, ωdot::Float64 = 0.0)
    N = 1
    return Sim.Propulsion.RotorOutput{N}(
        thrust_n = SVector{N,Float64}(thrust),
        shaft_torque_nm = SVector{N,Float64}(torque),
        ω_rad_s = SVector{N,Float64}(ω),
        ω_dot_rad_s2 = SVector{N,Float64}(ωdot),
        motor_current_a = SVector{N,Float64}(0.0),
        bus_current_a = 0.0,
    )
end


@testset "Vehicles: full inertia tensor + rotor gyroscopic coupling" begin
    env = Sim.Environment.EnvironmentModel()
    wind = T.vec3(0.0, 0.0, 0.0)

    @testset "Full inertia tensor affects ω̇ via ω×(Iω)" begin
        I = @SMatrix [
            0.02 0.001 0.0
            0.001 0.03 0.0
            0.0 0.0 0.04
        ]
        model = _test_model_1rotor(I = I, rotor_J = 0.0)
        x = RB.RigidBodyState(
            pos_ned = T.vec3(0.0, 0.0, 0.0),
            vel_ned = T.vec3(0.0, 0.0, 0.0),
            q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = T.vec3(1.0, 2.0, 3.0),
        )
        u = _test_rotor_u_1()
        d = V.dynamics(model, env, 0.0, x, u, wind)

        ω = x.ω_body
        expected = -inv(I) * cross(ω, I * ω)
        @test isapprox(d.ω_dot, expected; atol = 1e-12, rtol = 0.0)
    end

    @testset "Rotor acceleration produces a body reaction torque (Ḣ term)" begin
        I = @SMatrix [
            0.1 0.0 0.0
            0.0 0.1 0.0
            0.0 0.0 0.2
        ]
        J = 0.01
        rotor_dir = 1.0  # reaction torque sign (+Z for axis=(0,0,1))
        model = _test_model_1rotor(I = I, rotor_J = J, rotor_dir = rotor_dir)
        x = RB.RigidBodyState(
            pos_ned = T.vec3(0.0, 0.0, 0.0),
            vel_ned = T.vec3(0.0, 0.0, 0.0),
            q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = T.vec3(0.0, 0.0, 0.0),
        )
        ωdot_rotor = 100.0
        u = _test_rotor_u_1(ωdot = ωdot_rotor)
        d = V.dynamics(model, env, 0.0, x, u, wind)

        # With ω_body = 0 and τ_ext = 0:
        #   ω̇_body = -I^{-1} Ḣ
        # where rotor spin sign is opposite the reaction torque sign.
        # Ḣ = J * s_spin * ω̇_rotor * axis,  s_spin = -rotor_dir.
        expected = T.vec3(0.0, 0.0, rotor_dir * J * ωdot_rotor / 0.2)
        @test isapprox(d.ω_dot, expected; atol = 1e-12, rtol = 0.0)
    end

    @testset "Body rates couple through rotor angular momentum (ω×H term)" begin
        I = @SMatrix [
            1.0 0.0 0.0
            0.0 1.0 0.0
            0.0 0.0 1.0
        ]
        J = 0.01
        rotor_dir = 1.0
        model = _test_model_1rotor(I = I, rotor_J = J, rotor_dir = rotor_dir)
        x = RB.RigidBodyState(
            pos_ned = T.vec3(0.0, 0.0, 0.0),
            vel_ned = T.vec3(0.0, 0.0, 0.0),
            q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = T.vec3(1.0, 0.0, 0.0),
        )
        ω_rotor = 200.0
        u = _test_rotor_u_1(ω = ω_rotor)
        d = V.dynamics(model, env, 0.0, x, u, wind)

        # H = J * s_spin * ω_rotor * axis, s_spin = -rotor_dir.
        H = T.vec3(0.0, 0.0, -rotor_dir * J * ω_rotor)
        expected = -cross(x.ω_body, H)
        @test isapprox(d.ω_dot, expected; atol = 1e-12, rtol = 0.0)
    end
end
