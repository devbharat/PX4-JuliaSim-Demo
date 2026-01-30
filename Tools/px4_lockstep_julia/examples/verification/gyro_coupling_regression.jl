using PX4Lockstep
using StaticArrays
using LinearAlgebra

const Sim = PX4Lockstep.Sim
const V = Sim.Vehicles
const T = Sim.Types
const RB = Sim.RigidBody
const Env = Sim.Environment

function build_model(; rotor_axis::T.Vec3, rotor_dir::Float64, rotor_J::Float64, I::T.Mat3)
    params = V.QuadrotorParams(
        mass = 1.0,
        inertia_kgm2 = I,
        inertia_inv_kgm2 = inv(I),
        rotor_pos_body = SVector{1,T.Vec3}(T.vec3(0.0, 0.0, 0.0)),
        rotor_axis_body = SVector{1,T.Vec3}(rotor_axis),
        rotor_inertia_kgm2 = SVector{1,Float64}(rotor_J),
        rotor_dir = SVector{1,Float64}(rotor_dir),
        linear_drag = 0.0,
        angular_damping = T.vec3(0.0, 0.0, 0.0),
    )
    return V.GenericMultirotor{1}(params)
end

function build_rotor_u(; omega::Float64)
    return Sim.Propulsion.RotorOutput{1}(
        thrust_n = SVector{1,Float64}(0.0),
        shaft_torque_nm = SVector{1,Float64}(0.0),
        ω_rad_s = SVector{1,Float64}(omega),
        ω_dot_rad_s2 = SVector{1,Float64}(0.0),
        motor_current_a = SVector{1,Float64}(0.0),
        bus_current_a = 0.0,
    )
end

function main()
    I = @SMatrix [
        1.0 0.0 0.0
        0.0 1.0 0.0
        0.0 0.0 1.0
    ]
    rotor_axis = T.vec3(0.0, 1.0, 0.0)
    rotor_J = 0.01
    rotor_dir = 1.0
    omega_rotor = 200.0

    model = build_model(
        rotor_axis = rotor_axis,
        rotor_dir = rotor_dir,
        rotor_J = rotor_J,
        I = I,
    )
    env = Env.EnvironmentModel()
    wind = T.vec3(0.0, 0.0, 0.0)

    x = RB.RigidBodyState(
        pos_ned = T.vec3(0.0, 0.0, 0.0),
        vel_ned = T.vec3(0.0, 0.0, 0.0),
        q_bn = T.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = T.vec3(1.0, 0.0, 0.0),
    )
    u = build_rotor_u(omega = omega_rotor)
    d = V.dynamics(model, env, 0.0, x, u, wind)

    expected = T.vec3(0.0, 0.0, rotor_J * omega_rotor)

    out_dir = joinpath(@__DIR__, "out")
    mkpath(out_dir)
    out_path = joinpath(out_dir, "gyro_coupling.csv")
    open(out_path, "w") do io
        println(io, "component,expected,simulated")
        println(io, "x,$(expected[1]),$(d.ω_dot[1])")
        println(io, "y,$(expected[2]),$(d.ω_dot[2])")
        println(io, "z,$(expected[3]),$(d.ω_dot[3])")
    end
    println("Wrote $(out_path)")
end

main()
