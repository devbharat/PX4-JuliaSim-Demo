@testset "PX4Lockstep ABI handshake helper" begin
    # Should pass with self-reported expectations.
    PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(0),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION + UInt32(1),
        UInt32(0),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(4),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )
end

@testset "Vehicles.FirstOrderActuators exact discretization" begin
    a = Sim.Vehicles.FirstOrderActuators{1}(τ = 0.1, y0 = SVector{1,Float64}(0.0))
    u = SVector{1,Float64}(1.0)
    y = Sim.Vehicles.step_actuators!(a, u, 0.05)
    @test isapprox(y[1], 1.0 - exp(-0.05 / 0.1); atol = 1e-12)
end

@testset "Logging.SimLog push" begin
    log = Sim.Logging.SimLog()
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1, 2, 3),
        vel_ned = Sim.Types.vec3(0.1, 0.2, 0.3),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    cmd = Sim.Vehicles.ActuatorCommand(
        motors = SVector{12,Float64}(fill(0.1, 12)),
        servos = SVector{8,Float64}(fill(0.0, 8)),
    )
    wind = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Logging.log!(log, 0.0, x, cmd; wind_ned = wind, rho = 1.2)
    @test length(log.t) == 1
    @test log.time_us[1] == UInt64(0)
    @test log.pos_ned[1] == (1.0, 2.0, 3.0)
end
