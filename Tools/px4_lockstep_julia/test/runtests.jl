using Test
using Random
using StaticArrays
using PX4Lockstep
const Sim = PX4Lockstep.Sim

@testset "Scheduling.PeriodicTrigger" begin
    trig = Sim.Scheduling.PeriodicTrigger(0.1, 0.0)
    @test Sim.Scheduling.due!(trig, 0.0)
    @test !Sim.Scheduling.due!(trig, 0.05)
    @test Sim.Scheduling.due!(trig, 0.1)
end

@testset "Noise.AR1 deterministic" begin
    rng = MersenneTwister(1234)
    a = Sim.Noise.AR1(1.0, 0.5)
    x1 = Sim.Noise.step!(a, rng, 0.01)
    rng2 = MersenneTwister(1234)
    a2 = Sim.Noise.AR1(1.0, 0.5)
    x2 = Sim.Noise.step!(a2, rng2, 0.01)
    @test x1 == x2
end

@testset "Estimators.NoisyEstimator shape" begin
    rng = MersenneTwister(0)
    est = Sim.Estimators.NoisyEstimator(pos_sigma_m=Sim.Types.vec3(1.0, 2.0, 3.0),
                                        yaw_sigma_rad=0.1,
                                        bias_tau_s=10.0,
                                        pos_bias_sigma_m=Sim.Types.vec3(0.5,0.5,0.5))
    x = Sim.RigidBody.RigidBodyState(
        pos_ned=Sim.Types.vec3(0,0,0),
        vel_ned=Sim.Types.vec3(0,0,0),
        q_bn=Sim.Types.Quat(1.0,0.0,0.0,0.0),
        ω_body=Sim.Types.vec3(0,0,0),
    )
    y = Sim.Estimators.estimate!(est, rng, 0.0, x, 0.01)
    @test length(y.pos_ned) == 3
    @test length(y.q_bn) == 4
end

@testset "Logging.SimLog push" begin
    log = Sim.Logging.SimLog()
    x = Sim.RigidBody.RigidBodyState(
        pos_ned=Sim.Types.vec3(1,2,3),
        vel_ned=Sim.Types.vec3(0.1,0.2,0.3),
        q_bn=Sim.Types.Quat(1.0,0.0,0.0,0.0),
        ω_body=Sim.Types.vec3(0.0,0.0,0.0),
    )
    cmd = Sim.Vehicles.ActuatorCommand(motors=SVector{12,Float64}(fill(0.1, 12)), servos=SVector{8,Float64}(fill(0.0, 8)))
    wind = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Logging.log!(log, 0.0, x, cmd; wind_ned=wind, rho=1.2)
    @test length(log.t) == 1
    @test log.pos_ned[1] == (1.0,2.0,3.0)
end
