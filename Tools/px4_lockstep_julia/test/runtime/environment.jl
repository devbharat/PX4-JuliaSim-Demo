@testset "Noise.AR1 deterministic" begin
    rng = MersenneTwister(1234)
    a = Sim.Noise.AR1(1.0, 0.5)
    x1 = Sim.Noise.step!(a, rng, 0.01)
    rng2 = MersenneTwister(1234)
    a2 = Sim.Noise.AR1(1.0, 0.5)
    x2 = Sim.Noise.step!(a2, rng2, 0.01)
    @test x1 == x2
end

@testset "Environment.GustStep delegates stepping" begin
    # If mean wind is stateful (OU), stepping a GustStep wrapper should still advance it.
    rng = MersenneTwister(42)
    ou = Sim.Environment.OUWind(mean = Sim.Types.vec3(0, 0, 0), σ = Sim.Types.vec3(1, 0, 0), τ_s = 1.0)
    w = Sim.Environment.GustStep(ou, Sim.Types.vec3(0, 0, 0), 0.0, 1.0)
    pos = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Environment.step_wind!(w, pos, 0.0, 0.1, rng)
    @test ou.v_gust[1] != 0.0
end

@testset "Environment ISA altitude uses origin.alt_msl_m" begin
    atm = Sim.Environment.ISA1976()
    origin = Sim.Types.WorldOrigin(alt_msl_m = 1000.0)
    env = Sim.Environment.EnvironmentModel(atmosphere = atm, origin = origin)
    # At z=0 (at home origin), MSL altitude should be origin alt.
    rho0 = Sim.Environment.air_density(env.atmosphere, env.origin.alt_msl_m - 0.0)
    @test isapprox(rho0, Sim.Environment.air_density(atm, 1000.0); rtol = 1e-12)

    # At z=-100 (100 m above home), MSL altitude should be 1100 m.
    rho1 = Sim.Environment.air_density(env.atmosphere, env.origin.alt_msl_m - (-100.0))
    @test isapprox(rho1, Sim.Environment.air_density(atm, 1100.0); rtol = 1e-12)
end
