using Test
using PX4Lockstep

@testset "Workflows.compare_integrators_recording" begin
    res = PX4Lockstep.Tests.Fixtures.compare_integrators_recording_results()
    rows1 = res.rows1

    @test length(rows1) == 2
    @test rows1[1].solver == "euler"
    @test rows1[2].solver == "rk4"
    @test isfinite(rows1[1].pos_max_m)
    @test isfinite(rows1[2].pos_max_m)

    # Euler should be (much) worse than RK4 on position for this piecewise-constant acceleration.
    @test rows1[1].pos_max_m > rows1[2].pos_max_m

    # Determinism sanity (ignore wall_time_ms, which depends on machine load).
    rows2 = res.rows2

    @test rows2[1].solver == rows1[1].solver
    @test rows2[2].solver == rows1[2].solver
    @test isapprox(rows2[1].pos_max_m, rows1[1].pos_max_m; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[2].pos_max_m, rows1[2].pos_max_m; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[1].vel_max_mps, rows1[1].vel_max_mps; atol = 1e-12, rtol = 0.0)
    @test isapprox(rows2[2].vel_max_mps, rows1[2].vel_max_mps; atol = 1e-12, rtol = 0.0)
end
