using Test


@testset "AircraftSpec: Iris replay parity (Phase 1)" begin
    @test PX4Lockstep.Tests.Fixtures.iris_replay_parity_result()
end