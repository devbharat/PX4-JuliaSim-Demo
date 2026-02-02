using Test

const Sim = PX4Lockstep.Sim

@testset "Battery: asset path validation" begin
    mktempdir() do dir
        toml_bad = """
        schema_version = 1
        [power]
        [[power.batteries]]
        id = "bat1"
        model = "thevenin"
        pack_asset = "example_8s1p_3290mah"
        """
        path_bad = joinpath(dir, "bad.toml")
        write(path_bad, toml_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_bad; strict = true, base_spec = :default)

        toml_missing = """
        schema_version = 1
        [power]
        [[power.batteries]]
        id = "bat1"
        model = "thevenin"
        pack_asset = "missing_meta.toml"
        """
        path_missing = joinpath(dir, "missing.toml")
        write(path_missing, toml_missing)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_missing; strict = true, base_spec = :default)
    end
end
