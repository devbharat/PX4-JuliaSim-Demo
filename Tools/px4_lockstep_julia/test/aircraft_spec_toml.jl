using Test

@testset "AircraftSpec TOML uORB instance validation" begin
    mktempdir() do dir
        toml_pub_ok = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -1
        """
        path_pub_ok = joinpath(dir, "pub_ok.toml")
        write(path_pub_ok, toml_pub_ok)
        _ = Sim.Aircraft.load_spec(path_pub_ok; strict = true)

        toml_pub_default = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        """
        path_pub_default = joinpath(dir, "pub_default.toml")
        write(path_pub_default, toml_pub_default)
        _ = Sim.Aircraft.load_spec(path_pub_default; strict = true)

        toml_pub_bad = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.pubs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -2
        """
        path_pub_bad = joinpath(dir, "pub_bad.toml")
        write(path_pub_bad, toml_pub_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_pub_bad; strict = true)

        toml_sub_bad = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.subs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        instance = -1
        """
        path_sub_bad = joinpath(dir, "sub_bad.toml")
        write(path_sub_bad, toml_sub_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_sub_bad; strict = true)

        toml_sub_default = """
        schema_version = 1
        [px4.uorb]
        [[px4.uorb.subs]]
        key = "battery_status"
        type = "BatteryStatusMsg"
        """
        path_sub_default = joinpath(dir, "sub_default.toml")
        write(path_sub_default, toml_sub_default)
        _ = Sim.Aircraft.load_spec(path_sub_default; strict = true)
    end
end
