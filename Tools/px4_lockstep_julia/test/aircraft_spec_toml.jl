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

@testset "AircraftSpec TOML inertia parsing" begin
    mktempdir() do dir
        toml_matrix = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [
          [0.02, 0.001, 0.0],
          [0.001, 0.03, 0.0],
          [0.0, 0.0, 0.04],
        ]
        """
        path_matrix = joinpath(dir, "inertia_matrix.toml")
        write(path_matrix, toml_matrix)
        spec = Sim.Aircraft.load_spec(path_matrix; strict = true, base_spec = :default)
        @test spec.airframe.inertia_diag_kgm2 == Sim.Types.vec3(0.02, 0.03, 0.04)
        @test spec.airframe.inertia_products_kgm2 == Sim.Types.vec3(0.001, 0.0, 0.0)

        toml_flat = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [0.02, 0.001, 0.0,
                       0.001, 0.03, 0.0,
                       0.0, 0.0, 0.04]
        """
        path_flat = joinpath(dir, "inertia_flat.toml")
        write(path_flat, toml_flat)
        spec_flat = Sim.Aircraft.load_spec(path_flat; strict = true, base_spec = :default)
        @test spec_flat.airframe.inertia_diag_kgm2 == Sim.Types.vec3(0.02, 0.03, 0.04)
        @test spec_flat.airframe.inertia_products_kgm2 == Sim.Types.vec3(0.001, 0.0, 0.0)

        toml_nonsym = """
        schema_version = 1
        [airframe]
        inertia_kgm2 = [
          [0.02, 0.1, 0.0],
          [0.001, 0.03, 0.0],
          [0.0, 0.0, 0.04],
        ]
        """
        path_nonsym = joinpath(dir, "inertia_nonsym.toml")
        write(path_nonsym, toml_nonsym)
        @test_throws ErrorException Sim.Aircraft.load_spec(
            path_nonsym;
            strict = true,
            base_spec = :default,
        )
    end
end

@testset "AircraftSpec inertia validation (SPD)" begin
    mktempdir() do dir
        toml_bad = """
        schema_version = 1
        [airframe]
        inertia_diag_kgm2 = [1.0, 1.0, 1.0]
        inertia_products_kgm2 = [2.0, 0.0, 0.0]
        """
        path_bad = joinpath(dir, "inertia_bad.toml")
        write(path_bad, toml_bad)
        spec_bad = Sim.Aircraft.load_spec(path_bad; strict = true, base_spec = :default)
        @test_throws ArgumentError Sim.Aircraft.validate_spec(spec_bad)
    end
end

@testset "AircraftSpec integrator table parsing" begin
    mktempdir() do dir
        toml_integrator = """
        schema_version = 1
        [plant.integrator]
        kind = "RK45"
        rtol_pos = 1e-5
        atol_omega = 1e-4
        max_substeps = 12345
        quantize_us = false
        """
        path_integrator = joinpath(dir, "integrator.toml")
        write(path_integrator, toml_integrator)
        spec = Sim.Aircraft.load_spec(path_integrator; strict = true)
        integ = spec.plant.integrator
        @test integ isa Sim.Integrators.RK45Integrator
        @test integ.rtol_pos == 1e-5
        @test integ.atol_ω == 1e-4
        @test integ.max_substeps == 12345
        @test integ.quantize_us == false

        toml_bad_kind = """
        schema_version = 1
        [plant.integrator]
        kind = "RK99"
        """
        path_bad = joinpath(dir, "integrator_bad.toml")
        write(path_bad, toml_bad_kind)
        @test_throws ErrorException Sim.Aircraft.load_spec(path_bad; strict = true)
    end
end
