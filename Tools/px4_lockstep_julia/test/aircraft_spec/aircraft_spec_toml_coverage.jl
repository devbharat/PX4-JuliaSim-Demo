using Test

@testset "AircraftSpec TOML: home + timeline parsing" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [home]
        lat_deg = 1.5
        lon_deg = -2.5
        alt_msl_m = 123.0
        [timeline]
        t_end_s = 12.0
        dt_autopilot_s = 0.004
        dt_wind_s = 0.02
        dt_log_s = 0.01
        dt_phys_s = 0.002
        """
        path = joinpath(dir, "home_timeline.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        @test spec.home.lat_deg == 1.5
        @test spec.home.lon_deg == -2.5
        @test spec.home.alt_msl_m == 123.0
        @test spec.timeline.t_end_s == 12.0
        @test spec.timeline.dt_autopilot_s == 0.004
        @test spec.timeline.dt_wind_s == 0.02
        @test spec.timeline.dt_log_s == 0.01
        @test spec.timeline.dt_phys_s == 0.002
    end
end

@testset "AircraftSpec TOML: plant contact parsing" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [plant.contact]
        kind = "flat_ground_constraint"
        z_slop_m = 0.001
        vz_slop_mps = 0.01
        v_rest_threshold_mps = 0.02
        """
        path = joinpath(dir, "contact.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        contact = spec.plant.contact
        @test contact isa Sim.Contacts.FlatGroundConstraintContact
        @test contact.z_slop_m == 0.001
        @test contact.vz_slop_mps == 0.01
        @test contact.v_rest_threshold_mps == 0.02

        toml_bad = """
        schema_version = 1
        [plant.contact]
        kind = "flat_ground_constraint"
        vz_slop_mps = -0.01
        """
        path_bad = joinpath(dir, "contact_bad.toml")
        write(path_bad, toml_bad)
        @test_throws ErrorException Sim.Aircraft.load_spec(
            path_bad;
            strict = true,
            base_spec = :default,
        )
    end
end

@testset "AircraftSpec TOML: sensors parsing" begin
    mktempdir() do dir
        toml_tbl = """
        schema_version = 1
        [sensors]
        sensors = [{kind="rangefinder", id="rf0", uorb_instance=1}]
        """
        path_tbl = joinpath(dir, "sensors_tbl.toml")
        write(path_tbl, toml_tbl)
        spec_tbl = Sim.Aircraft.load_spec(path_tbl; strict = true, base_spec = :default)
        @test length(spec_tbl.sensors) == 1
        @test spec_tbl.sensors[1] isa Sim.Aircraft.RangefinderSpec
        @test spec_tbl.sensors[1].id == :rf0
        @test spec_tbl.sensors[1].uorb_instance == 1

        toml_arr = """
        schema_version = 1
        [[sensors]]
        kind = "gps"
        id = "gps0"
        uorb_instance = 2
        """
        path_arr = joinpath(dir, "sensors_arr.toml")
        write(path_arr, toml_arr)
        spec_arr = Sim.Aircraft.load_spec(path_arr; strict = true, base_spec = :default)
        @test length(spec_arr.sensors) == 1
        @test spec_arr.sensors[1] isa Sim.Aircraft.GpsSpec
        @test spec_arr.sensors[1].id == :gps0
        @test spec_arr.sensors[1].uorb_instance == 2
    end
end

@testset "AircraftSpec TOML: px4 lockstep + params parsing" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [px4.lockstep]
        enable_commander = 1
        navigator_rate_hz = 50
        [px4]
        params = [
            {name="FOO", value=1},
            {name="BAR", value=1.5},
        ]
        """
        path = joinpath(dir, "px4_lockstep.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        @test spec.px4.lockstep_config.enable_commander == 1
        @test spec.px4.lockstep_config.navigator_rate_hz == 50
        @test length(spec.px4.params) == 2
        @test spec.px4.params[1].name == "FOO"
        @test spec.px4.params[1].value isa Int32
        @test spec.px4.params[2].name == "BAR"
        @test spec.px4.params[2].value isa Float32
    end
end

@testset "AircraftSpec TOML: extends deep-merge semantics" begin
    mktempdir() do dir
        base = """
        schema_version = 1
        [environment]
        wind = "ou"
        wind_tau_s = 2.0
        [actuation]
        [[actuation.motors]]
        id = "m1"
        channel = 1
        [[actuation.motors]]
        id = "m2"
        channel = 2
        """
        child = """
        schema_version = 1
        extends = ["base.toml"]
        [environment]
        wind_tau_s = 5.0
        [actuation]
        [[actuation.motors]]
        id = "m3"
        channel = 3
        """
        base_path = joinpath(dir, "base.toml")
        child_path = joinpath(dir, "child.toml")
        write(base_path, base)
        write(child_path, child)
        spec = Sim.Aircraft.load_spec(child_path; strict = true, base_spec = :default)
        @test spec.environment.wind == :ou
        @test spec.environment.wind_tau_s == 5.0
        @test length(spec.actuation.motors) == 1
        @test spec.actuation.motors[1].id == :m3
    end
end

@testset "AircraftSpec TOML: run_spec overrides [run]" begin
    mktempdir() do dir
        rec_in = joinpath(dir, "rec.jls")
        t0_us = UInt64(0)
        t_end_us = UInt64(1_000)
        axis = Sim.Runtime.TimeAxis(:ap, [t0_us, t_end_us])
        tl = Sim.Runtime.Timeline(
            t0_us,
            t_end_us,
            axis,
            axis,
            axis,
            axis,
            axis,
            axis,
        )
        recorder = Sim.Recording.InMemoryRecorder()
        for t in axis.t_us
            Sim.Recording.record!(recorder, :cmd, t, Sim.Vehicles.ActuatorCommand())
            Sim.Recording.record!(recorder, :wind_base_ned, t, Sim.Types.vec3(0, 0, 0))
            Sim.Recording.record!(recorder, :plant, t, Sim.Plant.PlantState{4,1}())
            Sim.Recording.record!(recorder, :battery, t, 0.0)
            Sim.Recording.record!(recorder, :batteries, t, 0.0)
            Sim.Recording.record!(recorder, :ap_cmd, t, Sim.Autopilots.AutopilotCommand())
            Sim.Recording.record!(recorder, :landed, t, false)
            Sim.Recording.record!(recorder, :faults, t, Sim.Faults.FaultState())
        end
        rec = Sim.Recording.Tier0Recording(
            recorder = recorder,
            timeline = tl,
            plant0 = Sim.Plant.PlantState{4,1}(),
        )
        Sim.Recording.write_recording(rec_in, rec)
        toml = """
        schema_version = 1
        [run]
        mode = "record"
        recording_in = "rec.jls"
        recording_out = "out.jls"
        """
        path = joinpath(dir, "run.toml")
        write(path, toml)

        @test_throws ErrorException Sim.Aircraft.run_spec(path; strict = true, base_spec = :default)

        eng = Sim.Aircraft.run_spec(
            path;
            mode = :replay,
            recording_in = rec_in,
            strict = true,
            base_spec = :default,
        )
        @test eng !== nothing
    end
end

@testset "AircraftSpec TOML: px4 path resolution" begin
    mktempdir() do dir
        mission = joinpath(dir, "mission.waypoints")
        write(mission, "QGC WPL 110\n")
        toml = """
        schema_version = 1
        [px4]
        mission_path = "mission.waypoints"
        """
        path = joinpath(dir, "px4_paths.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        @test spec.px4.mission_path == normpath(mission)
    end
end

@testset "AircraftSpec TOML: plant contact string shorthand" begin
    mktempdir() do dir
        toml = """
        schema_version = 1
        [plant]
        contact = "no_contact"
        """
        path = joinpath(dir, "contact_str.toml")
        write(path, toml)
        spec = Sim.Aircraft.load_spec(path; strict = true, base_spec = :default)
        @test spec.plant.contact isa Sim.Contacts.NoContact
    end
end
