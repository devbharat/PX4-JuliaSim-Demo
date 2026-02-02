using Test
using PX4Lockstep

const Sim = PX4Lockstep.Sim

function _have_lockstep_lib()
    try
        PX4Lockstep.find_library()
        return true
    catch
        return false
    end
end

function _spec_with_lib(; t_end_s::Float64, arm_time_s::Float64, mission_time_s::Float64)
    spec = iris_spec_for_tests()
    lib = PX4Lockstep.find_library()
    px4 = Sim.Aircraft.PX4Spec(
        mission_path = spec.px4.mission_path,
        libpath = lib,
        lockstep_config = spec.px4.lockstep_config,
        uorb_cfg = spec.px4.uorb_cfg,
        params = spec.px4.params,
        derive_ca_params = spec.px4.derive_ca_params,
        edge_trigger = spec.px4.edge_trigger,
    )
    timeline = Sim.Aircraft.TimelineSpec(
        t_end_s = t_end_s,
        dt_autopilot_s = spec.timeline.dt_autopilot_s,
        dt_wind_s = spec.timeline.dt_wind_s,
        dt_log_s = spec.timeline.dt_log_s,
        dt_phys_s = spec.timeline.dt_phys_s,
    )
    scenario = Sim.Aircraft.ScenarioSpec(
        arm_time_s = arm_time_s,
        mission_time_s = mission_time_s,
    )
    return Sim.Aircraft.spec_with(spec; px4 = px4, timeline = timeline, scenario = scenario)
end

@testset "Lockstep integration: Tier 3 (record-only smoke)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    spec = _spec_with_lib(t_end_s = 3.0, arm_time_s = 0.3, mission_time_s = 0.6)

    mktempdir() do dir
        rec_path = joinpath(dir, "tier3_recording.jls")
        rec = Sim.Aircraft.build_engine(
            spec;
            mode = :record,
            recording_out = rec_path,
            log_sinks = nothing,
        )
        @test rec isa Sim.Recording.Tier0Recording
        @test isfile(rec_path)
        loaded = Sim.Recording.read_recording(rec_path)
        @test loaded.timeline.t_end_us == rec.timeline.t_end_us
    end
end
