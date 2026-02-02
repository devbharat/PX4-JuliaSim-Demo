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

@testset "Lockstep integration: Tier 5 (long mission regression)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    spec = _spec_with_lib(t_end_s = 120.0, arm_time_s = 0.5, mission_time_s = 1.0)
    log = Sim.Logging.SimLog()
    rec = Sim.Aircraft.build_engine(spec; mode = :record, log_sinks = log)
    plant = Sim.Recording.stream_values(rec.recorder, :plant)
    battery = Sim.Recording.stream_values(rec.recorder, :battery)

    @test !isempty(plant)
    alts = [-ps.rb.pos_ned[3] for ps in plant]
    @test maximum(alts) > 5.0

    @test battery[end].remaining < battery[1].remaining

    finished_idx = findfirst(i -> (log.mission_finished[i] == 1 &&
                                   log.mission_count[i] > 0 &&
                                   log.mission_seq[i] >= (log.mission_count[i] - 1)),
                             eachindex(log.mission_finished))
    @test finished_idx !== nothing
    count0 = log.mission_count[finished_idx]
    @test count0 > 0

    # Mission finished should not flicker before the last item is reached.
    @test all(i -> (log.mission_finished[i] == 0 ||
                    log.mission_count[i] == 0 ||
                    log.mission_seq[i] >= (log.mission_count[i] - 1)),
              eachindex(log.mission_finished))

    # Once mission finishes, it should stay finished for the remainder of the run.
    @test all(i -> log.mission_finished[i] == 1, finished_idx:length(log.mission_finished))

    # Mission should have entered AUTO_MISSION at least once.
    @test any(ns -> ns == 3, log.nav_state)

    # After completion, commander-lite should leave AUTO_MISSION.
    @test log.nav_state[end] == 4

    # Mission should advance through all items by the end.
    max_seq = maximum(log.mission_seq)
    max_count = maximum(log.mission_count)
    @test max_count > 0
    @test max_seq >= max_count - 1

    for ps in plant
        @test all(isfinite, ps.rb.pos_ned)
        @test all(isfinite, ps.rb.vel_ned)
    end
end
