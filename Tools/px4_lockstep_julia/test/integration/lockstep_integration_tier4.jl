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
    spec = PX4Lockstep.Tests.Fixtures.iris_spec_for_tests()
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

@testset "Lockstep integration: Tier 4 (short mission sanity)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    spec = _spec_with_lib(t_end_s = 20.0, arm_time_s = 0.5, mission_time_s = 1.0)
    log = Sim.Logging.SimLog()
    Sim.Aircraft.build_engine(spec; mode = :live, log_sinks = log)

    @test !isempty(log.pos_ned)
    alts = [-pos[3] for pos in log.pos_ned]
    @test maximum(alts) > 1.0

    max_ω = 0.0
    any_finite = false
    for ωs in log.rotor_omega_rad_s
        for w in ωs
            if isfinite(w)
                any_finite = true
                max_ω = max(max_ω, abs(w))
            end
        end
    end
    if !any_finite
        @info "No finite rotor omega samples in log"
    end
    @test max_ω > 1.0

    @test log.batt_remaining[end] <= log.batt_remaining[1]
end