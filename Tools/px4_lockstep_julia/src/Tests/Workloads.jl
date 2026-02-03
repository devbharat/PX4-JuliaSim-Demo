module Workloads
using ..Fixtures

function precompile_unit()
    Fixtures.tier1_subsystems()
    Fixtures.tier2_fullplant_results()
    Fixtures.compare_integrators_recording_results()
    Fixtures.record_replay_equivalence_log_ticks()
    Fixtures.record_replay_equivalence_wind_disturbance()
    Fixtures.iris_replay_parity_result()
    return nothing
end

end # module Workloads
