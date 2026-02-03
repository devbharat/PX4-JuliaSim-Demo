using Test
using Random
using StaticArrays
using LinearAlgebra
using PX4Lockstep

const Sim = PX4Lockstep.Sim
const Workflows = PX4Lockstep.Workflows

# TOML-first Iris helpers for tests.
include("_helpers/_iris_helpers.jl")
include("_helpers/_test_helpers.jl")

# Optional test sharding to reduce wall time in CI / local iteration.
#
# - PX4 lockstep integration tests cannot run concurrently in a single process
#   because libpx4_lockstep is not re-entrant (one handle per process). To
#   parallelize safely, run them in a separate Julia process.
#
# Usage:
#   PX4_LOCKSTEP_TEST_GROUP=unit         julia --project=... test/runtests.jl
#   PX4_LOCKSTEP_TEST_GROUP=integration  julia --project=... test/runtests.jl
#   PX4_LOCKSTEP_UNIT_GROUP=verification,runtime  julia --project=... test/runtests.jl
#   (default: "all")
const PX4_LOCKSTEP_TEST_GROUP = get(ENV, "PX4_LOCKSTEP_TEST_GROUP", "all")
const RUN_UNIT = PX4_LOCKSTEP_TEST_GROUP in ("all", "unit")
const RUN_INTEGRATION = PX4_LOCKSTEP_TEST_GROUP in ("all", "integration")

if !(RUN_UNIT || RUN_INTEGRATION)
    error("Unknown PX4_LOCKSTEP_TEST_GROUP=$(PX4_LOCKSTEP_TEST_GROUP). Expected: all|unit|integration.")
end

const _INTEGRATION_FILTER_RAW = lowercase(get(ENV, "PX4_LOCKSTEP_INTEGRATION_FILTER", "all"))
const _INTEGRATION_ALL = _INTEGRATION_FILTER_RAW == "all"
const _INTEGRATION_FILTER = _INTEGRATION_ALL ? Set{String}() :
    Set(filter(!isempty, strip.(split(_INTEGRATION_FILTER_RAW, ","))))

function _run_integration_tier(name::String)
    return _INTEGRATION_ALL || (name in _INTEGRATION_FILTER)
end

if RUN_INTEGRATION && !_INTEGRATION_ALL
    allowed = Set(["tier0", "tier1", "tier2", "tier3", "tier4", "tier5"])
    unknown = setdiff(_INTEGRATION_FILTER, allowed)
    isempty(unknown) || error("Unknown PX4_LOCKSTEP_INTEGRATION_FILTER entries: $(collect(unknown)). Expected: $(collect(allowed)) or 'all'.")
end

const _UNIT_GROUP_RAW = lowercase(get(ENV, "PX4_LOCKSTEP_UNIT_GROUP", "all"))
const _UNIT_ALL = _UNIT_GROUP_RAW == "all"
const _UNIT_FILTER = _UNIT_ALL ? Set{String}() :
    Set(filter(!isempty, strip.(split(_UNIT_GROUP_RAW, ","))))

function _run_unit_group(name::String)
    return _UNIT_ALL || (name in _UNIT_FILTER)
end

if RUN_UNIT && !_UNIT_ALL
    allowed = Set(["verification", "runtime", "aircraft", "battery"])
    unknown = setdiff(_UNIT_FILTER, allowed)
    isempty(unknown) || error("Unknown PX4_LOCKSTEP_UNIT_GROUP entries: $(collect(unknown)). Expected: $(collect(allowed)) or 'all'.")
end

const RUN_UNIT_VERIFICATION = RUN_UNIT && _run_unit_group("verification")
const RUN_UNIT_RUNTIME = RUN_UNIT && _run_unit_group("runtime")
const RUN_UNIT_AIRCRAFT = RUN_UNIT && _run_unit_group("aircraft")
const RUN_UNIT_BATTERY = RUN_UNIT && _run_unit_group("battery")

if RUN_UNIT_VERIFICATION
    # Verification cases (analytic + invariants). Keep in a separate file so the
    # main test entrypoint stays readable.
    include("verification/verification_cases.jl")

    # Verification: system-level contracts + missing subsystem unit coverage.
    # These start as `@test_skip` shells and will be filled in incrementally.
    include("verification/verification_contracts.jl")

    include("verification/integrators.jl")
    include("verification/analytic.jl")
end

if RUN_UNIT_RUNTIME
    # Ground contact regression harness (Phase 0/1).
    include("contact/ground_contact_harness.jl")

    # uORB interface + injection scheduling checks (no PX4 binary required).
    include("uorb/uorb_injection.jl")
    include("uorb/uorb_bridge_allocs.jl")

    # Compare-integrators workflow (record/replay + metrics)
    include("verification/compare_integrators.jl")
    include("verification/record_replay_engine.jl")

    include("runtime/environment.jl")
    include("runtime/scheduler.jl")
    include("runtime/engine.jl")
    include("runtime/misc.jl")
    include("runtime/propulsion.jl")
    include("runtime/strict_lockstep_rates.jl")
end

if RUN_UNIT_AIRCRAFT
    # AircraftSpec scaffolding (Phase 0) checks.
    include("aircraft_spec/aircraft_spec_iris_parity.jl")
    include("aircraft_spec/aircraft_spec_toml.jl")
    include("aircraft_spec/aircraft_spec_toml_coverage.jl")

    # Phase 2: actuator mapping + generic multirotor counts (no PX4 required).
    include("verification/multirotor_motor_map.jl")

    # Vehicle dynamics unit coverage (inertia tensor + rotor gyroscopic coupling).
    include("verification/vehicles_inertia_gyro.jl")

    include("aircraft/plantmodels_bus_voltage.jl")
end

if RUN_UNIT_BATTERY
    # Phase 5: DCIR-driven loaded voltage sag regression (Example pack).
    include("battery/battery_loaded_voltage_dcir.jl")

    # Phase 4: cell asset pack scaling laws.
    include("battery/battery_pack_scaling.jl")

    # Phase 6: battery asset validation + scaling/sanity checks.
    include("battery/battery_assets_validation.jl")

    # Phase 7: thermal hook regression.
    include("battery/battery_thermal_hook.jl")
    include("battery/battery_r0_overhead.jl")
    include("battery/battery_surface_scaling.jl")
    include("battery/battery_rc_scaling.jl")
    include("battery/battery_min_voltage_scaling.jl")
    include("battery/battery_surface_units.jl")
    include("battery/battery_ocv_csv_cell_scaling.jl")
    include("battery/battery_ocv_resample.jl")
    include("battery/battery_asset_path_validation.jl")
    include("battery/battery_thermal_cooling.jl")
    include("battery/battery_temperature_clamp.jl")
end

if RUN_INTEGRATION
    if _run_integration_tier("tier0")
        include("integration/lockstep_runtime.jl")
        include("integration/lockstep_integration_tier0.jl")
    end
    if _run_integration_tier("tier1")
        include("integration/lockstep_integration_tier1.jl")
    end
    if _run_integration_tier("tier2")
        include("integration/lockstep_integration_tier2.jl")
    end
    if _run_integration_tier("tier3")
        include("integration/lockstep_integration_tier3.jl")
    end
    if _run_integration_tier("tier4")
        include("integration/lockstep_integration_tier4.jl")
    end
    if _run_integration_tier("tier5")
        include("integration/lockstep_integration_tier5.jl")
    end
end
