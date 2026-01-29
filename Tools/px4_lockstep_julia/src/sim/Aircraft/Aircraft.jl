"""PX4Lockstep.Sim.Aircraft

Aircraft composition / build layer.

This module provides a thin *spec -> engine* builder so that:
- the canonical engine (`Sim.Runtime.Engine`) remains the single run loop
- aircraft configuration is declarative and instance-based

The current builder targets multirotor plant models and applies spec-driven PX4
parameter injection (allocator geometry) on top of the instance + wiring model.
"""
module Aircraft

include("Spec.jl")
include("BatteryAssets.jl")
include("Validate.jl")
include("Build.jl")
include("TOMLIO.jl")

# Long-term preference: TOML/spec is the primary interface; direct Powertrain
# constructors are deprecated in favor of `BatterySpec` + `build_battery`.
export PX4Spec,
    PX4ParamSpec,
    TimelineSpec,
    EnvironmentSpec,
    ScenarioSpec,
    EstimatorSpec,
    PlantSpec,
    AircraftSpec

# Composition specs
export AirframeSpec, PropulsionSpec, EscSpec, MotorSpec
export ActuationSpec, MotorChannelSpec, ServoSpec
export PowerSpec, PowerBusSpec, BatterySpec, BatteryThermalSpec
export AbstractActuatorModelSpec,
    DirectActuatorSpec, FirstOrderActuatorSpec, SecondOrderActuatorSpec
export AbstractSensorSpec, GpsSpec, RangefinderSpec, RadarSpec

export validate_spec, build_engine, build_battery

# Declarative specs
export load_spec, spec_from_toml_dict, run_spec
export default_multirotor_spec_path, default_multirotor_spec

end # module Aircraft
