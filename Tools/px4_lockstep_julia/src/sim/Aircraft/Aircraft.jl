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
include("Validate.jl")
include("Build.jl")
include("TOMLIO.jl")

export PX4Spec, PX4ParamSpec, TimelineSpec, PlantSpec, AircraftSpec

# Composition specs
export AirframeSpec, PropulsionSpec
export ActuationSpec, MotorSpec, ServoSpec
export PowerSpec, PowerBusSpec, BatterySpec
export AbstractActuatorModelSpec,
    DirectActuatorSpec, FirstOrderActuatorSpec, SecondOrderActuatorSpec
export AbstractSensorSpec, GpsSpec, RangefinderSpec, RadarSpec

export validate_spec, build_engine

# Declarative specs
export load_spec, spec_from_toml_dict, run_spec
export default_multirotor_spec_path, default_multirotor_spec

end # module Aircraft
