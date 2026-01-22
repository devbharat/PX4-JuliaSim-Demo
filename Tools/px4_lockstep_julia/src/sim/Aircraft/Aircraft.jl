"""PX4Lockstep.Sim.Aircraft

Aircraft composition / build layer.

Phase 3 intent
--------------
This module introduces a thin *spec -> engine* builder so that:
- the canonical engine (`Sim.Runtime.Engine`) remains the single run loop
- aircraft configuration can become declarative and instance-based in later phases

In Phase 3, the builder targets the Iris workflow and generic multirotors, with
spec-driven PX4 parameter injection (allocator geometry) layered on top of the
Phase 1 instance + wiring model.
"""
module Aircraft

include("Spec.jl")
include("Validate.jl")
include("Build.jl")

export PX4Spec, PX4ParamSpec, TimelineSpec, PlantSpec, AircraftSpec

# Composition specs (Phase 1)
export AirframeSpec, PropulsionSpec
export ActuationSpec, MotorSpec, ServoSpec
export PowerSpec, PowerBusSpec, BatterySpec
export AbstractActuatorModelSpec, DirectActuatorSpec, FirstOrderActuatorSpec, SecondOrderActuatorSpec
export AbstractSensorSpec, GpsSpec, RangefinderSpec, RadarSpec

export iris_spec, octa_spec, validate_spec, build_engine

end # module Aircraft
