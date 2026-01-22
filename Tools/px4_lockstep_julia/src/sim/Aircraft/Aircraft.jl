"""PX4Lockstep.Sim.Aircraft

Aircraft composition / build layer.

Phase 1 intent
--------------
This module introduces a thin *spec -> engine* builder so that:
- the canonical engine (`Sim.Runtime.Engine`) remains the single run loop
- aircraft configuration can become declarative and instance-based in later phases

In Phase 1, the builder still targets the Iris workflow, but the spec contains
explicit component instances (motors, batteries, sensors) and simple connection
metadata (power buses), setting the stage for multi-aircraft composition.
"""
module Aircraft

include("Spec.jl")
include("Validate.jl")
include("Build.jl")

export PX4Spec, TimelineSpec, PlantSpec, AircraftSpec

# Composition specs (Phase 1)
export AirframeSpec, PropulsionSpec
export ActuationSpec, MotorSpec, ServoSpec
export PowerSpec, PowerBusSpec, BatterySpec
export AbstractActuatorModelSpec, DirectActuatorSpec, FirstOrderActuatorSpec, SecondOrderActuatorSpec
export AbstractSensorSpec, GpsSpec, RangefinderSpec, RadarSpec

export iris_spec, octa_spec, validate_spec, build_engine

end # module Aircraft
