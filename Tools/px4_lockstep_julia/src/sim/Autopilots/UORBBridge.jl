using PX4Lockstep: LockstepConfig
using PX4Lockstep: UORBPublisher, UORBSubscriber, UORBMsg
using PX4Lockstep: create_publisher, create_subscriber, publish!
using PX4Lockstep: uorb_check!, uorb_copy!, uorb_unsubscribe!
using PX4Lockstep: BatteryStatusMsg, VehicleAttitudeMsg, VehicleLocalPositionMsg
using PX4Lockstep:
    VehicleGlobalPositionMsg, VehicleAngularVelocityMsg, VehicleLandDetectedMsg
using PX4Lockstep: VehicleStatusMsg, VehicleControlModeMsg, ActuatorArmedMsg
using PX4Lockstep: HomePositionMsg, GeofenceStatusMsg
using PX4Lockstep: VehicleTorqueSetpointMsg, VehicleThrustSetpointMsg
using PX4Lockstep: ActuatorMotorsMsg, ActuatorServosMsg
using PX4Lockstep: VehicleAttitudeSetpointMsg, VehicleRatesSetpointMsg
using PX4Lockstep: MissionResultMsg, TrajectorySetpointMsg

const ZERO_VEC3_F32 = (0.0f0, 0.0f0, 0.0f0)
const ZERO_VEC2_F32 = (0.0f0, 0.0f0)
const ZERO_Q_F32 = (0.0f0, 0.0f0, 0.0f0, 0.0f0)
const NAN_F32 = Float32(NaN)
const ZERO_CELL_V_F32 = (
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
)
const ZERO_PAD_U8 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_1 = (UInt8(0),)
const ZERO_PAD_U8_2 = (UInt8(0), UInt8(0))
const ZERO_PAD_U8_3 = (UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_5 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_6 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_7 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_CONTROLS_8 = ntuple(_ -> 0.0f0, 8)
const ZERO_CONTROLS_12 = ntuple(_ -> 0.0f0, 12)
const NAN_CONTROLS_8 = ntuple(_ -> NAN_F32, 8)
const NAN_CONTROLS_12 = ntuple(_ -> NAN_F32, 12)
const ARMING_STATE_DISARMED = UInt8(1)
const ARMING_STATE_ARMED = UInt8(2)
const NAV_STATE_MANUAL = UInt8(0)
const NAV_STATE_AUTO_MISSION = UInt8(3)
const NAV_STATE_AUTO_RTL = UInt8(5)
const VEHICLE_TYPE_ROTARY_WING = UInt8(1)

include("uorb_bridge/Backend.jl")
include("uorb_bridge/Slots.jl")
include("uorb_bridge/Build.jl")
include("uorb_bridge/Publish.jl")
include("uorb_bridge/Read.jl")
include("uorb_bridge/Conversions.jl")
