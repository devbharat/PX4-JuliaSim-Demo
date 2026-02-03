const Autopilots = Sim.Autopilots
const Types = Sim.Types
const Powertrain = Sim.Powertrain

@testset "UORBBridge conversions arity" begin
    battery = Powertrain.BatteryStatus(
        voltage_v = 11.1,
        current_a = 2.0,
        temperature_c = 30.0,
        remaining = 0.5,
        warning = 1,
    )
    @test Autopilots._battery_status_msg(UInt64(1), battery) isa PX4Lockstep.BatteryStatusMsg

    q = Types.Quat(1.0, 0.0, 0.0, 0.0)
    @test Autopilots._vehicle_attitude_msg(UInt64(1), q) isa PX4Lockstep.VehicleAttitudeMsg

    pos = Types.vec3(1.0, 2.0, -3.0)
    vel = Types.vec3(0.1, 0.2, 0.3)
    @test Autopilots._vehicle_local_position_msg(
        UInt64(1),
        pos,
        vel,
        0.0,
        47.0,
        8.0,
        500.0,
    ) isa PX4Lockstep.VehicleLocalPositionMsg

    @test Autopilots._vehicle_global_position_msg(
        UInt64(1),
        47.0,
        8.0,
        500.0,
    ) isa PX4Lockstep.VehicleGlobalPositionMsg

    @test (
        Autopilots._vehicle_angular_velocity_msg(UInt64(1), vel) isa
        PX4Lockstep.VehicleAngularVelocityMsg
    )

    @test (
        Autopilots._vehicle_land_detected_msg(UInt64(1), true) isa
        PX4Lockstep.VehicleLandDetectedMsg
    )

    @test Autopilots._vehicle_status_msg(
        UInt64(1),
        Autopilots.NAV_STATE_AUTO_MISSION,
        Autopilots.ARMING_STATE_ARMED,
    ) isa PX4Lockstep.VehicleStatusMsg

    cmd = Autopilots.AutopilotCommand(armed = true)
    @test Autopilots._vehicle_control_mode_msg(
        UInt64(1),
        cmd,
        true,
        Autopilots.NAV_STATE_AUTO_MISSION,
        true,
    ) isa PX4Lockstep.VehicleControlModeMsg

    @test (
        Autopilots._actuator_armed_msg(UInt64(1), cmd) isa
        PX4Lockstep.ActuatorArmedMsg
    )

    @test Autopilots._geofence_status_msg(UInt64(1)) isa PX4Lockstep.GeofenceStatusMsg

    @test (
        Autopilots._home_position_msg(UInt64(1), 47.0, 8.0, 500.0, UInt32(1)) isa
        PX4Lockstep.HomePositionMsg
    )
end
