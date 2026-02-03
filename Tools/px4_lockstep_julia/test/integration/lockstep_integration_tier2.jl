using Test
using PX4Lockstep

const Sim = PX4Lockstep.Sim
const Autopilots = Sim.Autopilots

function _have_lockstep_lib()
    try
        PX4Lockstep.find_library()
        return true
    catch
        return false
    end
end

function _quiet_create()
    cfg = PX4Lockstep.LockstepConfig()
    return redirect_stdout(devnull) do
        redirect_stderr(devnull) do
            PX4Lockstep.create(cfg)
        end
    end
end

function _minimal_uorb_cfg()
    pubs = Autopilots.UORBPubSpec[
        Autopilots.UORBPubSpec(key = :battery_status, type = PX4Lockstep.BatteryStatusMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_attitude, type = PX4Lockstep.VehicleAttitudeMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_local_position, type = PX4Lockstep.VehicleLocalPositionMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_global_position, type = PX4Lockstep.VehicleGlobalPositionMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_angular_velocity, type = PX4Lockstep.VehicleAngularVelocityMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_land_detected, type = PX4Lockstep.VehicleLandDetectedMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_status, type = PX4Lockstep.VehicleStatusMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :vehicle_control_mode, type = PX4Lockstep.VehicleControlModeMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :actuator_armed, type = PX4Lockstep.ActuatorArmedMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :home_position, type = PX4Lockstep.HomePositionMsg, instance = Int32(-1)),
        Autopilots.UORBPubSpec(key = :geofence_status, type = PX4Lockstep.GeofenceStatusMsg, instance = Int32(-1)),
    ]

    subs = Autopilots.UORBSubSpec[
        Autopilots.UORBSubSpec(key = :vehicle_status, type = PX4Lockstep.VehicleStatusMsg, instance = UInt32(0)),
        Autopilots.UORBSubSpec(key = :battery_status, type = PX4Lockstep.BatteryStatusMsg, instance = UInt32(0)),
        Autopilots.UORBSubSpec(key = :actuator_motors, type = PX4Lockstep.ActuatorMotorsMsg, instance = UInt32(0)),
        Autopilots.UORBSubSpec(key = :actuator_servos, type = PX4Lockstep.ActuatorServosMsg, instance = UInt32(0)),
        Autopilots.UORBSubSpec(key = :torque_sp, type = PX4Lockstep.VehicleTorqueSetpointMsg, instance = UInt32(0)),
        Autopilots.UORBSubSpec(key = :thrust_sp, type = PX4Lockstep.VehicleThrustSetpointMsg, instance = UInt32(0)),
    ]

    return Autopilots.PX4UORBInterfaceConfig(pubs = pubs, subs = subs)
end

@testset "Lockstep integration: Tier 2 (PX4 step + bridge sanity)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    cfg = _minimal_uorb_cfg()
    ap = Autopilots.init!(
        config = PX4Lockstep.LockstepConfig(),
        uorb_cfg = cfg,
        edge_trigger = false,
    )

    sub = PX4Lockstep.create_subscriber(
        ap.handle,
        PX4Lockstep.VehicleStatusMsg;
        instance = Int32(0),
    )

    try
        pos = Sim.Types.vec3(0.0, 0.0, 0.0)
        vel = Sim.Types.vec3(0.0, 0.0, 0.0)
        q = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0)
        ω = Sim.Types.vec3(0.0, 0.0, 0.0)
        cmd = Autopilots.AutopilotCommand(armed = true, request_mission = false)
        battery = Sim.Powertrain.BatteryStatus(
            voltage_v = 12.0,
            current_a = 0.0,
            remaining = 1.0,
            warning = 0,
            temperature_c = 25.0,
        )

        time_us = UInt64(0)
        for _ = 1:200
            time_us += UInt64(4_000)
            Autopilots.autopilot_step(
                ap,
                time_us,
                pos,
                vel,
                q,
                ω,
                cmd;
                landed = false,
                battery = battery,
                batteries = [battery],
            )
        end

        updated = Ref{Int32}(0)
        @test PX4Lockstep.uorb_check!(ap.handle, sub, updated)
        msg = Ref{PX4Lockstep.VehicleStatusMsg}()
        PX4Lockstep.uorb_copy!(ap.handle, sub, msg)
        @test msg[].timestamp > 0
        @test ap.uorb_outputs.nav_state >= 0
    finally
        PX4Lockstep.uorb_unsubscribe!(ap.handle, sub)
        Autopilots.close!(ap)
    end
end
