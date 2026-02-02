using Test
using PX4Lockstep

function _have_lockstep_lib()
    try
        PX4Lockstep.find_library()
        return true
    catch
        return false
    end
end

function _quiet_create()
    cfg = PX4Lockstep.LockstepConfig(enable_control_allocator = 0)
    return redirect_stdout(devnull) do
        redirect_stderr(devnull) do
            PX4Lockstep.create(cfg)
        end
    end
end

@testset "Lockstep integration: Tier 1 (uORB round-trip)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    handle = _quiet_create()
    try
        pub, inst = PX4Lockstep.create_publisher(
            handle,
            PX4Lockstep.VehicleAttitudeMsg;
            instance = 0,
        )
        sub = PX4Lockstep.create_subscriber(
            handle,
            PX4Lockstep.VehicleAttitudeMsg;
            instance = Int32(inst),
        )

        t = UInt64(1_000)
        msg = PX4Lockstep.VehicleAttitudeMsg(
            t,
            t,
            (1.0f0, 0.0f0, 0.0f0, 0.0f0),
            (0.0f0, 0.0f0, 0.0f0, 0.0f0),
            UInt8(0),
            ntuple(_ -> UInt8(0), 7),
        )

        PX4Lockstep.publish!(handle, pub, msg)
        PX4Lockstep.step_uorb!(handle, t)

        updated = Ref{Int32}(0)
        @test PX4Lockstep.uorb_check!(handle, sub, updated)
        buf = Ref{PX4Lockstep.VehicleAttitudeMsg}()
        PX4Lockstep.uorb_copy!(handle, sub, buf)
        @test buf[].timestamp == t
        @test buf[].timestamp_sample == t

        pub2, inst2 = PX4Lockstep.create_publisher(
            handle,
            PX4Lockstep.VehicleAttitudeMsg;
            instance = 1,
        )
        sub2 = PX4Lockstep.create_subscriber(
            handle,
            PX4Lockstep.VehicleAttitudeMsg;
            instance = Int32(inst2),
        )

        t2 = UInt64(2_000)
        msg2 = PX4Lockstep.VehicleAttitudeMsg(
            t2,
            t2,
            (0.0f0, 1.0f0, 0.0f0, 0.0f0),
            (0.0f0, 0.0f0, 0.0f0, 0.0f0),
            UInt8(0),
            ntuple(_ -> UInt8(0), 7),
        )
        PX4Lockstep.publish!(handle, pub2, msg2)
        PX4Lockstep.step_uorb!(handle, t2)

        updated2 = Ref{Int32}(0)
        @test PX4Lockstep.uorb_check!(handle, sub2, updated2)
        buf2 = Ref{PX4Lockstep.VehicleAttitudeMsg}()
        PX4Lockstep.uorb_copy!(handle, sub2, buf2)
        @test buf2[].timestamp == t2

        PX4Lockstep.uorb_unsubscribe!(handle, sub)
        PX4Lockstep.uorb_unsubscribe!(handle, sub2)
    finally
        PX4Lockstep.destroy(handle)
    end
end
