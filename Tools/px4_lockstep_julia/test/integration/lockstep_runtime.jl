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

@testset "Lockstep runtime (requires libpx4_lockstep)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    @testset "uORB publish visible in same step" begin
        handle = _quiet_create()
        try
            pub, _ = PX4Lockstep.create_publisher(
                handle,
                PX4Lockstep.VehicleAttitudeMsg;
                instance = 0,
            )
            sub = PX4Lockstep.create_subscriber(
                handle,
                PX4Lockstep.VehicleAttitudeMsg;
                instance = 0,
            )

            t = UInt64(1_000)
            q = (1.0f0, 0.0f0, 0.0f0, 0.0f0)
            dq = (0.0f0, 0.0f0, 0.0f0, 0.0f0)
            msg = PX4Lockstep.VehicleAttitudeMsg(
                t,
                t,
                q,
                dq,
                UInt8(0),
                ntuple(_ -> UInt8(0), 7),
            )

            PX4Lockstep.publish!(handle, pub, msg)
            PX4Lockstep.step_uorb!(handle, t)

            @test PX4Lockstep.uorb_check(handle, sub)
            got = PX4Lockstep.uorb_copy(handle, sub)
            @test got.timestamp == t
            @test got.timestamp_sample == t
            PX4Lockstep.uorb_unsubscribe!(handle, sub)
        finally
            PX4Lockstep.destroy(handle)
        end
    end

    @testset "create/destroy stress" begin
        for _ = 1:50
            h = _quiet_create()
            redirect_stdout(devnull) do
                redirect_stderr(devnull) do
                    PX4Lockstep.destroy(h)
                end
            end
        end
        @test true
    end
end
