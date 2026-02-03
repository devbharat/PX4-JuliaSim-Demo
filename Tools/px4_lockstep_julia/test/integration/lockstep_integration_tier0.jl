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
    cfg = PX4Lockstep.LockstepConfig()
    return redirect_stdout(devnull) do
        redirect_stderr(devnull) do
            PX4Lockstep.create(cfg)
        end
    end
end

@testset "Lockstep integration: Tier 0 (ABI + contract subset)" begin
    if !_have_lockstep_lib()
        @test_skip "PX4 lockstep library not found; skipping"
        return
    end

    handle = _quiet_create()
    try
        types = [
            PX4Lockstep.BatteryStatusMsg,
            PX4Lockstep.VehicleAttitudeMsg,
            PX4Lockstep.VehicleLocalPositionMsg,
            PX4Lockstep.VehicleStatusMsg,
            PX4Lockstep.VehicleRatesSetpointMsg,
        ]
        PX4Lockstep.verify_uorb_contract!(handle; types = types)
        @test true
    finally
        PX4Lockstep.destroy(handle)
    end
end
