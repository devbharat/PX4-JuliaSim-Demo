using Test
using PX4Lockstep

const Sim = PX4Lockstep.Sim

@testset "UORBBridge: mock backend allocation-free reads" begin
    backend = Sim.Autopilots.MockUORBBackend()
    cfg = Sim.Autopilots.PX4UORBInterfaceConfig(
        subs = [
            Sim.Autopilots.UORBSubSpec(
                key = :torque_sp,
                type = PX4Lockstep.VehicleTorqueSetpointMsg,
                instance = UInt32(0),
            ),
            Sim.Autopilots.UORBSubSpec(
                key = :thrust_sp,
                type = PX4Lockstep.VehicleThrustSetpointMsg,
                instance = UInt32(0),
            ),
        ],
    )

    handle = PX4Lockstep.LockstepHandle(Ptr{Cvoid}(0), Ptr{Cvoid}(0), PX4Lockstep.LockstepConfig())
    bridge = Sim.Autopilots._init_uorb_bridge(handle, cfg; backend = backend)
    out = Sim.Autopilots.UORBOutputs()

    torque = PX4Lockstep.VehicleTorqueSetpointMsg(
        UInt64(0),
        UInt64(0),
        (1.0f0, 2.0f0, 3.0f0),
        (UInt8(0), UInt8(0), UInt8(0), UInt8(0)),
    )
    thrust = PX4Lockstep.VehicleThrustSetpointMsg(
        UInt64(0),
        UInt64(0),
        (0.0f0, 0.0f0, 4.0f0),
        (UInt8(0), UInt8(0), UInt8(0), UInt8(0)),
    )

    Sim.Autopilots.mock_publish!(backend, :torque_sp, torque)
    Sim.Autopilots.mock_publish!(backend, :thrust_sp, thrust)
    _ = Sim.Autopilots._update_uorb_outputs!(bridge, out)
    @test out.actuator_controls[1:4] == (1.0f0, 2.0f0, 3.0f0, 4.0f0)

    # Warm up before allocation measurement.
    Sim.Autopilots.mock_publish!(backend, :torque_sp, torque)
    Sim.Autopilots.mock_publish!(backend, :thrust_sp, thrust)
    _ = Sim.Autopilots._update_uorb_outputs!(bridge, out)

    Sim.Autopilots.mock_publish!(backend, :torque_sp, torque)
    Sim.Autopilots.mock_publish!(backend, :thrust_sp, thrust)
    alloc = @allocated Sim.Autopilots._update_uorb_outputs!(bridge, out)
    @test alloc == 0
end
