const Autopilots = Sim.Autopilots

@testset "uORB traits and injection scheduling" begin
    @test isdefined(PX4Lockstep, :UORB_ALL_TYPES)
    @test length(PX4Lockstep.UORB_ALL_TYPES) > 0

    T = PX4Lockstep.BatteryStatusMsg
    @test PX4Lockstep.uorb_topic(T) == "battery_status"
    @test PX4Lockstep.uorb_queue_length(T) >= 1
    fields = PX4Lockstep.uorb_fields(T)
    @test !isempty(fields)
    @test PX4Lockstep.uorb_fields_hash(T) == PX4Lockstep.uorb_fields_hash_runtime(fields)
    @test PX4Lockstep.uorb_message_hash(T) != 0

    @test Autopilots._is_due(UInt64(10), UInt64(10), UInt64(0), UInt64(0))
    @test !Autopilots._is_due(UInt64(10), UInt64(10), UInt64(50), UInt64(20))
    @test Autopilots._is_due(UInt64(30), UInt64(10), UInt64(50), UInt64(20))
    @test Autopilots._is_due(UInt64(80), UInt64(10), UInt64(50), UInt64(20))

    struct DummyInjection <: Autopilots.AbstractUORBInjectionSource
        period_us::UInt64
    end

    Autopilots.uorb_period_us(src::DummyInjection) = src.period_us
    Autopilots.inject_due!(::DummyInjection, ::PX4Lockstep.LockstepHandle, ::Autopilots.PX4StepContext, ::UInt64) =
        nothing

    inj = Autopilots.PX4UORBInjector()
    Autopilots.add_source!(inj, DummyInjection(UInt64(100)))
    Autopilots.add_source!(inj, DummyInjection(UInt64(0)))
    Autopilots.add_source!(inj, DummyInjection(UInt64(50)))

    @test sort(Autopilots.injection_periods_us(inj)) == UInt64[50, 100]
    @test Autopilots.recommended_step_dt_us(inj) == UInt64(50)

    dummy_handle = PX4Lockstep.LockstepHandle(Ptr{Cvoid}(0), Ptr{Cvoid}(0), PX4Lockstep.LockstepConfig())
    pubs = Dict{Symbol,PX4Lockstep.UORBPublisher}(
        :battery_status =>
            PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(0), UInt32(sizeof(T))),
        :home_position =>
            PX4Lockstep.UORBPublisher{PX4Lockstep.HomePositionMsg}(
                Int32(1),
                UInt32(sizeof(PX4Lockstep.HomePositionMsg)),
            ),
    )
    subs = Dict{Symbol,PX4Lockstep.UORBSubscriber}()
    bridge = Autopilots.UORBBridge(dummy_handle, pubs, subs)
    inj2 = Autopilots.build_state_injection_injector(bridge, Autopilots.HomeLocation())
    @test Set(map(Autopilots.uorb_name, inj2.sources)) == Set([:battery_status, :home_position])
end
