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

    # Single-instance publishers (legacy).
    pubs = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBPublisher,Int32}}}(
        :battery_status => [
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(0), UInt32(sizeof(T))),
                Int32(0),
            ),
        ],
        :home_position => [
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.HomePositionMsg}(
                    Int32(1),
                    UInt32(sizeof(PX4Lockstep.HomePositionMsg)),
                ),
                Int32(0),
            ),
        ],
    )
    subs = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBSubscriber,Int32}}}()
    bridge = Autopilots.UORBBridge(dummy_handle, pubs, subs)
    inj2 = Autopilots.build_state_injection_injector(bridge, Autopilots.HomeLocation())
    @test Set(map(Autopilots.uorb_name, inj2.sources)) == Set([:battery_status, :home_position])

    # Multi-battery injection: two configured publisher instances -> two battery_status sources.
    pubs_multi = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBPublisher,Int32}}}(
        :battery_status => [
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(0), UInt32(sizeof(T))),
                Int32(0),
            ),
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(1), UInt32(sizeof(T))),
                Int32(1),
            ),
        ],
    )
    subs_multi = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBSubscriber,Int32}}}()
    bridge_multi = Autopilots.UORBBridge(dummy_handle, pubs_multi, subs_multi)
    inj3 = Autopilots.build_state_injection_injector(bridge_multi, Autopilots.HomeLocation())
    @test Set(map(Autopilots.uorb_name, inj3.sources)) == Set([:battery_status_0, :battery_status_1])

    b0 = Sim.Powertrain.BatteryStatus(voltage_v = 11.0, current_a = 1.0, remaining = 0.9, warning = 0)
    b1 = Sim.Powertrain.BatteryStatus(voltage_v = 10.5, current_a = 2.0, remaining = 0.8, warning = 1)

    ctx = Autopilots.PX4StepContext(
        time_us = UInt64(100),
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        cmd = Sim.Autopilots.AutopilotCommand(),
        landed = false,
        battery = b0,
        batteries = [b0, b1],
        yaw_rad = 0.0,
        lat_deg = 0.0,
        lon_deg = 0.0,
        alt_msl_m = 0.0,
        ref_lat_deg = 0.0,
        ref_lon_deg = 0.0,
        ref_alt_m = 0.0,
        auto_mode = false,
        nav_state = UInt8(0),
        arming_state = UInt8(0),
        control_allocator_enabled = false,
    )

    bsrc0 = only(filter(s -> Autopilots.uorb_name(s) == :battery_status_0, inj3.sources))
    bsrc1 = only(filter(s -> Autopilots.uorb_name(s) == :battery_status_1, inj3.sources))
    msg0 = bsrc0.builder(ctx)
    msg1 = bsrc1.builder(ctx)

    @test msg0.id == UInt8(0)
    @test msg1.id == UInt8(1)
    @test msg0.voltage_v ≈ Float32(b0.voltage_v)
    @test msg1.voltage_v ≈ Float32(b1.voltage_v)

    # Fail fast if publishers expect more batteries than provided by context.
    ctx_short = Autopilots.PX4StepContext(
        time_us = UInt64(100),
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        cmd = Sim.Autopilots.AutopilotCommand(),
        landed = false,
        battery = b0,
        batteries = [b0],
        yaw_rad = 0.0,
        lat_deg = 0.0,
        lon_deg = 0.0,
        alt_msl_m = 0.0,
        ref_lat_deg = 0.0,
        ref_lon_deg = 0.0,
        ref_alt_m = 0.0,
        auto_mode = false,
        nav_state = UInt8(0),
        arming_state = UInt8(0),
        control_allocator_enabled = false,
    )
    @test_throws ErrorException bsrc1.builder(ctx_short)

    # Fail fast on duplicate battery indices.
    pubs_dup = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBPublisher,Int32}}}(
        :battery_status => [
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(0), UInt32(sizeof(T))),
                Int32(0),
            ),
            (
                PX4Lockstep.UORBPublisher{PX4Lockstep.BatteryStatusMsg}(Int32(1), UInt32(sizeof(T))),
                Int32(0), # duplicate instance -> duplicate battery index
            ),
        ],
    )
    subs_dup = Dict{Symbol,Vector{Tuple{PX4Lockstep.UORBSubscriber,Int32}}}()
    bridge_dup = Autopilots.UORBBridge(dummy_handle, pubs_dup, subs_dup)
    @test_throws ErrorException Autopilots.build_state_injection_injector(bridge_dup, Autopilots.HomeLocation())
end
