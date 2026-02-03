@testset "Runtime.Scheduler: due flags are correct" begin
    RT = Sim.Runtime

    t0_us = UInt64(0)
    t_end_us = UInt64(20_000)  # 0.02 s

    tl = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(10_000),
        dt_phys_us = UInt64(2_000),
    )

    sched = RT.Scheduler(tl)

    ev0 = RT.boundary_event(sched)
    @test ev0.time_us == t0_us
    @test ev0.due_ap
    @test ev0.due_wind
    @test ev0.due_log
    @test ev0.due_scn
    @test ev0.due_phys

    RT.consume_boundary!(sched, ev0)
    RT.advance_evt!(sched)

    ev1 = RT.boundary_event(sched)
    @test ev1.time_us == UInt64(2_000)
    @test !ev1.due_ap
    @test !ev1.due_wind
    @test !ev1.due_log
    @test !ev1.due_scn
    @test ev1.due_phys
end

@testset "Runtime.Timeline/Scheduler invariants (randomized)" begin
    RT = Sim.Runtime
    rng = Random.Xoshiro(0x12345678)

    for _ in 1:50
        t0_us = UInt64(0)
        t_end_us = UInt64(rand(rng, 10_000:500_000))

        # Random microsecond steps, always positive. Allow the axis dt to exceed
        # t_end (valid: axis becomes [t0]).
        dt_ap_us = UInt64(rand(rng, 1_000:100_000))
        dt_wind_us = UInt64(rand(rng, 1_000:150_000))
        dt_log_us = UInt64(rand(rng, 1_000:200_000))
        dt_phys_us = rand(rng, Bool) ? nothing : UInt64(rand(rng, 1_000:50_000))

        # Random scenario times, with possible duplicates.
        n_scn = rand(rng, 0:10)
        scn_times_us = [UInt64(rand(rng, 0:UInt64(t_end_us))) for _ in 1:n_scn]

        tl = RT.build_timeline(
            t0_us,
            t_end_us;
            dt_ap_us = dt_ap_us,
            dt_wind_us = dt_wind_us,
            dt_log_us = dt_log_us,
            scn_times_us = scn_times_us,
            dt_phys_us = dt_phys_us,
        )

        # Axis invariants
        @test issorted(tl.evt.t_us)
        @test length(unique(tl.evt.t_us)) == length(tl.evt.t_us)
        @test tl.evt.t_us[1] == t0_us
        @test tl.evt.t_us[end] == t_end_us
        @test tl.log.t_us[end] == t_end_us

        evt_set = Set(tl.evt.t_us)
        for axis in (tl.ap, tl.wind, tl.log, tl.scn, tl.phys)
            for t_us in axis.t_us
                @test t_us in evt_set
            end
        end

        # Scheduler invariants
        ap_set = Set(tl.ap.t_us)
        wind_set = Set(tl.wind.t_us)
        log_set = Set(tl.log.t_us)
        scn_set = Set(tl.scn.t_us)
        phys_set = Set(tl.phys.t_us)

        sched = RT.Scheduler(tl)
        n = 0
        prev = UInt64(0)
        first = true
        while true
            ev = RT.boundary_event(sched)
            if !first
                @test ev.time_us > prev
            end
            first = false
            prev = ev.time_us
            n += 1

            @test ev.due_ap == (ev.time_us in ap_set)
            @test ev.due_wind == (ev.time_us in wind_set)
            @test ev.due_log == (ev.time_us in log_set)
            @test ev.due_scn == (ev.time_us in scn_set)
            @test ev.due_phys == (ev.time_us in phys_set)

            RT.consume_boundary!(sched, ev)
            RT.has_next(sched) || break
            RT.advance_evt!(sched)
        end
        @test n == length(tl.evt.t_us)
    end
end

@testset "Runtime.Scheduler: 10-minute schedule has exact hits and constant time_us deltas" begin
    # Scheduler-level test: no drift over long horizons even with a dense physics axis.
    RT = Sim.Runtime

    t0_us = UInt64(0)
    t_end_us = UInt64(600_000_000)  # 600 s

    dt_phys_us = UInt64(2_000)
    dt_ap_us = UInt64(10_000)

    tl = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
        dt_phys_us = dt_phys_us,
    )

    sched = RT.Scheduler(tl)

    hits = 0
    last_ap_us = nothing

    while true
        ev = RT.boundary_event(sched)

        if ev.due_ap
            hits += 1
            if last_ap_us !== nothing
                @test ev.time_us - last_ap_us == dt_ap_us
            end
            last_ap_us = ev.time_us
        end

        RT.consume_boundary!(sched, ev)

        if RT.has_next(sched)
            RT.advance_evt!(sched)
        else
            break
        end
    end

    @test hits == length(tl.ap.t_us)
    @test last_ap_us == t_end_us
end
