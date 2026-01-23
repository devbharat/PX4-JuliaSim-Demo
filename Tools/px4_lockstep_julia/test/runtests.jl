using Test
using Random
using StaticArrays
using LinearAlgebra
using PX4Lockstep

const Sim = PX4Lockstep.Sim

# Verification cases (analytic + invariants). Keep in a separate file so the
# main test entrypoint stays readable.
include("verification_cases.jl")

# Verification: system-level contracts + missing subsystem unit coverage.
# These start as `@test_skip` shells and will be filled in incrementally.
include("verification_contracts.jl")

# uORB interface + injection scheduling checks (no PX4 binary required).
include("uorb_injection.jl")

# Record/replay engine (Option A) checks.

# Compare-integrators workflow (record/replay + metrics)
include("compare_integrators.jl")
include("record_replay_engine.jl")

# AircraftSpec scaffolding (Phase 0) checks.
include("aircraft_spec_iris_parity.jl")

# Phase 2: actuator mapping + generic multirotor counts (no PX4 required).
include("multirotor_motor_map.jl")

"""Return the geodesic rotation error (rad) between two quaternions.

Quaternions are treated as equivalent up to sign (q == -q).
"""
function quat_angle_error(q::Sim.Types.Quat, q_ref::Sim.Types.Quat)
    d = abs(sum(q .* q_ref))
    d = clamp(d, 0.0, 1.0)
    return 2.0 * acos(d)
end

struct ZeroRB end

function (f::ZeroRB)(t::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
    return Sim.RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
end

@testset "Integrators: adaptive RK45 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    # Analytic solution in NED (positive z = down).
    @test isapprox(x1.vel_ned[3], g; atol = 1e-4)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 1e-4)

    st = Sim.Integrators.last_stats(integ1)
    @test st.nfev > 0
    @test st.naccept > 0

    # Determinism: a fresh integrator produces identical results.
    integ2 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK23 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.vel_ned[3], g; atol = 5e-4)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 5e-4)

    st = Sim.Integrators.last_stats(integ1)
    @test st.nfev > 0
    @test st.naccept > 0

    integ2 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: RK4 free-fall correctness and determinism" begin
    g = 9.80665

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ1 = Sim.Integrators.RK4Integrator()
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.vel_ned[3], g; atol = 1e-6)
    @test isapprox(x1.pos_ned[3], 0.5 * g; atol = 1e-6)

    integ2 = Sim.Integrators.RK4Integrator()
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK45 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 1e-4)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 1e-4)

    # Other state groups remain unchanged for this RHS.
    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    # Determinism check.
    integ2 = Sim.Integrators.RK45Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: adaptive RK23 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 5e-4)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 5e-4)

    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    integ2 = Sim.Integrators.RK23Integrator(
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
        h_min = 1e-6,
        h_max = 0.5,
    )
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end

@testset "Integrators: RK4 supports PlantState" begin
    g = 9.80665

    function f(t::Float64, x::Sim.Plant.PlantState{4,1}, u)
        rḃ = Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.rb.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.RigidBody.quat_deriv(x.rb.q_bn, x.rb.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        return Sim.Plant.PlantDeriv{4,1}(rb = rḃ)
    end

    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    x0 = Sim.Plant.PlantState{4,1}(
        rb = rb0,
        power = Sim.Plant.PowerState{1}(
            soc = SVector{1,Float64}(1.0),
            v1 = SVector{1,Float64}(0.0),
        ),
    )

    integ1 = Sim.Integrators.RK4Integrator()
    x1 = Sim.Integrators.step_integrator(integ1, f, 0.0, x0, nothing, 1.0)

    @test isapprox(x1.rb.vel_ned[3], g; atol = 1e-6)
    @test isapprox(x1.rb.pos_ned[3], 0.5 * g; atol = 1e-6)

    @test x1.motors_y == x0.motors_y
    @test x1.servos_y == x0.servos_y
    @test x1.rotor_ω == x0.rotor_ω
    @test x1.power == x0.power

    integ2 = Sim.Integrators.RK4Integrator()
    x2 = Sim.Integrators.step_integrator(integ2, f, 0.0, x0, nothing, 1.0)
    @test x1 == x2
end


@testset "Integrators: plant-aware error norm is opt-in" begin
    rb0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    ω_hi = SVector{4,Float64}(100.0, 100.0, 100.0, 100.0)
    ω_lo = SVector{4,Float64}(90.0, 90.0, 90.0, 90.0)

    power0 = Sim.Plant.PowerState{1}(
        soc = SVector{1,Float64}(1.0),
        v1 = SVector{1,Float64}(0.0),
    )
    x_ref = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_hi, power = power0)
    x_hi = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_hi, power = power0)
    x_lo = Sim.Plant.PlantState{4,1}(rb = rb0, rotor_ω = ω_lo, power = power0)

    integ = Sim.Integrators.RK45Integrator(
        plant_error_control = false,  # default behavior
        atol_rotor = 1.0,
        rtol_rotor = 0.0,
        # Keep RB tolerances finite but RB deltas are zero here.
        rtol_pos = 1e-8,
        atol_pos = 1e-8,
        rtol_vel = 1e-8,
        atol_vel = 1e-8,
        rtol_ω = 1e-8,
        atol_ω = 1e-8,
        atol_att_rad = 1e-8,
    )

    err_off = Sim.Integrators._err_norm(integ, x_hi, x_lo, x_ref)
    @test err_off == 0.0

    integ.plant_error_control = true
    err_on = Sim.Integrators._err_norm(integ, x_hi, x_lo, x_ref)
    @test err_on > 1.0
end


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

@testset "PX4Lockstep ABI handshake helper" begin
    # Should pass with self-reported expectations.
    PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(0),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION + UInt32(1),
        UInt32(0),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )

    @test_throws ErrorException PX4Lockstep._check_abi!(
        PX4Lockstep.PX4_LOCKSTEP_ABI_VERSION,
        UInt32(4),
        UInt32(0),
        UInt32(sizeof(PX4Lockstep.LockstepConfig)),
    )
end

@testset "Noise.AR1 deterministic" begin
    rng = MersenneTwister(1234)
    a = Sim.Noise.AR1(1.0, 0.5)
    x1 = Sim.Noise.step!(a, rng, 0.01)
    rng2 = MersenneTwister(1234)
    a2 = Sim.Noise.AR1(1.0, 0.5)
    x2 = Sim.Noise.step!(a2, rng2, 0.01)
    @test x1 == x2
end

@testset "Environment.GustStep delegates stepping" begin
    # If mean wind is stateful (OU), stepping a GustStep wrapper should still advance it.
    rng = MersenneTwister(42)
    ou = Sim.Environment.OUWind(mean = Sim.Types.vec3(0, 0, 0), σ = Sim.Types.vec3(1, 0, 0), τ_s = 1.0)
    w = Sim.Environment.GustStep(ou, Sim.Types.vec3(0, 0, 0), 0.0, 1.0)
    pos = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Environment.step_wind!(w, pos, 0.0, 0.1, rng)
    @test ou.v_gust[1] != 0.0
end

@testset "Environment ISA altitude uses origin.alt_msl_m" begin
    atm = Sim.Environment.ISA1976()
    origin = Sim.Types.WorldOrigin(alt_msl_m = 1000.0)
    env = Sim.Environment.EnvironmentModel(atmosphere = atm, origin = origin)
    # At z=0 (at home origin), MSL altitude should be origin alt.
    rho0 = Sim.Environment.air_density(env.atmosphere, env.origin.alt_msl_m - 0.0)
    @test isapprox(rho0, Sim.Environment.air_density(atm, 1000.0); rtol = 1e-12)

    # At z=-100 (100 m above home), MSL altitude should be 1100 m.
    rho1 = Sim.Environment.air_density(env.atmosphere, env.origin.alt_msl_m - (-100.0))
    @test isapprox(rho1, Sim.Environment.air_density(atm, 1100.0); rtol = 1e-12)
end

@testset "Vehicles.FirstOrderActuators exact discretization" begin
    a = Sim.Vehicles.FirstOrderActuators{1}(τ = 0.1, y0 = SVector{1,Float64}(0.0))
    u = SVector{1,Float64}(1.0)
    y = Sim.Vehicles.step_actuators!(a, u, 0.05)
    @test isapprox(y[1], 1.0 - exp(-0.05 / 0.1); atol = 1e-12)
end

@testset "Runtime.Engine time_us is exact and log samples pre-step state" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(2_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1.0, 2.0, 3.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    log = Sim.Logging.SimLog()
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        log_sinks = log,
    )
    RT.run!(sim)

    # Log is pre-step state at t=0.
    @test length(log.t) >= 1
    @test log.t[1] == 0.0
    @test log.time_us[1] == UInt64(0)
    @test log.pos_ned[1] == (1.0, 2.0, 3.0)
end

@testset "Runtime.Engine respects t_end (no overshoot)" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(5_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(2_000),
        dt_wind_us = UInt64(2_000),
        dt_log_us = UInt64(2_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK45Integrator(h_min = 1e-6, h_max = 0.01)
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = ZeroRB(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)
    @test sim.bus.time_us == t_end_us
end

@testset "Runtime.Engine: AtTime scenario boundaries are true event boundaries" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    # Minimal scenario source that declares a one-off event boundary at 5 ms and
    # toggles a motor disable fault at that boundary.
    mutable struct MarkScenario
        times::Vector{UInt64}
    end

    function Sim.Runtime.event_times_us(src::MarkScenario, t0_us::UInt64, t_end_us::UInt64)
        t_evt = UInt64(5_000)
        return (t_evt >= t0_us && t_evt <= t_end_us) ? UInt64[t0_us, t_evt] : UInt64[t0_us]
    end

    function Sim.Runtime.update!(src::MarkScenario, bus::RT.SimBus, plant_state, t_us::UInt64)
        push!(src.times, t_us)
        if t_us >= UInt64(5_000)
            bus.faults = Sim.Faults.disable_motor(bus.faults, 1)
        end
        return nothing
    end

    scenario = MarkScenario(UInt64[])
    t0_us = UInt64(0)
    t_end_us = UInt64(12_000)
    timeline = RT.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(20_000),
        scenario = scenario,
    )

    # The evt axis must contain the scenario boundary (5 ms) even though AP ticks are 0/10 ms.
    @test UInt64(5_000) in timeline.evt.t_us

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState()
    integ = Sim.Integrators.RK4Integrator()

    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = integ,
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = scenario,
    )

    evt = timeline.evt.t_us
    @test evt[1] == t0_us
    # Process the first boundary, step to the 5 ms event, and process it.
    RT.process_events_at!(sim)
    RT.step_to_next_event!(sim)
    RT.process_events_at!(sim)

    @test sim.t_us == UInt64(5_000)
    @test Sim.Faults.is_motor_disabled(sim.bus.faults, 1)
    @test UInt64(5_000) in scenario.times
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

@testset "Runtime.Engine holds wind constant between wind ticks" begin
    RT = Sim.Runtime
    REC = Sim.Recording

    struct TimeWind <: Sim.Environment.AbstractWind end
    Sim.Environment.step_wind!(::TimeWind, ::Sim.Types.Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing
    Sim.Environment.sample_wind!(::TimeWind, ::Sim.Types.Vec3, ::Float64) = nothing
    Sim.Environment.wind_velocity(::TimeWind, ::Sim.Types.Vec3, t::Float64) =
        Sim.Types.vec3(t, 0.0, 0.0)

    env = Sim.Environment.EnvironmentModel(wind = TimeWind())

    t0_us = UInt64(0)
    t_end_us = UInt64(300_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(100_000),
        dt_wind_us = UInt64(200_000),
        dt_log_us = UInt64(100_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_src = Sim.Sources.LiveWindSource(env.wind, Random.Xoshiro(1), 0.2)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    log = Sim.Logging.SimLog()
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = ZeroRB(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        log_sinks = log,
    )
    RT.run!(sim)

    @test length(log.wind_ned) == length(timeline.log.t_us)
    @test log.wind_ned[1][1] == 0.0
    @test log.wind_ned[2][1] == 0.0
    @test isapprox(log.wind_ned[3][1], 0.2; atol = 1e-12)
    @test isapprox(log.wind_ned[4][1], 0.2; atol = 1e-12)
end

@testset "Logging.SimLog push" begin
    log = Sim.Logging.SimLog()
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1, 2, 3),
        vel_ned = Sim.Types.vec3(0.1, 0.2, 0.3),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    cmd = Sim.Vehicles.ActuatorCommand(
        motors = SVector{12,Float64}(fill(0.1, 12)),
        servos = SVector{8,Float64}(fill(0.0, 8)),
    )
    wind = Sim.Types.vec3(0.0, 0.0, 0.0)
    Sim.Logging.log!(log, 0.0, x, cmd; wind_ned = wind, rho = 1.2)
    @test length(log.t) == 1
    @test log.time_us[1] == UInt64(0)
    @test log.pos_ned[1] == (1.0, 2.0, 3.0)
end

@testset "Propulsion owns rotor_dir yaw-torque sign" begin
    env = Sim.iris_default_env_replay()
    vehicle = Sim.iris_default_vehicle()
    battery = Sim.iris_default_battery()

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        Sim.Contacts.NoContact(),
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    plant0 = Sim.Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Spin only rotor 3 (which has rotor_dir = -1 in the default set).
    motors = SVector{12,Float64}(ntuple(i -> (i == 3 ? 0.5 : 0.0), 12))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = zero(SVector{8,Float64}))
    u = Sim.Plant.PlantInput(
        cmd = cmd,
        wind_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        faults = Sim.Faults.FaultState(),
    )

    plant0_mapped =
        applicable(Sim.plant_on_autopilot_tick, dynfun, plant0, cmd) ?
        Sim.plant_on_autopilot_tick(dynfun, plant0, cmd) :
        plant0
    plant0_spin = Sim.Plant.PlantState{4,1}(
        rb = plant0_mapped.rb,
        motors_y = plant0_mapped.motors_y,
        motors_ydot = plant0_mapped.motors_ydot,
        servos_y = plant0_mapped.servos_y,
        servos_ydot = plant0_mapped.servos_ydot,
        rotor_ω = SVector{4,Float64}(0.0, 0.0, 300.0, 0.0),
        power = plant0_mapped.power,
    )

    y = Sim.plant_outputs(dynfun, 0.0, plant0_spin, u)
    out = y.rotors
    @test out.shaft_torque_nm[3] < 0.0

    d = Sim.Vehicles.dynamics(
        vehicle.model,
        env,
        0.0,
        plant0_spin.rb,
        out,
        Sim.Types.vec3(0.0, 0.0, 0.0),
    )
    @test d.ω_dot[3] < 0.0
end

@testset "Analytic: free-fall under gravity matches closed form" begin
    g = 9.80665
    dt = 0.01
    T = 1.0
    n = Int(round(T / dt))

    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = x.vel_ned,
            vel_dot = Sim.Types.vec3(0.0, 0.0, g),
            q_dot = Sim.Types.Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Closed form in NED (z is down):
    #   v_z(t) = g t
    #   z(t)   = 0.5 g t^2
    z_exp = 0.5 * g * T^2
    vz_exp = g * T

    @test isapprox(x.pos_ned[3], z_exp; atol = 1e-9)
    @test isapprox(x.vel_ned[3], vz_exp; atol = 1e-9)
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)
end

@testset "Analytic: constant body-rate quaternion integration" begin
    dt = 0.001
    T = 1.0
    n = Int(round(T / dt))

    ω = π  # rad/s about body Z (down)
    x = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, ω),
    )

    function f(t::Float64, x::Sim.RigidBody.RigidBodyState, _u)
        return Sim.RigidBody.RigidBodyDeriv(
            pos_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            vel_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
            ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
    end

    integrator = Sim.Integrators.RK4Integrator()
    t = 0.0
    for _ = 1:n
        x = Sim.Integrators.step_integrator(integrator, f, t, x, nothing, dt)
        t += dt
    end

    # Exact solution: q(t) = [cos(ωt/2), 0, 0, sin(ωt/2)]
    q_ref = Sim.Types.quat_from_axis_angle(Sim.Types.vec3(0.0, 0.0, 1.0), ω*T)
    θ_err = quat_angle_error(x.q_bn, q_ref)

    @test θ_err < 1e-6
    @test isapprox(norm(x.q_bn), 1.0; atol = 1e-12)

    yaw = Sim.Types.yaw_from_quat(x.q_bn)
    @test isapprox(Sim.Types.wrap_pi(yaw), Sim.Types.wrap_pi(ω*T); atol = 1e-6)
end

@testset "Analytic: DelayedEstimator ring-buffer exactness" begin
    rng = MersenneTwister(0)
    dt_est = 0.01
    delay_s = 0.03
    delay_steps = 3

    inner = Sim.Estimators.TruthEstimator()
    est = Sim.Estimators.DelayedEstimator(inner; delay_s = delay_s, dt_est = dt_est)
    Sim.Estimators.reset!(est)

    outs = Float64[]
    for k = 0:10
        t = k * dt_est
        x = Sim.RigidBody.RigidBodyState(
            pos_ned = Sim.Types.vec3(Float64(k), 0.0, 0.0),
            vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
            q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
        )
        y = Sim.Estimators.estimate!(est, rng, t, x, dt_est)
        push!(outs, y.pos_ned[1])
    end

    # During fill, the estimator returns current.
    @test outs[1] == 0.0
    @test outs[2] == 1.0
    @test outs[3] == 2.0

    # After warm-up, output is delayed by exactly delay_steps samples.
    for k = delay_steps+1:11
        # outs is 1-indexed; k corresponds to truth sample k-1.
        truth_k = k - 1
        @test outs[k] == Float64(truth_k - delay_steps)
    end

    # Reset should restore the warm-up behavior.
    Sim.Estimators.reset!(est)
    x = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(42.0, 0.0, 0.0))
    y = Sim.Estimators.estimate!(est, rng, 0.0, x, dt_est)
    @test y.pos_ned[1] == 42.0

    # Delay must be an exact multiple of dt_est (no silent rounding).
    @test_throws ErrorException Sim.Estimators.DelayedEstimator(
        inner;
        delay_s = 0.015,
        dt_est = dt_est,
    )

    # The runtime stepping cadence must match the configured dt_est.
    x = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0))
    @test_throws ErrorException Sim.Estimators.estimate!(est, rng, 0.0, x, 2 * dt_est)
end

@testset "Simulation strict_lockstep_rates errors on autopilot rate mismatch" begin
    # Minimal autopilot that advertises a fast internal loop rate.
    Base.@kwdef mutable struct RateOutputs
        actuator_motors::NTuple{12,Float32} = ntuple(_ -> 0f0, 12)
        actuator_servos::NTuple{8,Float32} = ntuple(_ -> 0f0, 8)
        trajectory_setpoint_position::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_velocity::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_acceleration::NTuple{3,Float32} = (0f0, 0f0, 0f0)
        trajectory_setpoint_yaw::Float32 = 0f0
        trajectory_setpoint_yawspeed::Float32 = 0f0
        nav_state::Int32 = Int32(0)
        arming_state::Int32 = Int32(0)
        mission_seq::Int32 = Int32(0)
        mission_count::Int32 = Int32(0)
        mission_finished::Int32 = Int32(0)
    end

    mutable struct RateAutopilot <: Sim.Autopilots.AbstractAutopilot
        out::RateOutputs
    end
    Sim.Autopilots.autopilot_output_type(::RateAutopilot) = RateOutputs
    Sim.Autopilots.max_internal_rate_hz(::RateAutopilot) = 250

    function Sim.Autopilots.autopilot_step(
        ap::RateAutopilot,
        time_us::UInt64,
        pos::Sim.Types.Vec3,
        vel::Sim.Types.Vec3,
        q::Sim.Types.Quat,
        ω::Sim.Types.Vec3,
        cmd::Sim.Autopilots.AutopilotCommand;
        landed::Bool = false,
        battery::Sim.Powertrain.BatteryStatus = Sim.Powertrain.BatteryStatus(),
        batteries::Vector{Sim.Powertrain.BatteryStatus} = Sim.Powertrain.BatteryStatus[battery],
    )::RateOutputs
        return ap.out
    end

    RT = Sim.Runtime
    REC = Sim.Recording

    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    ap = RateAutopilot(RateOutputs())
    autopilot_src = Sim.Sources.LiveAutopilotSource(ap)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    # dt_ap=0.01 (100 Hz) is slower than required for a 250 Hz loop.
    @test_throws ArgumentError Sim.simulate(
        mode = :live,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        dynfun = ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        strict_lockstep_rates = true,
    )

    # Opt-out should only warn (and succeed).
    sim = Sim.simulate(
        mode = :live,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        dynfun = ZeroRB(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        strict_lockstep_rates = false,
    )
    @test sim.cfg.mode == RT.MODE_LIVE
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


@testset "PlantModels: bus voltage solve (linear + saturated regimes)" begin
    pset = Sim.Propulsion.default_iris_quadrotor_set()
    p = pset  # QuadRotorSet{4}
    ω = SVector{4,Float64}(400.0, 400.0, 400.0, 400.0)

    ocv = 12.6
    v1 = 0.0
    R0 = 0.05
    V_min = 8.0

    # Linear-ish regime: moderate duty and ω so currents are positive but not saturated.
    duty_lin = SVector{4,Float64}(0.2, 0.2, 0.2, 0.2)
    V_lin = Sim.PlantModels._solve_bus_voltage(p, ω, duty_lin, ocv, v1, R0, V_min)
    I_lin = Sim.PlantModels._bus_current_total(p, ω, duty_lin, V_lin)
    @test isfinite(V_lin)
    @test V_lin >= V_min - 1e-9
    @test V_lin <= (ocv - v1) + 1e-9
    @test V_lin > V_min + 1e-6
    @test isapprox(V_lin + R0 * I_lin, ocv - v1; atol = 1e-5)

    # Saturated regime: near-stall at full duty (forces current clamping).
    ω_stall = SVector{4,Float64}(0.0, 0.0, 0.0, 0.0)
    duty_sat = SVector{4,Float64}(1.0, 1.0, 1.0, 1.0)
    V_sat = Sim.PlantModels._solve_bus_voltage(p, ω_stall, duty_sat, ocv, v1, R0, V_min)
    I_sat = Sim.PlantModels._bus_current_total(p, ω_stall, duty_sat, V_sat)
    @test isfinite(V_sat)
    @test V_sat >= V_min - 1e-9
    @test V_sat <= (ocv - v1) + 1e-9
    @test isapprox(V_sat, V_min; atol = 1e-6)
    res_sat = V_sat + R0 * I_sat - (ocv - v1)
    if isapprox(V_sat, V_min; atol = 1e-6)
        @test res_sat >= -1e-6
    else
        @test isapprox(res_sat, 0.0; atol = 1e-3)
    end

    # Current-limited regime (force I_lin >= Imax but avoid V_min clamp).
    p_sat = Sim.Propulsion.default_iris_quadrotor_set()
    units_sat = [
        Sim.Propulsion.MotorPropUnit(
            esc = unit.esc,
            motor = Sim.Propulsion.BLDCMotorParams(
                Kv_rpm_per_volt = unit.motor.Kv_rpm_per_volt,
                R_ohm = unit.motor.R_ohm,
                J_kgm2 = unit.motor.J_kgm2,
                I0_a = unit.motor.I0_a,
                viscous_friction_nm_per_rad_s = unit.motor.viscous_friction_nm_per_rad_s,
                max_current_a = 5.0,
            ),
            prop = unit.prop,
        ) for unit in p_sat.units
    ]
    p_sat = Sim.Propulsion.QuadRotorSet{4}(units_sat, p_sat.rotor_dir)

    ocv_sat = 12.6
    R0_sat = 0.02
    V_min_sat = 0.0
    V_cur = Sim.PlantModels._solve_bus_voltage(p_sat, ω_stall, duty_sat, ocv_sat, v1, R0_sat, V_min_sat)
    I_cur = Sim.PlantModels._bus_current_total(p_sat, ω_stall, duty_sat, V_cur)
    @test V_cur > V_min_sat + 1e-6
    @test isapprox(V_cur + R0_sat * I_cur, ocv_sat - v1; atol = 1e-3)
end
