using PX4Lockstep
using Plots

const G = 9.80665
const EARTH_RADIUS_M = 6.378137e6

function mission_home(path::AbstractString, default_lat::Float64, default_lon::Float64, default_alt::Float64)
    (isempty(path) || !isfile(path)) && return default_lat, default_lon, default_alt
    for line in readlines(path)
        startswith(strip(line), "QGC") && continue
        parts = split(strip(line))
        length(parts) < 11 && continue
        frame = parse(Int, parts[3])
        lat = parse(Float64, parts[9])
        lon = parse(Float64, parts[10])
        alt = frame == 3 ? default_alt : parse(Float64, parts[11])
        return lat, lon, alt
    end
    return default_lat, default_lon, default_alt
end

function ned_to_lla(x::Float64, y::Float64, z::Float64, lat0::Float64, lon0::Float64, alt0::Float64)
    dlat = x / EARTH_RADIUS_M
    dlon = y / (EARTH_RADIUS_M * cosd(lat0))
    lat = lat0 + rad2deg(dlat)
    lon = lon0 + rad2deg(dlon)
    alt = alt0 - z
    return lat, lon, alt
end

function quat_to_dcm(q::NTuple{4, Float64})
    qw, qx, qy, qz = q
    return [
        1.0 - 2.0 * (qy * qy + qz * qz)  2.0 * (qx * qy - qw * qz)        2.0 * (qx * qz + qw * qy)
        2.0 * (qx * qy + qw * qz)        1.0 - 2.0 * (qx * qx + qz * qz)  2.0 * (qy * qz - qw * qx)
        2.0 * (qx * qz - qw * qy)        2.0 * (qy * qz + qw * qx)        1.0 - 2.0 * (qx * qx + qy * qy)
    ]
end

function quat_integrate(q::NTuple{4, Float64}, rates::NTuple{3, Float64}, dt::Float64)
    qw, qx, qy, qz = q
    wx, wy, wz = rates
    dq = (
        -0.5 * (qx * wx + qy * wy + qz * wz),
         0.5 * (qw * wx + qy * wz - qz * wy),
         0.5 * (qw * wy - qx * wz + qz * wx),
         0.5 * (qw * wz + qx * wy - qy * wx),
    )
    qn = (qw + dq[1] * dt, qx + dq[2] * dt, qy + dq[3] * dt, qz + dq[4] * dt)
    norm_q = sqrt(sum(qn .* qn))
    return (qn[1] / norm_q, qn[2] / norm_q, qn[3] / norm_q, qn[4] / norm_q)
end

function yaw_from_quat(q::NTuple{4, Float64})
    qw, qx, qy, qz = q
    return atan(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
end

function run_lockstep()
    lib_path = get(ENV, PX4Lockstep.LIB_ENV, "")
    config = PX4Lockstep.LockstepConfig(enable_commander=0, enable_control_allocator=1)
    handle = isempty(lib_path) ? PX4Lockstep.create(config) : PX4Lockstep.create(config; libpath=lib_path)

    mission_path = get(ENV, "PX4_LOCKSTEP_MISSION", "")
    if !isempty(mission_path)
        rc = PX4Lockstep.load_mission(handle, mission_path)
        @info "Mission load" path=mission_path result=rc
    end

    default_lat = 47.397742
    default_lon = 8.545594
    default_alt = 488.0
    home_lat, home_lon, home_alt = mission_home(mission_path, default_lat, default_lon, default_alt)

    dt = 0.02
    dt_us = UInt64(round(Int, dt * 1e6))
    sim_steps = 2000

    pos = [0.0, 0.0, 0.0]
    vel = [0.0, 0.0, 0.0]
    rates = (0.0, 0.0, 0.0)
    q = (1.0, 0.0, 0.0, 0.0)

    drag_coeff = 0.05
    mass = 1.5
    rotor_count = 4
    inertia = (0.029125, 0.029125, 0.055225)
    angular_damping = (0.02, 0.02, 0.01)
    hover_thrust = 0.5
    max_total_thrust = mass * G / hover_thrust
    max_thrust_per_rotor = max_total_thrust / rotor_count
    rotor_positions = (
        (0.1515, 0.245, 0.0),
        (-0.1515, -0.1875, 0.0),
        (0.1515, -0.245, 0.0),
        (-0.1515, 0.1875, 0.0),
    )
    rotor_km = (0.05, 0.05, -0.05, -0.05)

    times = Float64[]
    pos_log = Vector{NTuple{3, Float64}}()
    vel_log = Vector{NTuple{3, Float64}}()
    pos_sp_log = Vector{NTuple{3, Float64}}()
    vel_sp_log = Vector{NTuple{3, Float64}}()
    thrust_log = Float64[]
    torque_log = Vector{NTuple{3, Float64}}()

    try
        for step in 0:sim_steps
            time_us = UInt64(step) * dt_us
            sim_time = step * dt

            armed_cmd = sim_time > 1.0 ? 1 : 0
            nav_auto_mission = (!isempty(mission_path) && sim_time > 2.0) ? 1 : 0
            takeoff_override = (armed_cmd == 1 && nav_auto_mission == 1 && sim_time > 2.5)
            landed = takeoff_override ? 0 : ((pos[3] >= -0.05 && abs(vel[3]) < 0.2) ? 1 : 0)

            lat, lon, alt = ned_to_lla(pos[1], pos[2], pos[3], home_lat, home_lon, home_alt)
            yaw = yaw_from_quat(q)

            inputs = PX4Lockstep.LockstepInputs(
                time_us=time_us,
                armed=armed_cmd,
                nav_auto_mission=nav_auto_mission,
                nav_auto_rtl=0,
                landed=landed,
                x=Cfloat(pos[1]),
                y=Cfloat(pos[2]),
                z=Cfloat(pos[3]),
                vx=Cfloat(vel[1]),
                vy=Cfloat(vel[2]),
                vz=Cfloat(vel[3]),
                yaw=Cfloat(yaw),
                lat_deg=lat,
                lon_deg=lon,
                alt_msl_m=Cfloat(alt),
                q=(Cfloat(q[1]), Cfloat(q[2]), Cfloat(q[3]), Cfloat(q[4])),
                rates_xyz=(Cfloat(rates[1]), Cfloat(rates[2]), Cfloat(rates[3])),
            )

            outputs = PX4Lockstep.step!(handle, inputs)

            thrust_body = (
                Float64(outputs.thrust_setpoint_body[1]),
                Float64(outputs.thrust_setpoint_body[2]),
                Float64(outputs.thrust_setpoint_body[3]),
            )
            motor_cmds = (
                Float64(outputs.actuator_motors[1]),
                Float64(outputs.actuator_motors[2]),
                Float64(outputs.actuator_motors[3]),
                Float64(outputs.actuator_motors[4]),
            )
            torque_cmd = (
                Float64(outputs.actuator_controls[1]),
                Float64(outputs.actuator_controls[2]),
                Float64(outputs.actuator_controls[3]),
            )
            pos_sp = (
                Float64(outputs.trajectory_setpoint_position[1]),
                Float64(outputs.trajectory_setpoint_position[2]),
                Float64(outputs.trajectory_setpoint_position[3]),
            )
            vel_sp = (
                Float64(outputs.trajectory_setpoint_velocity[1]),
                Float64(outputs.trajectory_setpoint_velocity[2]),
                Float64(outputs.trajectory_setpoint_velocity[3]),
            )

            if !all(isfinite, thrust_body)
                thrust_body = (0.0, 0.0, 0.0)
            end

            if !all(isfinite, motor_cmds)
                motor_cmds = (NaN, NaN, NaN, NaN)
            end

            if !all(isfinite, torque_cmd)
                torque_cmd = (0.0, 0.0, 0.0)
            end

            if all(isfinite, motor_cmds)
                total_thrust = 0.0
                torque_body = (0.0, 0.0, 0.0)
                for i in 1:rotor_count
                    cmd = clamp(motor_cmds[i], 0.0, 1.0)
                    thrust_i = cmd * max_thrust_per_rotor
                    total_thrust += thrust_i
                    rx, ry, _ = rotor_positions[i]
                    torque_body = (
                        torque_body[1] - ry * thrust_i,
                        torque_body[2] + rx * thrust_i,
                        torque_body[3] + rotor_km[i] * thrust_i,
                    )
                end
                thrust_body = (0.0, 0.0, -total_thrust / mass)
            else
                arm_length = maximum(abs, (
                    rotor_positions[1][1], rotor_positions[1][2],
                    rotor_positions[2][1], rotor_positions[2][2],
                    rotor_positions[3][1], rotor_positions[3][2],
                    rotor_positions[4][1], rotor_positions[4][2],
                ))
                total_thrust = clamp(-thrust_body[3], 0.0, 1.0) * max_total_thrust
                max_torque_xy = arm_length * max_total_thrust
                max_torque_z = max_total_thrust * maximum(abs, rotor_km)
                torque_body = (
                    clamp(torque_cmd[1], -1.0, 1.0) * max_torque_xy,
                    clamp(torque_cmd[2], -1.0, 1.0) * max_torque_xy,
                    clamp(torque_cmd[3], -1.0, 1.0) * max_torque_z,
                )
                thrust_body = (0.0, 0.0, -total_thrust / mass)
            end

            rates_dot = (
                (torque_body[1] - (inertia[3] - inertia[2]) * rates[2] * rates[3] - angular_damping[1] * rates[1]) / inertia[1],
                (torque_body[2] - (inertia[1] - inertia[3]) * rates[1] * rates[3] - angular_damping[2] * rates[2]) / inertia[2],
                (torque_body[3] - (inertia[2] - inertia[1]) * rates[1] * rates[2] - angular_damping[3] * rates[3]) / inertia[3],
            )
            rates = (
                rates[1] + rates_dot[1] * dt,
                rates[2] + rates_dot[2] * dt,
                rates[3] + rates_dot[3] * dt,
            )
            q = quat_integrate(q, rates, dt)

            thrust_accel_body = [
                0.0,
                0.0,
                thrust_body[3],
            ]
            rot = quat_to_dcm(q)
            thrust_accel_ned = rot * thrust_accel_body
            accel = thrust_accel_ned + [0.0, 0.0, G] - drag_coeff .* vel

            vel .+= accel .* dt
            pos .+= vel .* dt

            if pos[3] > 0.0
                pos[3] = 0.0
                vel[3] = min(vel[3], 0.0)
            end

            push!(times, sim_time)
            push!(pos_log, (pos[1], pos[2], pos[3]))
            push!(vel_log, (vel[1], vel[2], vel[3]))
            push!(pos_sp_log, pos_sp)
            push!(vel_sp_log, vel_sp)
            push!(thrust_log, thrust_body[3])
            push!(torque_log, torque_body)

            if step % 50 == 0
                @info "Step" step=step time_s=round(sim_time, digits=2) pos=Tuple(pos) vel=Tuple(vel) mission=(outputs.mission_seq, outputs.mission_count, outputs.mission_finished)
            end
        end
    finally
        PX4Lockstep.destroy(handle)
    end

    open("lockstep_log.csv", "w") do io
        println(io, "time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,pos_sp_x,pos_sp_y,pos_sp_z,vel_sp_x,vel_sp_y,vel_sp_z,thrust_z,torque_x,torque_y,torque_z")
        for i in eachindex(times)
            pos_i = pos_log[i]
            vel_i = vel_log[i]
            pos_sp_i = pos_sp_log[i]
            vel_sp_i = vel_sp_log[i]
            torque_i = torque_log[i]
            println(io, join((
                times[i], pos_i[1], pos_i[2], pos_i[3], vel_i[1], vel_i[2], vel_i[3],
                pos_sp_i[1], pos_sp_i[2], pos_sp_i[3], vel_sp_i[1], vel_sp_i[2], vel_sp_i[3],
                thrust_log[i], torque_i[1], torque_i[2], torque_i[3]
            ), ","))
        end
    end

    alt = map(p -> -p[3], pos_log)
    alt_sp = map(p -> -p[3], pos_sp_log)
    p1 = plot(times, alt, label="alt", ylabel="m")
    plot!(p1, times, alt_sp, label="alt_sp")
    p2 = plot(times, map(p -> p[1], pos_log), label="x", ylabel="m")
    plot!(p2, times, map(p -> p[1], pos_sp_log), label="x_sp")
    plot!(p2, times, map(p -> p[2], pos_log), label="y")
    plot!(p2, times, map(p -> p[2], pos_sp_log), label="y_sp")
    p3 = plot(times, map(p -> p[3], vel_log), label="vz", ylabel="m/s", xlabel="time (s)")
    plot!(p3, times, map(p -> p[3], vel_sp_log), label="vz_sp")
    plot(p1, p2, p3, layout=(3, 1), size=(900, 900))
    savefig("lockstep_plot.png")
end

run_lockstep()
