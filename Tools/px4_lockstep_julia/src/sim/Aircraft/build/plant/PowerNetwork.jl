"""Power network assembly helpers."""

"""Build a simple `PlantModels.PowerNetwork` from the aircraft spec.

The topology is intentionally simple:
* every motor must be assigned to exactly one bus
* every battery must be assigned to exactly one bus

Cross-feed/diode OR-ing is out of scope.
"""
function _build_power_network(spec::AircraftSpec)
    N = length(spec.actuation.motors)
    B = length(spec.power.batteries)
    K = length(spec.power.buses)

    # ID → index maps (deterministic, order-driven).
    motor_idx = Dict{MotorId,Int}()
    for (i, m) in enumerate(spec.actuation.motors)
        motor_idx[m.id] = i
    end
    bat_idx = Dict{BatteryId,Int}()
    for (i, b) in enumerate(spec.power.batteries)
        bat_idx[b.id] = i
    end

    # Assign each motor/battery to exactly one bus.
    # (Spec validation already enforces topology; keep only internal assertions here.)
    bus_for_motor = fill(0, N)
    bus_for_battery = fill(0, B)

    for (k, bus) in enumerate(spec.power.buses)
        for mid in bus.motor_ids
            i = motor_idx[mid]
            @assert bus_for_motor[i] == 0
            bus_for_motor[i] = k
        end
        for bid in bus.battery_ids
            i = bat_idx[bid]
            @assert bus_for_battery[i] == 0
            bus_for_battery[i] = k
        end
    end

    @assert all(x -> x != 0, bus_for_motor)
    @assert all(x -> x != 0, bus_for_battery)

    return PowerNetwork{N,B,K}(
        bus_for_motor = SVector{N,Int}(ntuple(i -> bus_for_motor[i], N)),
        bus_for_battery = SVector{B,Int}(ntuple(i -> bus_for_battery[i], B)),
        avionics_load_w = SVector{K,Float64}(
            ntuple(i -> Float64(spec.power.buses[i].avionics_load_w), K),
        ),
        share_mode = spec.power.share_mode,
    )
end
