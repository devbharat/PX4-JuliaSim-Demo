############################
# Protocol: algebraic outputs
############################

"""Evaluate algebraic plant outputs at (t, x, u).

Intended for boundary-time logging and PX4 injection (battery_status). Must be pure.
"""
function plant_outputs(
    f::CoupledMultirotorModel{M,E,C,AM,AS,P,BAT,PowerNetwork{N,B,K},MM,SM},
    t::Float64,
    x::PlantState{N,B},
    u::PlantInput,
) where {M,E,C,AM,AS,P,BAT,MM,SM,N,B,K}
    p = f.propulsion
    p isa Propulsion.QuadRotorSet{N} ||
        error("PlantOutputs currently supports QuadRotorSet{$N} propulsion")

    rot_out, _ωdot, _I_motors_total, V_bus, I_bus_total, I_batt, _ρ, _v_air_body =
        _eval_propulsion_and_power_network(
            p,
            f.batteries,
            f.power_net,
            f.env,
            t,
            x,
            u,
            f.motor_map,
            f.model.params.rotor_axis_body,
        )

    # Compute per-battery telemetry.
    batt_all = SVector{B,BatteryStatus}(
        ntuple(
            i -> begin
                k = f.power_net.bus_for_battery[i]
                V_i = (1 <= k <= length(V_bus)) ? V_bus[k] : 0.0
                I_i = I_batt[i]
                _battery_status_from_state(
                    f.batteries[i],
                    x.power.soc[i],
                    x.power.v1[i],
                    I_i,
                    V_i;
                    temp_c = x.power.temp_c[i],
                    connected = u.faults.battery_connected,
                )
            end,
            B,
        ),
    )

    # Atmosphere expects MSL altitude (consistent with density in propulsion eval).
    alt_msl_m = f.env.origin.alt_msl_m - x.rb.pos_ned[3]
    temp_k = air_temperature(f.env.atmosphere, alt_msl_m)

    # Contact observables (optional).
    contact_y = nothing
    if !(f.contact isa NoContact)
        if f.contact isa FlatGroundConstraintContact
            # The constraint contact depends on the unconstrained acceleration; compute it
            # consistently with the RHS (contact not applied).
            d_rb_free = Vehicles.dynamics(f.model, f.env, t, x.rb, rot_out, u.wind_ned)
            contact_y = contact_info(f.contact, x.rb, t, mass(f.model), d_rb_free.vel_dot)
        else
            contact_y = contact_info(f.contact, x.rb, t)
        end
    end
    return PlantOutputs{N,B,K}(
        rotors = rot_out,
        bus_current_a = SVector{K,Float64}(I_bus_total),
        bus_voltage_v = SVector{K,Float64}(V_bus),
        rho_kgm3 = _ρ,
        temp_k = temp_k,
        air_vel_body = _v_air_body,
        contact = contact_y,
        battery_statuses = batt_all,
    )
end
