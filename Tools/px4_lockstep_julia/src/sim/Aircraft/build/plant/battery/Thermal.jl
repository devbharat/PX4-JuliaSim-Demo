"""Battery thermal parameter helpers."""

function _build_battery_thermal_params(bs::BatterySpec, pack_asset_loaded)
    # -----------------------------
    # Temperature is a plant state. If thermal is disabled, its derivative is held at 0
    # and the initial value acts as a constant.
    temp_init =
        bs.thermal.initial_temp_c === nothing ? bs.temp_c : bs.thermal.initial_temp_c
    temp_amb = bs.thermal.ambient_temp_c === nothing ? bs.temp_c : bs.thermal.ambient_temp_c

    c_th_default =
        pack_asset_loaded === nothing ? nothing : pack_asset_loaded.thermal_c_th_j_per_k
    k_default =
        pack_asset_loaded === nothing ? nothing :
        pack_asset_loaded.thermal_k_to_ambient_w_per_k
    if pack_asset_loaded !== nothing
        if :c_th_j_per_k in bs.thermal.explicit_keys &&
           c_th_default !== nothing &&
           bs.thermal.c_th_j_per_k !== c_th_default
            @warn "Battery $(bs.id): TOML thermal.c_th_j_per_k=$(bs.thermal.c_th_j_per_k) overrides asset value $(c_th_default)"
        end
        if :k_to_ambient_w_per_k in bs.thermal.explicit_keys &&
           k_default !== nothing &&
           bs.thermal.k_to_ambient_w_per_k !== k_default
            @warn "Battery $(bs.id): TOML thermal.k_to_ambient_w_per_k=$(bs.thermal.k_to_ambient_w_per_k) overrides asset value $(k_default)"
        end
        if :ambient_temp_c in bs.thermal.explicit_keys &&
           bs.thermal.ambient_temp_c !== nothing
            @warn "Battery $(bs.id): TOML thermal.ambient_temp_c=$(bs.thermal.ambient_temp_c) overrides asset defaults"
        end
        if :initial_temp_c in bs.thermal.explicit_keys &&
           bs.thermal.initial_temp_c !== nothing
            @warn "Battery $(bs.id): TOML thermal.initial_temp_c=$(bs.thermal.initial_temp_c) overrides asset defaults"
        end
    end

    c_th =
        bs.thermal.c_th_j_per_k !== nothing ? bs.thermal.c_th_j_per_k :
        (c_th_default !== nothing ? c_th_default : 600.0)
    k_to_amb =
        bs.thermal.k_to_ambient_w_per_k !== nothing ? bs.thermal.k_to_ambient_w_per_k :
        (k_default !== nothing ? k_default : 0.0)

    thermal_params = Powertrain.ThermalParams(
        enabled = bs.thermal.enabled,
        c_th_j_per_k = c_th,
        k_to_ambient_w_per_k = k_to_amb,
        ambient_temp_c = temp_amb,
    )

    temp_c = temp_init

    return (thermal_params = thermal_params, temp_c = temp_c)
end
