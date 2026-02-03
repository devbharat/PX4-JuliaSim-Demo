module BuildPX4

"""PX4 lockstep initialization and parameter wiring helpers."""

using ..Aircraft: AircraftSpec, PX4ParamSpec
using ...Vehicles
using ...Autopilots
using ...Powertrain

using PX4Lockstep:
    param_set!, param_notify!, param_get, param_preinit_set!, control_alloc_update_params!

export _build_px4_autopilot

"""Derive PX4 control allocator (CA_*) parameters from the aircraft spec.

This matches the old lockstep C-side defaults, but is now spec-driven.
"""
function _derive_ca_params(spec::AircraftSpec, vehicle::Vehicles.VehicleInstance)
    cfg = spec.px4.lockstep_config
    cfg.enable_control_allocator == 0 && return PX4ParamSpec[]

    N = length(spec.actuation.motors)
    pos = spec.airframe.rotor_pos_body_m
    length(pos) == N || throw(
        ArgumentError("Cannot derive CA params: rotor_pos_body_m length != motor count"),
    )

    # Rotor/propulsor axes: required by spec; thrust is applied along -axis_b.
    axis_src = spec.airframe.rotor_axis_body_m
    length(axis_src) == N || throw(
        ArgumentError("Cannot derive CA params: rotor_axis_body_m length != motor count"),
    )

    km_mag = spec.airframe.propulsion.km_m
    rotor_dir = vehicle.propulsion.rotor_dir

    params = PX4ParamSpec[
        PX4ParamSpec("CA_AIRFRAME", 0),              # multicopter (custom geometry)
        PX4ParamSpec("CA_ROTOR_COUNT", N),
    ]

    for i = 1:N
        p = pos[i]
        a = axis_src[i]
        n2 = a[1] * a[1] + a[2] * a[2] + a[3] * a[3]
        n2 > 1e-12 || throw(
            ArgumentError(
                "Cannot derive CA params: rotor_axis_body_m[$i] is near-zero: $a",
            ),
        )
        invn = inv(sqrt(n2))
        axis_b = a .* invn
        # PX4 CA_ROTOR*_A* expects the *thrust vector direction* in body frame.
        # Our convention stores propulsor axis such that F = -T * axis_b.
        axis_thrust = -axis_b
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PX", Float32(p[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PY", Float32(p[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PZ", Float32(p[3])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_KM", Float32(km_mag * rotor_dir[i])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AX", Float32(axis_thrust[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AY", Float32(axis_thrust[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AZ", Float32(axis_thrust[3])))
    end

    return params
end

"""Apply a list of PX4 parameters to a running lockstep autopilot."""
function _apply_px4_params!(
    ap::Autopilots.PX4LockstepAutopilot,
    params::Vector{PX4ParamSpec},
)
    for p in params
        param_set!(ap.handle, p.name, p.value)
    end
    return nothing
end

function _debug_px4_params!(
    ap::Autopilots.PX4LockstepAutopilot,
    params::Vector{PX4ParamSpec},
)
    flag = lowercase(String(get(ENV, "PX4_LOCKSTEP_DEBUG_PARAMS", "")))
    if !(flag in ("1", "true", "yes", "on"))
        return nothing
    end

    if isempty(params)
        println("[PX4 params] (no spec params applied)")
        return nothing
    end

    println("[PX4 params] applied:")
    for p in params
        value = try
            param_get(ap.handle, p.name)
        catch err
            "<error: $(sprint(showerror, err))>"
        end
        println("  ", p.name, " = ", value)
    end
    return nothing
end


@inline function _is_ca_axis_param(name::AbstractString)::Bool
    # PX4 CA rotor axis params: CA_ROTOR<i>_AX / _AY / _AZ
    return startswith(name, "CA_ROTOR") &&
           (endswith(name, "_AX") || endswith(name, "_AY") || endswith(name, "_AZ"))
end

function _build_px4_autopilot(spec::AircraftSpec, vehicle::Vehicles.VehicleInstance, plant0)
    # Apply spec-driven PX4 parameters.
    px4_params = PX4ParamSpec[]
    if spec.px4.derive_ca_params
        append!(px4_params, _derive_ca_params(spec, vehicle))
    end
    # Explicit overrides win.
    append!(px4_params, spec.px4.params)

    # Some PX4 branches do not expose CA_ROTOR*_A* parameters (tilt/axis support). We
    # therefore stage those as *optional* and apply them after PX4 init if they exist.
    axis_params = PX4ParamSpec[]
    for p in px4_params
        if _is_ca_axis_param(p.name)
            push!(axis_params, p)
        else
            param_preinit_set!(p.name, p.value; libpath = spec.px4.libpath)
        end
    end

    # PX4 autopilot (lockstep) init.
    ap = Autopilots.init!(
        config = spec.px4.lockstep_config,
        libpath = spec.px4.libpath,
        home = spec.home,
        uorb_cfg = spec.px4.uorb_cfg,
        edge_trigger = spec.px4.edge_trigger,
    )

    # Apply optional CA_ROTOR*_A* params if the PX4 build supports them.
    axis_applied = false
    if !isempty(axis_params)
        for p in axis_params
            # param_get throws if param doesn't exist; treat that as "not supported".
            supported = try
                _ = param_get(ap.handle, p.name)
                true
            catch
                false
            end
            if supported
                param_set!(ap.handle, p.name, p.value)
                axis_applied = true
            end
        end
        # Reload CA params without a global param_notify.
        (axis_applied && spec.px4.lockstep_config.enable_control_allocator != 0) &&
            control_alloc_update_params!(ap.handle)
    end

    # Only broadcast a parameter update if explicit overrides were provided.
    isempty(spec.px4.params) || param_notify!(ap.handle)
    _debug_px4_params!(ap, px4_params)

    # Initial step at t=0 so PX4 has a consistent state baseline.
    _ = Autopilots.autopilot_step(
        ap,
        UInt64(0),
        plant0.rb.pos_ned,
        plant0.rb.vel_ned,
        plant0.rb.q_bn,
        plant0.rb.ω_body,
        Autopilots.AutopilotCommand();
        landed = true,
        battery = Powertrain.BatteryStatus(),
    )

    # Load mission if provided (after uORB topics are initialized).
    if spec.px4.mission_path !== nothing
        Autopilots.load_mission!(ap, spec.px4.mission_path)
    end

    return ap
end

end # module BuildPX4
