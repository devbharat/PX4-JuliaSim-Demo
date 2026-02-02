"""Aircraft spec TOML I/O.

This module provides a small, dependency-light path from declarative TOML aircraft
specs to the existing `AircraftSpec` structs and `build_engine` entrypoint.

Design goals
------------
* **Deterministic**: parsing is pure; relative paths resolve against the TOML
  directory.
* **Strict by default**: unknown keys throw to catch typos early.
* **Composable**: supports `extends = ["base.toml", ...]` with deep table merges.

TOML schema
-----------
This intentionally mirrors the spec struct layout:

Top-level:
  - schema_version = 1
  - extends = ["base.toml"]  (optional)
  - [aircraft]
  - [home]
  - [px4]
  - [timeline]
  - [environment]
  - [scenario]
  - [estimator]
  - [plant]
  - [airframe]
  - [actuation]
  - [power]
  - [sensors]
  - [run] (optional convenience for `run_spec`)
"""

using TOML

using ..Types: Vec3, Quat, vec3, quat_normalize
using ..RigidBody: RigidBodyState
using ..Contacts
using ..Integrators
using ..Autopilots
using ..Logging

import ...LockstepConfig


# Root module handle (PX4Lockstep). Used to resolve generated uORB message types.
const _ROOT = parentmodule(parentmodule(@__MODULE__))


include("toml/Utils.jl")
include("toml/Defaults.jl")
include("toml/Sections/Home.jl")
include("toml/Sections/Timeline.jl")
include("toml/Sections/Environment.jl")
include("toml/Sections/Scenario.jl")
include("toml/Sections/Estimator.jl")
include("toml/Sections/Plant.jl")
include("toml/Sections/PX4.jl")
include("toml/Sections/Airframe.jl")
include("toml/Sections/Actuation.jl")
include("toml/Sections/Power.jl")
include("toml/Sections/Sensors.jl")


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

"""Parse an `AircraftSpec` from a TOML dictionary.

`base_dir` is used to resolve relative paths. If `base_spec` is provided, it
supplies defaults for any fields not present in the TOML.
"""
function spec_from_toml_dict(
    cfg_any::AbstractDict;
    base_dir::AbstractString = ".",
    strict::Bool = true,
    base_spec::Union{Nothing,AircraftSpec} = nothing,
)
    cfg = Dict{String,Any}(String(k) => v for (k, v) in cfg_any)

    # Top-level key guard.
    top_allowed = Set([
        "schema_version",
        "extends",
        "aircraft",
        "home",
        "px4",
        "timeline",
        "environment",
        "scenario",
        "estimator",
        "plant",
        "airframe",
        "actuation",
        "power",
        "sensors",
        "run",
    ])
    strict && _known_keys!(cfg, top_allowed, "root")

    schema = get(cfg, "schema_version", 1)
    _as_int(schema, "schema_version") == 1 ||
        error("Unsupported schema_version=$schema (expected 1)")

    # Optional name/seed metadata.
    name_sym = :aircraft
    atbl = nothing
    if haskey(cfg, "aircraft")
        atbl = _as_table(cfg["aircraft"], "aircraft")
        strict && _known_keys!(atbl, Set(["name", "seed"]), "aircraft")
        if haskey(atbl, "name")
            name_sym = _sym(_as_string(atbl["name"], "aircraft.name"))
        end
    end

    # Start from provided base spec (or default struct values).
    base = base_spec === nothing ? AircraftSpec(name = name_sym) : base_spec
    name = name_sym
    px4 = base.px4
    timeline = base.timeline
    environment = base.environment
    scenario = base.scenario
    estimator = base.estimator
    plant = base.plant
    airframe = base.airframe
    actuation = base.actuation
    power = base.power
    sensors = base.sensors
    seed = base.seed
    home = base.home
    telemetry = base.telemetry
    log_sinks = base.log_sinks
    if atbl !== nothing && haskey(atbl, "seed")
        seed = _as_int(atbl["seed"], "aircraft.seed")
    end

    # --- home ---
    if haskey(cfg, "home")
        home = _parse_home(cfg["home"]; strict = strict, ctx = "home", base = home)
    end

    # --- px4 ---
    if haskey(cfg, "px4")
        px4 = _parse_px4(
            cfg["px4"];
            strict = strict,
            ctx = "px4",
            base = px4,
            base_dir = base_dir,
        )
    end

    # --- timeline ---
    if haskey(cfg, "timeline")
        timeline = _parse_timeline(
            cfg["timeline"];
            strict = strict,
            ctx = "timeline",
            base = timeline,
        )
    end

    # --- environment ---
    if haskey(cfg, "environment")
        environment = _parse_environment(
            cfg["environment"];
            strict = strict,
            ctx = "environment",
            base = environment,
        )
    end

    # --- scenario ---
    if haskey(cfg, "scenario")
        scenario = _parse_scenario(
            cfg["scenario"];
            strict = strict,
            ctx = "scenario",
            base = scenario,
        )
    end

    # --- estimator ---
    if haskey(cfg, "estimator")
        estimator = _parse_estimator(
            cfg["estimator"];
            strict = strict,
            ctx = "estimator",
            base = estimator,
        )
    end

    # --- plant ---
    if haskey(cfg, "plant")
        plant = _parse_plant(cfg["plant"]; strict = strict, ctx = "plant", base = plant)
    end

    # --- airframe ---
    if haskey(cfg, "airframe")
        airframe = _parse_airframe(
            cfg["airframe"];
            strict = strict,
            ctx = "airframe",
            base = airframe,
        )
    end

    # --- actuation ---
    if haskey(cfg, "actuation")
        actuation = _parse_actuation(
            cfg["actuation"];
            strict = strict,
            ctx = "actuation",
            base = actuation,
        )
    end

    # --- power ---
    if haskey(cfg, "power")
        power = _parse_power(
            cfg["power"];
            strict = strict,
            ctx = "power",
            base = power,
            base_dir = base_dir,
        )
    end

    # --- sensors ---
    if haskey(cfg, "sensors")
        # Allow either [sensors] table (with sensors=[...]) or [[sensors]] array at top-level.
        sensors = _parse_sensors(cfg["sensors"]; strict = strict, ctx = "sensors")
    end

    # Convenience: if rotor axes not specified, default to classic multirotor axes.
    # This helps keep TOMLs shorter while preserving the required axis fields.
    if isempty(airframe.rotor_axis_body_m) && !isempty(airframe.rotor_pos_body_m)
        N = length(airframe.rotor_pos_body_m)
        airframe = AirframeSpec(
            kind = airframe.kind,
            mass_kg = airframe.mass_kg,
            inertia_diag_kgm2 = airframe.inertia_diag_kgm2,
            inertia_products_kgm2 = airframe.inertia_products_kgm2,
            rotor_pos_body_m = airframe.rotor_pos_body_m,
            rotor_axis_body_m = Vec3[vec3(0.0, 0.0, 1.0) for _ = 1:N],
            linear_drag = airframe.linear_drag,
            angular_damping = airframe.angular_damping,
            x0 = airframe.x0,
            propulsion = airframe.propulsion,
        )
    end

    return AircraftSpec(
        name = name,
        px4 = px4,
        timeline = timeline,
        environment = environment,
        scenario = scenario,
        estimator = estimator,
        plant = plant,
        airframe = airframe,
        actuation = actuation,
        power = power,
        sensors = sensors,
        seed = seed,
        home = home,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )
end


"""Load an `AircraftSpec` from a TOML file path.

By default this is strict and **does not** apply internal defaults. To explicitly
layer the built-in generic multirotor defaults, pass `base_spec=:default`.
"""
function load_spec(path::AbstractString; strict::Bool = true, base_spec = nothing)
    cfg = _load_toml_with_extends(path; strict = strict)
    base_dir = dirname(abspath(path))
    base = _resolve_base_spec(path, base_spec; strict = strict)
    return spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict, base_spec = base)
end


include("toml/RunSpec.jl")
