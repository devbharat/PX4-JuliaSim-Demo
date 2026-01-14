module PX4Lockstep

using Libdl

export LockstepConfig,
    LockstepInputs,
    LockstepOutputs,
    LockstepHandle,
    create,
    destroy,
    load_mission,
    step!,
    find_library

# Simulation framework lives in a submodule so the core PX4 lockstep wrapper stays small.
export Sim

const LIB_ENV = "PX4_LOCKSTEP_LIB"
const PX4_LOCKSTEP_ABI_VERSION = UInt32(1)
const _LIB_HANDLE = Ref{Ptr{Cvoid}}(C_NULL)
const _SYMBOL_CACHE = Dict{Tuple{Ptr{Cvoid},Symbol},Ptr{Cvoid}}()

_lib_extension() =
    Sys.isapple() ? ".dylib" : Sys.islinux() ? ".so" : Sys.iswindows() ? ".dll" : ""

function find_library()
    if haskey(ENV, LIB_ENV)
        return ENV[LIB_ENV]
    end

    repo_root = normpath(joinpath(@__DIR__, "..", "..", ".."))
    ext = _lib_extension()
    candidates = (
        joinpath(
            repo_root,
            "build",
            "px4_sitl_lockstep",
            "src",
            "lib",
            "px4_lockstep",
            "libpx4_lockstep" * ext,
        ),
        joinpath(
            repo_root,
            "build",
            "px4_sitl_default",
            "src",
            "lib",
            "px4_lockstep",
            "libpx4_lockstep" * ext,
        ),
    )

    for path in candidates
        if isfile(path)
            return path
        end
    end

    error("PX4 lockstep library not found. Set $LIB_ENV to the built library path.")
end

function _load_library(path::Union{Nothing,AbstractString} = nothing)
    if path === nothing
        if _LIB_HANDLE[] == C_NULL
            _LIB_HANDLE[] = Ptr{Cvoid}(Libdl.dlopen(find_library()))
        end
        return _LIB_HANDLE[]
    end

    return Ptr{Cvoid}(Libdl.dlopen(String(path)))
end

function _resolve_symbol(lib::Ptr{Cvoid}, sym::Symbol)
    key = (lib, sym)
    return get!(_SYMBOL_CACHE, key) do
        Libdl.dlsym(lib, sym)
    end
end

Base.@kwdef struct LockstepConfig
    dataman_use_ram::Int32 = 1
    enable_commander::Int32 = 1
    commander_rate_hz::Int32 = 100
    navigator_rate_hz::Int32 = 20
    mc_pos_control_rate_hz::Int32 = 100
    mc_att_control_rate_hz::Int32 = 250
    mc_rate_control_rate_hz::Int32 = 250
    enable_control_allocator::Int32 = 1
    control_allocator_rate_hz::Int32 = 250
end

Base.@kwdef struct LockstepInputs
    time_us::UInt64 = 0
    armed::Int32 = 0
    nav_auto_mission::Int32 = 0
    nav_auto_rtl::Int32 = 0
    landed::Int32 = 1
    x::Cfloat = 0.0f0
    y::Cfloat = 0.0f0
    z::Cfloat = 0.0f0
    vx::Cfloat = 0.0f0
    vy::Cfloat = 0.0f0
    vz::Cfloat = 0.0f0
    yaw::Cfloat = 0.0f0
    lat_deg::Cdouble = 0.0
    lon_deg::Cdouble = 0.0
    alt_msl_m::Cfloat = 0.0f0
    q::NTuple{4,Cfloat} = (1.0f0, 0.0f0, 0.0f0, 0.0f0)
    rates_xyz::NTuple{3,Cfloat} = (0.0f0, 0.0f0, 0.0f0)
    battery_connected::Int32 = 1
    battery_voltage_v::Cfloat = 12.0f0
    battery_current_a::Cfloat = 0.0f0
    battery_remaining::Cfloat = 1.0f0
    battery_warning::Int32 = 0
end

struct LockstepOutputs
    actuator_controls::NTuple{8,Cfloat}
    actuator_motors::NTuple{12,Cfloat}
    actuator_servos::NTuple{8,Cfloat}
    attitude_setpoint_q::NTuple{4,Cfloat}
    rates_setpoint_xyz::NTuple{3,Cfloat}
    thrust_setpoint_body::NTuple{3,Cfloat}
    mission_seq::Int32
    mission_count::Int32
    mission_finished::Int32
    nav_state::Int32
    arming_state::Int32
    battery_warning::Int32
    trajectory_setpoint_position::NTuple{3,Cfloat}
    trajectory_setpoint_velocity::NTuple{3,Cfloat}
    trajectory_setpoint_acceleration::NTuple{3,Cfloat}
    trajectory_setpoint_yaw::Cfloat
    trajectory_setpoint_yawspeed::Cfloat
end

"""Handle for a loaded `libpx4_lockstep` instance."""
mutable struct LockstepHandle
    ptr::Ptr{Cvoid}
    lib::Ptr{Cvoid}
    outbuf::Ref{LockstepOutputs}
    config::LockstepConfig
end

"""Validate Julia <-> C ABI compatibility.

This is intentionally separate from the shared-library queries so it can be unit tested
without loading `libpx4_lockstep`.
"""
function _check_abi!(abi_version::UInt32, in_sz::UInt32, out_sz::UInt32, cfg_sz::UInt32)
    isbitstype(LockstepInputs) || error("LockstepInputs must be an isbits type")
    isbitstype(LockstepOutputs) || error("LockstepOutputs must be an isbits type")
    isbitstype(LockstepConfig) || error("LockstepConfig must be an isbits type")

    abi_version == PX4_LOCKSTEP_ABI_VERSION || error(
        "libpx4_lockstep ABI mismatch: expected ABI version $(PX4_LOCKSTEP_ABI_VERSION), got $abi_version",
    )

    exp_in = UInt32(sizeof(LockstepInputs))
    exp_out = UInt32(sizeof(LockstepOutputs))
    exp_cfg = UInt32(sizeof(LockstepConfig))

    in_sz == exp_in || error("ABI mismatch: inputs size expected $exp_in bytes, got $in_sz")
    out_sz == exp_out ||
        error("ABI mismatch: outputs size expected $exp_out bytes, got $out_sz")
    cfg_sz == exp_cfg ||
        error("ABI mismatch: config size expected $exp_cfg bytes, got $cfg_sz")

    return nothing
end

"""Run the C-side ABI handshake against the loaded library."""
function _abi_handshake!(lib::Ptr{Cvoid})
    fn_ver = _resolve_symbol(lib, :px4_lockstep_abi_version)
    abi = UInt32(ccall(fn_ver, Cuint, ()))

    fn_sizes = _resolve_symbol(lib, :px4_lockstep_sizes)
    in_sz = Ref{Cuint}(0)
    out_sz = Ref{Cuint}(0)
    cfg_sz = Ref{Cuint}(0)
    ccall(fn_sizes, Cvoid, (Ref{Cuint}, Ref{Cuint}, Ref{Cuint}), in_sz, out_sz, cfg_sz)

    _check_abi!(abi, UInt32(in_sz[]), UInt32(out_sz[]), UInt32(cfg_sz[]))
    return nothing
end

function create(
    config::LockstepConfig = LockstepConfig();
    libpath::Union{Nothing,AbstractString} = nothing,
)
    lib = _load_library(libpath)
    # Fail fast if the shared library does not match the Julia-side struct layout.
    _abi_handshake!(lib)
    fn = _resolve_symbol(lib, :px4_lockstep_create)
    handle = ccall(fn, Ptr{Cvoid}, (Ref{LockstepConfig},), config)
    handle == C_NULL && error("px4_lockstep_create returned NULL")
    lockstep = LockstepHandle(handle, lib, Ref{LockstepOutputs}(), config)
    finalizer(lockstep) do instance
        try
            destroy(instance)
        catch
        end
    end
    return lockstep
end

function destroy(handle::LockstepHandle)
    handle.ptr == C_NULL && return
    fn = _resolve_symbol(handle.lib, :px4_lockstep_destroy)
    ccall(fn, Cvoid, (Ptr{Cvoid},), handle.ptr)
    handle.ptr = C_NULL
    return nothing
end

function load_mission(handle::LockstepHandle, path::AbstractString)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_load_mission_qgc_wpl)
    return ccall(fn, Cint, (Ptr{Cvoid}, Cstring), handle.ptr, path)
end

function step!(handle::LockstepHandle, inputs::LockstepInputs)
    outputs = handle.outbuf
    fn = _resolve_symbol(handle.lib, :px4_lockstep_step)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Ref{LockstepInputs}, Ref{LockstepOutputs}),
        handle.ptr,
        inputs,
        outputs,
    )
    ret == 0 || error("px4_lockstep_step failed with code $ret")
    return outputs[]
end

include("sim/Sim.jl")

end
