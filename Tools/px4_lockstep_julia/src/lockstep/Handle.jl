Base.@kwdef struct LockstepConfig
    dataman_use_ram::Int32 = 1
    enable_commander::Int32 = 0
    commander_rate_hz::Int32 = 100
    navigator_rate_hz::Int32 = 20
    mc_pos_control_rate_hz::Int32 = 100
    mc_att_control_rate_hz::Int32 = 250
    mc_rate_control_rate_hz::Int32 = 250
    enable_control_allocator::Int32 = 1
    control_allocator_rate_hz::Int32 = 250
end

Base.@kwdef struct LockstepCmd
    armed::UInt8 = 0x00
    request_mission::UInt8 = 0x00
    request_rtl::UInt8 = 0x00
end

"""Handle for a loaded `libpx4_lockstep` instance."""
struct LockstepFns
    orb_check::Ptr{Cvoid}
    orb_copy::Ptr{Cvoid}
    orb_unsubscribe::Ptr{Cvoid}
end

mutable struct LockstepHandle
    ptr::Ptr{Cvoid}
    lib::Ptr{Cvoid}
    config::LockstepConfig
    fns::LockstepFns
end

"""Validate Julia <-> C ABI compatibility.

This is intentionally separate from the shared-library queries so it can be unit tested
without loading `libpx4_lockstep`.
"""
function _check_abi!(abi_version::UInt32, in_sz::UInt32, out_sz::UInt32, cfg_sz::UInt32)
    isbitstype(LockstepConfig) || error("LockstepConfig must be an isbits type")

    abi_version in PX4_LOCKSTEP_ABI_VERSIONS || error(
        "libpx4_lockstep ABI mismatch: expected ABI versions $(PX4_LOCKSTEP_ABI_VERSIONS), got $abi_version",
    )

    exp_cfg = UInt32(sizeof(LockstepConfig))

    (in_sz == 0 && out_sz == 0) || error(
        "libpx4_lockstep ABI mismatch: legacy inputs/outputs detected (sizes $in_sz/$out_sz). " *
        "Rebuild libpx4_lockstep for uORB-only ABI v$(PX4_LOCKSTEP_ABI_VERSION_V3).",
    )
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

function _load_uorb_fns(lib::Ptr{Cvoid})::LockstepFns
    return LockstepFns(
        _resolve_symbol(lib, :px4_lockstep_orb_check),
        _resolve_symbol(lib, :px4_lockstep_orb_copy),
        _resolve_symbol(lib, :px4_lockstep_orb_unsubscribe),
    )
end

function _dummy_handle()::LockstepHandle
    return LockstepHandle(
        Ptr{Cvoid}(0),
        Ptr{Cvoid}(0),
        LockstepConfig(),
        LockstepFns(C_NULL, C_NULL, C_NULL),
    )
end

function create(
    config::LockstepConfig = LockstepConfig();
    libpath::Union{Nothing,AbstractString} = nothing,
    allow_multiple_handles::Bool = false,
)
    if config.enable_commander != 0
        error("Commander-in-loop lockstep is not supported; set enable_commander=0.")
    end
    _acquire_handle!(allow_multiple_handles)
    try
        lib = _load_library(libpath)
        # Fail fast if the shared library does not match the Julia-side struct layout.
        _abi_handshake!(lib)
        fn = _resolve_symbol(lib, :px4_lockstep_create)
        handle = ccall(fn, Ptr{Cvoid}, (Ref{LockstepConfig},), config)
        handle == C_NULL && error("px4_lockstep_create returned NULL")
        lockstep = LockstepHandle(handle, lib, config, _load_uorb_fns(lib))
        finalizer(lockstep) do instance
            try
                destroy(instance)
            catch
            end
        end
        return lockstep
    catch
        _release_handle!()
        rethrow()
    end
end

function destroy(handle::LockstepHandle)
    handle.ptr == C_NULL && return
    fn = _resolve_symbol(handle.lib, :px4_lockstep_destroy)
    ccall(fn, Cvoid, (Ptr{Cvoid},), handle.ptr)
    handle.ptr = C_NULL
    _release_handle!()
    return nothing
end

function load_mission(handle::LockstepHandle, path::AbstractString)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_load_mission_qgc_wpl)
    return ccall(fn, Cint, (Ptr{Cvoid}, Cstring), handle.ptr, path)
end

function step_uorb!(handle::LockstepHandle, time_us::UInt64)
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_step_uorb)
    catch err
        error("px4_lockstep_step_uorb unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Ptr{Cvoid}, UInt64), handle.ptr, time_us)
    ret == 0 || error("px4_lockstep_step_uorb failed with code $ret")
    return nothing
end

function set_cmd!(handle::LockstepHandle, cmd::LockstepCmd)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_set_cmd)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Ref{LockstepCmd}), handle.ptr, cmd)
    ret == 0 || error("px4_lockstep_set_cmd failed with code $ret")
    return nothing
end
