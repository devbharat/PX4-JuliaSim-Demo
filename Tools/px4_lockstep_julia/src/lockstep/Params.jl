# -----------------------------------------------------------------------------
# PX4 parameter access (experimental)
# -----------------------------------------------------------------------------

"""Set an int32 PX4 parameter by name."""
function param_set_i32(handle::LockstepHandle, name::AbstractString, value::Integer)
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_param_set_i32)
    catch
        error("px4_lockstep_param_set_i32 unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Cstring, Int32), handle.ptr, name, Int32(value))
    ret == 0 || error("px4_lockstep_param_set_i32 failed for $(name) with code $(ret)")
    return nothing
end

"""Set a float32 PX4 parameter by name."""
function param_set_f32(handle::LockstepHandle, name::AbstractString, value::Real)
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_param_set_f32)
    catch
        error("px4_lockstep_param_set_f32 unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Cstring, Cfloat), handle.ptr, name, Float32(value))
    ret == 0 || error("px4_lockstep_param_set_f32 failed for $(name) with code $(ret)")
    return nothing
end

"""Get an int32 PX4 parameter by name."""
function param_get_i32(handle::LockstepHandle, name::AbstractString)::Int32
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_param_get_i32)
    catch
        error("px4_lockstep_param_get_i32 unavailable; rebuild libpx4_lockstep")
    end
    out = Ref{Int32}(0)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Cstring, Ref{Int32}), handle.ptr, name, out)
    ret == 0 || error("px4_lockstep_param_get_i32 failed for $(name) with code $(ret)")
    return out[]
end

"""Get a float32 PX4 parameter by name."""
function param_get_f32(handle::LockstepHandle, name::AbstractString)::Float32
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_param_get_f32)
    catch
        error("px4_lockstep_param_get_f32 unavailable; rebuild libpx4_lockstep")
    end
    out = Ref{Cfloat}(0)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Cstring, Ref{Cfloat}), handle.ptr, name, out)
    ret == 0 || error("px4_lockstep_param_get_f32 failed for $(name) with code $(ret)")
    return Float32(out[])
end

"""Queue an int32 PX4 parameter to be applied before module init."""
function param_preinit_set_i32(
    name::AbstractString,
    value::Integer;
    libpath::Union{Nothing,AbstractString} = nothing,
)
    lib = _load_library(libpath)
    if libpath !== nothing && _LIB_HANDLE[] == C_NULL
        _LIB_HANDLE[] = lib
    end
    fn = try
        _resolve_symbol(lib, :px4_lockstep_param_preinit_set_i32)
    catch
        error("px4_lockstep_param_preinit_set_i32 unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Cstring, Int32), name, Int32(value))
    ret == 0 ||
        error("px4_lockstep_param_preinit_set_i32 failed for $(name) with code $(ret)")
    return nothing
end

"""Queue a float32 PX4 parameter to be applied before module init."""
function param_preinit_set_f32(
    name::AbstractString,
    value::Real;
    libpath::Union{Nothing,AbstractString} = nothing,
)
    lib = _load_library(libpath)
    if libpath !== nothing && _LIB_HANDLE[] == C_NULL
        _LIB_HANDLE[] = lib
    end
    fn = try
        _resolve_symbol(lib, :px4_lockstep_param_preinit_set_f32)
    catch
        error("px4_lockstep_param_preinit_set_f32 unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Cstring, Cfloat), name, Float32(value))
    ret == 0 ||
        error("px4_lockstep_param_preinit_set_f32 failed for $(name) with code $(ret)")
    return nothing
end

"""Queue a PX4 parameter to be applied before module init."""
function param_preinit_set!(
    name::AbstractString,
    value;
    libpath::Union{Nothing,AbstractString} = nothing,
)
    if value isa Integer
        return param_preinit_set_i32(name, value; libpath = libpath)
    else
        return param_preinit_set_f32(name, Float32(value); libpath = libpath)
    end
end

"""Set a PX4 parameter by name, choosing int32 vs float32 based on the Julia value."""
function param_set!(handle::LockstepHandle, name::AbstractString, value)
    if value isa Integer
        return param_set_i32(handle, name, value)
    else
        return param_set_f32(handle, name, Float32(value))
    end
end

"""Get a PX4 parameter by name.

This attempts int32 first and falls back to float32 if the int32 query fails.
"""
function param_get(handle::LockstepHandle, name::AbstractString)
    fn_i32 = _resolve_symbol(handle.lib, :px4_lockstep_param_get_i32)
    out_i32 = Ref{Int32}(0)
    ret = ccall(fn_i32, Cint, (Ptr{Cvoid}, Cstring, Ref{Int32}), handle.ptr, name, out_i32)
    if ret == 0
        return out_i32[]
    end
    if ret != -3
        error("px4_lockstep_param_get_i32 failed for $(name) with code $(ret)")
    end

    fn_f32 = _resolve_symbol(handle.lib, :px4_lockstep_param_get_f32)
    out_f32 = Ref{Cfloat}(0)
    ret_f =
        ccall(fn_f32, Cint, (Ptr{Cvoid}, Cstring, Ref{Cfloat}), handle.ptr, name, out_f32)
    ret_f == 0 || error("px4_lockstep_param_get_f32 failed for $(name) with code $(ret_f)")
    return Float32(out_f32[])
end

"""Publish a PX4 parameter_update notification.

Useful after batching multiple parameter changes.
"""
function param_notify!(handle::LockstepHandle)
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_param_notify)
    catch
        error("px4_lockstep_param_notify unavailable; rebuild libpx4_lockstep")
    end
    ret = ccall(fn, Cint, (Ptr{Cvoid},), handle.ptr)
    ret == 0 || error("px4_lockstep_param_notify failed with code $(ret)")
    return nothing
end

"""Force the control allocator to reload CA_* parameters immediately.

This avoids a global param_notify broadcast when only control allocation geometry changes.
"""
function control_alloc_update_params!(handle::LockstepHandle)
    fn = try
        _resolve_symbol(handle.lib, :px4_lockstep_control_alloc_update_params)
    catch
        error(
            "px4_lockstep_control_alloc_update_params unavailable; rebuild libpx4_lockstep",
        )
    end
    ret = ccall(fn, Cint, (Ptr{Cvoid},), handle.ptr)
    ret == 0 || error("px4_lockstep_control_alloc_update_params failed with code $(ret)")
    return nothing
end
