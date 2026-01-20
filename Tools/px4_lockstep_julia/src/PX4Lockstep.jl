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
    step_uorb!,
    UORBPublisher,
    UORBSubscriber,
    create_uorb_publisher,
    queue_uorb_publish!,
    uorb_publisher_instance,
    create_uorb_subscriber,
    uorb_check,
    uorb_copy!,
    uorb_copy,
    uorb_unsubscribe!,
    find_library

# Simulation framework lives in a submodule so the core PX4 lockstep wrapper stays small.
export Sim

const LIB_ENV = "PX4_LOCKSTEP_LIB"
const PX4_LOCKSTEP_ABI_VERSION_V1 = UInt32(1)
const PX4_LOCKSTEP_ABI_VERSION_V2 = UInt32(2)
const PX4_LOCKSTEP_ABI_VERSIONS = (PX4_LOCKSTEP_ABI_VERSION_V1, PX4_LOCKSTEP_ABI_VERSION_V2)
const _LIB_HANDLE = Ref{Ptr{Cvoid}}(C_NULL)
const _SYMBOL_CACHE = Dict{Tuple{Ptr{Cvoid},Symbol},Ptr{Cvoid}}()
const _HANDLE_LOCK = ReentrantLock()
const _HANDLE_COUNT = Ref{Int}(0)

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

function _acquire_handle!(allow_multiple_handles::Bool)
    lock(_HANDLE_LOCK) do
        if !allow_multiple_handles && _HANDLE_COUNT[] > 0
            error(
                "Only one libpx4_lockstep handle may be active per process. " *
                "Close the existing handle or pass allow_multiple_handles=true (unsafe).",
            )
        end
        _HANDLE_COUNT[] += 1
    end
    return nothing
end

function _release_handle!()
    lock(_HANDLE_LOCK) do
        _HANDLE_COUNT[] = max(0, _HANDLE_COUNT[] - 1)
    end
    return nothing
end

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

    abi_version in PX4_LOCKSTEP_ABI_VERSIONS || error(
        "libpx4_lockstep ABI mismatch: expected ABI versions $(PX4_LOCKSTEP_ABI_VERSIONS), got $abi_version",
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
        lockstep = LockstepHandle(handle, lib, Ref{LockstepOutputs}(), config)
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

# -----------------------------------------------------------------------------
# Generic uORB pub/sub (experimental)
# -----------------------------------------------------------------------------

"""Opaque publisher handle for uORB topics created through `libpx4_lockstep`.

The publisher only identifies the advertised topic and stores the expected message size.

Messages are *queued* and published on the next `step!()` call, after the lockstep time has
been advanced on the C side.
"""
struct UORBPublisher
    id::Int32
    msg_size::UInt32
end

"""Opaque subscription handle for uORB topics created through `libpx4_lockstep`."""
struct UORBSubscriber
    id::Int32
    msg_size::UInt32
end

"""Create a uORB publisher by topic name.

Returns `(pub, instance)` where `instance == -1` until the first publish (uORB assigns an
instance on advertise).

`priority` should be one of the ORB_PRIO_* constants (see PX4 uORB API). If `priority <= 0`,
the C side uses ORB_PRIO_DEFAULT.

`queue_size` controls uORB internal queuing for the topic (1 disables queuing).
"""
function create_uorb_publisher(
    handle::LockstepHandle,
    topic::AbstractString;
    priority::Integer = 0,
    queue_size::Integer = 1,
)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_create_publisher)
    pub_id = Ref{Int32}(-1)
    instance = Ref{Int32}(-1)
    msg_size = Ref{UInt32}(0)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Cstring, Int32, UInt32, Ref{Int32}, Ref{Int32}, Ref{UInt32}),
        handle.ptr,
        topic,
        Int32(priority),
        UInt32(queue_size),
        pub_id,
        instance,
        msg_size,
    )
    ret == 0 || error("px4_lockstep_orb_create_publisher failed for $topic with code $ret")
    return UORBPublisher(pub_id[], msg_size[]), instance[]
end

"""Create a uORB publisher and validate the Julia struct size."""
function create_uorb_publisher_checked(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T};
    priority::Integer = 0,
    queue_size::Integer = 1,
) where {T}
    pub, instance = create_uorb_publisher(
        handle,
        topic;
        priority = priority,
        queue_size = queue_size,
    )
    n = UInt32(sizeof(T))
    n == pub.msg_size || error(
        "uORB msg size mismatch for $topic: got $n bytes, expected $(pub.msg_size)",
    )
    return pub, instance
end

"""Queue a uORB publish for the next `step!()` call.

The message is copied into the C side immediately, so the Julia value does not need to live
until the next tick.

`msg` must be an `isbits` struct that matches the uORB C layout exactly.
"""
function queue_uorb_publish!(handle::LockstepHandle, pub::UORBPublisher, msg::T) where {T}
    isbitstype(T) || error("uORB messages must be isbits structs (got $T)")
    n = UInt32(sizeof(T))
    n == pub.msg_size || error(
        "uORB msg size mismatch for pub $(pub.id): got $n bytes, expected $(pub.msg_size)",
    )
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_queue_publish)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Int32, Ref{T}, UInt32),
        handle.ptr,
        pub.id,
        msg,
        n,
    )
    ret == 0 || error("px4_lockstep_orb_queue_publish failed with code $ret")
    return nothing
end

"""Queue a uORB publish from raw bytes for the next `step!()` call."""
function queue_uorb_publish!(handle::LockstepHandle, pub::UORBPublisher, bytes::Vector{UInt8})
    n = UInt32(length(bytes))
    n == pub.msg_size || error(
        "uORB msg size mismatch for pub $(pub.id): got $n bytes, expected $(pub.msg_size)",
    )
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_queue_publish)
    GC.@preserve bytes begin
        ret = ccall(
            fn,
            Cint,
            (Ptr{Cvoid}, Int32, Ptr{UInt8}, UInt32),
            handle.ptr,
            pub.id,
            pointer(bytes),
            n,
        )
        ret == 0 || error("px4_lockstep_orb_queue_publish(bytes) failed with code $ret")
    end
    return nothing
end

"""Get the uORB instance id assigned to a publisher (assigned on advertise)."""
function uorb_publisher_instance(handle::LockstepHandle, pub::UORBPublisher)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_publisher_instance)
    instance = Ref{Int32}(-1)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Int32, Ref{Int32}),
        handle.ptr,
        pub.id,
        instance,
    )
    ret == 0 || error("px4_lockstep_orb_publisher_instance failed with code $ret")
    return instance[]
end

"""Create a uORB subscriber by topic name and instance."""
function create_uorb_subscriber(
    handle::LockstepHandle,
    topic::AbstractString;
    instance::Integer = 0,
)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_create_subscriber)
    sub_id = Ref{Int32}(-1)
    msg_size = Ref{UInt32}(0)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Cstring, UInt32, Ref{Int32}, Ref{UInt32}),
        handle.ptr,
        topic,
        UInt32(instance),
        sub_id,
        msg_size,
    )
    ret == 0 || error(
        "px4_lockstep_orb_create_subscriber failed for $topic[$instance] with code $ret",
    )
    return UORBSubscriber(sub_id[], msg_size[])
end

"""Return whether a subscription has new data."""
function uorb_check(handle::LockstepHandle, sub::UORBSubscriber)::Bool
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_check)
    updated = Ref{Int32}(0)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32, Ref{Int32}), handle.ptr, sub.id, updated)
    ret == 0 || error("px4_lockstep_orb_check failed with code $ret")
    return updated[] != 0
end

"""Copy the latest topic data into `out` (no allocation)."""
function uorb_copy!(handle::LockstepHandle, sub::UORBSubscriber, out::Ref{T}) where {T}
    isbitstype(T) || error("uORB messages must be isbits structs (got $T)")
    n = UInt32(sizeof(T))
    n == sub.msg_size || error(
        "uORB msg size mismatch for sub $(sub.id): got $n bytes, expected $(sub.msg_size)",
    )
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_copy)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32, Ref{T}, UInt32), handle.ptr, sub.id, out, n)
    ret == 0 || error("px4_lockstep_orb_copy failed with code $ret")
    return nothing
end

"""Copy the latest topic data and return it (allocates one `Ref`)."""
function uorb_copy(handle::LockstepHandle, sub::UORBSubscriber, ::Type{T}) where {T}
    out = Ref{T}()
    uorb_copy!(handle, sub, out)
    return out[]
end

"""Unsubscribe and free the underlying uORB handle."""
function uorb_unsubscribe!(handle::LockstepHandle, sub::UORBSubscriber)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_unsubscribe)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32), handle.ptr, sub.id)
    ret == 0 || error("px4_lockstep_orb_unsubscribe failed with code $ret")
    return nothing
end

include("UORBGenerated.jl")
include("sim/Sim.jl")

end
