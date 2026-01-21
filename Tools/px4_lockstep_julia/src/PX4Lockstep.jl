module PX4Lockstep

using Libdl

export LockstepConfig,
    LockstepHandle,
    create,
    destroy,
    load_mission,
    step_uorb!,
    UORBPublisher,
    UORBSubscriber,
    create_publisher,
    create_subscriber,
    publish!,
    publish_unsafe!,
    create_uorb_publisher,
    create_uorb_publisher_checked,
    queue_uorb_publish!,
    uorb_publisher_instance,
    uorb_topic_metadata,
    create_uorb_subscriber,
    create_uorb_subscriber_checked,
    uorb_check,
    uorb_copy!,
    uorb_copy,
    uorb_unsubscribe!,
    verify_uorb_type!,
    verify_uorb_contract!,
    find_library

# Simulation framework lives in a submodule so the core PX4 lockstep wrapper stays small.
export Sim

const LIB_ENV = "PX4_LOCKSTEP_LIB"
const PX4_LOCKSTEP_ABI_VERSION_V3 = UInt32(3)
const PX4_LOCKSTEP_ABI_VERSION = PX4_LOCKSTEP_ABI_VERSION_V3
const PX4_LOCKSTEP_ABI_VERSIONS = (PX4_LOCKSTEP_ABI_VERSION_V3,)
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

"""Handle for a loaded `libpx4_lockstep` instance."""
mutable struct LockstepHandle
    ptr::Ptr{Cvoid}
    lib::Ptr{Cvoid}
    config::LockstepConfig
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
        lockstep = LockstepHandle(handle, lib, config)
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

# -----------------------------------------------------------------------------
# Generic uORB pub/sub (experimental)
# -----------------------------------------------------------------------------

"""Query uORB topic metadata from the loaded PX4 binary.

Returns `(fields, size_bytes, size_no_padding_bytes, message_hash, queue_size)`.

`fields` is the uORB canonical fields description string (`orb_metadata.o_fields`).
The returned string is copied into Julia memory.

This is intended for *init-time* contract validation so Julia message structs
cannot silently drift away from the PX4 build they are interacting with.
"""
function uorb_topic_metadata(handle::LockstepHandle, topic::AbstractString)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_topic_metadata)
    fields_ptr = Ref{Cstring}(C_NULL)
    size = Ref{UInt32}(0)
    size_no_padding = Ref{UInt32}(0)
    message_hash = Ref{UInt32}(0)
    queue_size = Ref{UInt8}(0)
    ret = ccall(
        fn,
        Cint,
        (
            Ptr{Cvoid},
            Cstring,
            Ref{Cstring},
            Ref{UInt32},
            Ref{UInt32},
            Ref{UInt32},
            Ref{UInt8},
        ),
        handle.ptr,
        topic,
        fields_ptr,
        size,
        size_no_padding,
        message_hash,
        queue_size,
    )

    ret == 0 ||
        ret == -3 ||
        error("px4_lockstep_orb_topic_metadata failed for $topic with code $ret")

    fields = fields_ptr[] == C_NULL ? "" : unsafe_string(fields_ptr[])
    return (fields, size[], size_no_padding[], message_hash[], queue_size[])
end

# -----------------------------------------------------------------------------
# uORB contract verification ("no drift")
# -----------------------------------------------------------------------------

"""Canonicalize a uORB fields string for stable hashing.

This *must* match the canonicalization used by `scripts/uorb_codegen.jl`.

We remove all whitespace and any trailing semicolons. This makes the hash robust
to formatting differences between generated headers and PX4 runtime metadata.
"""
function canonicalize_uorb_fields(s::AbstractString)
    t = replace(s, r"\s+" => "")
    t = replace(t, r";+$" => "")
    return t
end

const _FNV1A_64_OFFSET_BASIS = UInt64(0xcbf29ce484222325)
const _FNV1A_64_PRIME = UInt64(0x100000001b3)

"""Compute a stable FNV-1a 64-bit hash of a string.

This *must* match the hashing used by `scripts/uorb_codegen.jl`.
"""
function fnv1a64(s::AbstractString)
    h = _FNV1A_64_OFFSET_BASIS
    for b in codeunits(s)
        h = (h ⊻ UInt64(b)) * _FNV1A_64_PRIME
    end
    return h
end

"""Compute the uORB contract hash from a PX4 `orb_metadata.o_fields` string."""
uorb_fields_hash_runtime(fields::AbstractString) = fnv1a64(canonicalize_uorb_fields(fields))

# Cache of verified (topic,type) pairs.
const _UORB_CONTRACT_CACHE = Dict{Tuple{String,DataType},Bool}()

"""Verify that a Julia uORB message type matches the loaded PX4 binary.

This enforces:

* `sizeof(T)` == `orb_metadata.o_size`
* `uorb_fields_hash(T)` == `hash(orb_metadata.o_fields)`

If PX4 does not provide an `o_fields` string for the topic, this throws.

This is intended for init-time checks (publisher/subscriber creation). It is not
performance-critical.
"""
function verify_uorb_type!(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T},
) where {T}
    key = (String(topic), T)
    get(_UORB_CONTRACT_CACHE, key, false) && return nothing

    # If the generator emitted a topic mapping, ensure the caller is not
    # accidentally pairing the wrong type with a topic.
    if @isdefined(uorb_topic)
        try
            exp_topic = uorb_topic(T)
            exp_topic == String(topic) || error(
                "uORB topic/type mismatch: requested topic '$(topic)' for type $(T), " *
                "but generated trait says topic '$(exp_topic)'.",
            )
        catch
            # If `uorb_topic(T)` isn't defined (stale generated file), we'll
            # fail below when checking for missing traits.
        end
    end

    # Require generator-provided traits.
    if !(@isdefined(uorb_fields_hash) && @isdefined(uorb_fields))
        error(
            "uORB contract traits are missing (uorb_fields_hash/uorb_fields not defined). " *
            "Regenerate Tools/px4_lockstep_julia/src/UORBGenerated.jl via scripts/uorb_codegen.jl.",
        )
    end

    exp_hash = try
        UInt64(uorb_fields_hash(T))
    catch err
        error(
            "uORB contract trait uorb_fields_hash(::Type{$(T)}) is missing. " *
            "Regenerate UORBGenerated.jl. (inner error: $(err))",
        )
    end
    exp_fields = try
        String(uorb_fields(T))
    catch err
        error(
            "uORB contract trait uorb_fields(::Type{$(T)}) is missing. " *
            "Regenerate UORBGenerated.jl. (inner error: $(err))",
        )
    end

    if isempty(exp_fields) || exp_hash == 0x0000000000000000
        error(
            "uORB contract traits for $(T) appear uninitialized (empty fields or zero hash). " *
            "This usually means UORBGenerated.jl is stale; regenerate it from your PX4 build.",
        )
    end

    px4_fields, px4_size, px4_size_no_padding, px4_message_hash, px4_queue_size =
        uorb_topic_metadata(handle, topic)

    julia_size = UInt32(sizeof(T))
    julia_size == px4_size || error(
        "uORB size mismatch for topic '$(topic)': Julia sizeof($(T)) = $(julia_size) bytes, " *
        "PX4 metadata size = $(px4_size) bytes (size_no_padding=$(px4_size_no_padding)).\n" *
        "Julia fields: $(exp_fields)\n" *
        "PX4 fields:   $(px4_fields)\n" *
        "Fix: re-run uorb_codegen against your PX4 build output.",
    )

    if !isempty(px4_fields)
        px4_hash = uorb_fields_hash_runtime(px4_fields)
        exp_hash == px4_hash || error(
            "uORB fields hash mismatch for topic '$(topic)' / type $(T).\n" *
            "Julia hash: 0x$(string(exp_hash, base=16, pad=16))\n" *
            "PX4  hash: 0x$(string(px4_hash, base=16, pad=16))\n" *
            "Julia fields: $(exp_fields)\n" *
            "PX4 fields:   $(px4_fields)\n" *
            "Fix: regenerate UORBGenerated.jl from the exact PX4 binary you are loading.",
        )
    else
        if !(@isdefined(uorb_message_hash))
            error(
                "PX4 uORB metadata does not expose field descriptions, and uorb_message_hash is not defined. " *
                "Regenerate UORBGenerated.jl with message hashes enabled.",
            )
        end

        exp_message_hash = try
            UInt32(uorb_message_hash(T))
        catch err
            error(
                "uORB contract trait uorb_message_hash(::Type{$(T)}) is missing. " *
                "Regenerate UORBGenerated.jl. (inner error: $(err))",
            )
        end
        exp_message_hash != 0 || error(
            "uORB message_hash for $(T) is zero; regenerate UORBGenerated.jl to include message hashes.",
        )
        px4_message_hash != 0 || error(
            "PX4 metadata returned message_hash=0 for topic '$(topic)'; cannot enforce layout compatibility.",
        )
        exp_message_hash == px4_message_hash || error(
            "uORB message_hash mismatch for topic '$(topic)' / type $(T).\n" *
            "Julia hash: 0x$(string(exp_message_hash, base=16, pad=8))\n" *
            "PX4  hash: 0x$(string(px4_message_hash, base=16, pad=8))\n" *
            "Fix: regenerate UORBGenerated.jl from the exact PX4 binary you are loading.",
        )
    end

    _UORB_CONTRACT_CACHE[key] = true
    return nothing
end

"""Verify a uORB type using its generated topic mapping.

This is a convenience wrapper around `verify_uorb_type!(handle, topic, T)`.
"""
function verify_uorb_type!(handle::LockstepHandle, ::Type{T}) where {T}
    @isdefined(uorb_topic) || error(
        "uorb_topic is not defined; regenerate UORBGenerated.jl via scripts/uorb_codegen.jl.",
    )
    topic = uorb_topic(T)
    return verify_uorb_type!(handle, topic, T)
end

"""Verify a set of uORB message types against the loaded PX4 binary.

By default, validates `UORB_ALL_TYPES` generated by `uorb_codegen.jl`.
"""
function verify_uorb_contract!(handle::LockstepHandle; types = nothing)
    if types === nothing
        @isdefined(UORB_ALL_TYPES) || error(
            "UORB_ALL_TYPES is not defined. Regenerate UORBGenerated.jl via uorb_codegen.jl.",
        )
        types = UORB_ALL_TYPES
    end

    for T in types
        # Derive the topic name from traits if available.
        if !(@isdefined(uorb_topic))
            error("uorb_topic is not defined; cannot derive topic names for verification")
        end
        topic = uorb_topic(T)
        verify_uorb_type!(handle, topic, T)
    end
    return nothing
end

# Generated uORB message types + traits
include("UORBGenerated.jl")

"""Opaque publisher handle for uORB topics created through `libpx4_lockstep`.

The publisher only identifies the advertised topic and stores the expected message size.

Messages are *queued* and published on the next `step_uorb!()` call, after the lockstep time has
been advanced on the C side.
"""
struct UORBPublisher{T<:UORBMsg}
    id::Int32
    msg_size::UInt32
end

"""Opaque subscription handle for uORB topics created through `libpx4_lockstep`."""
struct UORBSubscriber{T<:UORBMsg}
    id::Int32
    msg_size::UInt32
end

"""Create a uORB publisher by topic name.

Returns `(pub, instance)`.

* If `instance == -1` (default), uORB assigns an instance on advertise, so the returned
  value is `-1` until the first publish/advertise.
* If `instance >= 0`, the publisher requests that instance deterministically. The request
  is enforced at advertise time: if uORB assigns a different instance, the next
  `step_uorb!()` fails fast.

`priority` should be one of the ORB_PRIO_* constants (see PX4 uORB API). If `priority <= 0`,
the C side uses ORB_PRIO_DEFAULT.

`queue_size` controls uORB internal queuing for the topic (1 disables queuing).
"""
function create_uorb_publisher(
    handle::LockstepHandle,
    topic::AbstractString;
    priority::Integer = 0,
    queue_size::Integer = 1,
    instance::Integer = -1,
)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_create_publisher_ex)
    pub_id = Ref{Int32}(-1)
    instance_out = Ref{Int32}(-1)
    msg_size = Ref{UInt32}(0)
    ret = ccall(
        fn,
        Cint,
        (Ptr{Cvoid}, Cstring, Int32, UInt32, Int32, Ref{Int32}, Ref{Int32}, Ref{UInt32}),
        handle.ptr,
        topic,
        Int32(priority),
        UInt32(queue_size),
        Int32(instance),
        pub_id,
        instance_out,
        msg_size,
    )
    ret == 0 ||
        error("px4_lockstep_orb_create_publisher_ex failed for $topic with code $ret")
    return UORBPublisher{UORBMsg}(pub_id[], msg_size[]), instance_out[]
end

"""Create a uORB publisher and validate the Julia struct size."""
function create_uorb_publisher_checked(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T};
    priority::Integer = 0,
    queue_size::Integer = 1,
    instance::Integer = -1,
) where {T}
    # Fail fast if the generated Julia type does not match the loaded PX4 binary.
    verify_uorb_type!(handle, topic, T)
    pub_any, instance_out = create_uorb_publisher(
        handle,
        topic;
        priority = priority,
        queue_size = queue_size,
        instance = instance,
    )
    n = UInt32(sizeof(T))
    n == pub_any.msg_size || error(
        "uORB msg size mismatch for $topic: got $n bytes, expected $(pub_any.msg_size)",
    )
    return UORBPublisher{T}(pub_any.id, pub_any.msg_size), instance_out
end

"""Create a uORB publisher using the generated topic traits.

This is the preferred, type-driven API. The topic name is obtained from
`uorb_topic(T)` and the default queue length comes from `uorb_queue_length(T)`.

Returns `(pub, instance)`.
"""
function create_publisher(
    handle::LockstepHandle,
    ::Type{T};
    priority::Integer = 0,
    queue_size::Union{Nothing,Integer} = nothing,
    instance::Integer = -1,
) where {T<:UORBMsg}
    q = isnothing(queue_size) ? Int(uorb_queue_length(T)) : Int(queue_size)
    topic = uorb_topic(T)
    return create_uorb_publisher_checked(
        handle,
        topic,
        T;
        priority = priority,
        queue_size = q,
        instance = instance,
    )
end

"""Queue a uORB publish for the next `step_uorb!()` call.

This enforces that the message type matches the publisher type parameter. For
untyped publishers, use `publish_unsafe!` (or `queue_uorb_publish!`).
"""
@inline function publish!(
    handle::LockstepHandle,
    pub::UORBPublisher{T},
    msg::T,
) where {T<:UORBMsg}
    return queue_uorb_publish!(handle, pub, msg)
end

@inline function publish!(
    handle::LockstepHandle,
    pub::UORBPublisher{UORBMsg},
    msg::T,
) where {T<:UORBMsg}
    error(
        "publish! requires a typed UORBPublisher{T}. Use create_publisher or call publish_unsafe!/queue_uorb_publish! for raw usage.",
    )
end

"""Queue a uORB publish for the next `step_uorb!()` call (unsafe).

This bypasses the typed publisher check and should only be used for debugging or
raw/experimental topics.
"""
@inline function publish_unsafe!(
    handle::LockstepHandle,
    pub::UORBPublisher,
    msg::T,
) where {T}
    return queue_uorb_publish!(handle, pub, msg)
end

"""Queue a uORB publish for the next `step_uorb!()` call.

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
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32, Ref{T}, UInt32), handle.ptr, pub.id, msg, n)
    ret == 0 || error("px4_lockstep_orb_queue_publish failed with code $ret")
    return nothing
end

"""Queue a uORB publish from raw bytes for the next `step_uorb!()` call."""
function queue_uorb_publish!(
    handle::LockstepHandle,
    pub::UORBPublisher,
    bytes::Vector{UInt8},
)
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
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32, Ref{Int32}), handle.ptr, pub.id, instance)
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
    return UORBSubscriber{UORBMsg}(sub_id[], msg_size[])
end

"""Create a uORB subscriber and validate Julia/PX4 layout compatibility.

This is the subscriber analogue of `create_uorb_publisher_checked`.
"""
function create_uorb_subscriber_checked(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T};
    instance::Integer = 0,
) where {T}
    verify_uorb_type!(handle, topic, T)
    sub_any = create_uorb_subscriber(handle, topic; instance = instance)
    n = UInt32(sizeof(T))
    n == sub_any.msg_size || error(
        "uORB msg size mismatch for $topic[$instance]: got $n bytes, expected $(sub_any.msg_size)",
    )
    return UORBSubscriber{T}(sub_any.id, sub_any.msg_size)
end

"""Create a uORB subscriber using the generated topic traits.

This is the preferred, type-driven API. The topic name is obtained from
`uorb_topic(T)`.
"""
function create_subscriber(
    handle::LockstepHandle,
    ::Type{T};
    instance::Integer = 0,
) where {T<:UORBMsg}
    topic = uorb_topic(T)
    return create_uorb_subscriber_checked(handle, topic, T; instance = instance)
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

"""Copy the latest topic data and return it (allocates one `Ref`).

This method infers the uORB message type from the subscriber handle.
"""
function uorb_copy(handle::LockstepHandle, sub::UORBSubscriber{T}) where {T<:UORBMsg}
    out = Ref{T}()
    uorb_copy!(handle, sub, out)
    return out[]
end

function uorb_copy(handle::LockstepHandle, sub::UORBSubscriber{UORBMsg})
    error(
        "uorb_copy requires a typed UORBSubscriber{T}. Use create_subscriber or call uorb_copy(handle, sub, T).",
    )
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

include("sim/Sim.jl")

end
