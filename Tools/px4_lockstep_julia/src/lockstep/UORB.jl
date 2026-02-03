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

"""Internal: create a uORB publisher by topic name."""
function _create_uorb_publisher(
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

"""Internal: create a uORB publisher and validate the Julia struct size."""
function _create_uorb_publisher_checked(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T};
    priority::Integer = 0,
    queue_size::Integer = 1,
    instance::Integer = -1,
) where {T}
    # Fail fast if the generated Julia type does not match the loaded PX4 binary.
    verify_uorb_type!(handle, topic, T)
    pub_any, instance_out = _create_uorb_publisher(
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
Queue length is validated against the PX4 topic metadata; it cannot be overridden
at runtime in the current C API.

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
    return _create_uorb_publisher_checked(
        handle,
        topic,
        T;
        priority = priority,
        queue_size = q,
        instance = instance,
    )
end

"""Queue a uORB publish for the next `step_uorb!()` call.

This enforces that the message type matches the publisher type parameter.
"""
@inline function publish!(
    handle::LockstepHandle,
    pub::UORBPublisher{T},
    msg::T,
) where {T<:UORBMsg}
    return _queue_uorb_publish!(handle, pub, msg)
end

@inline function publish!(
    handle::LockstepHandle,
    pub::UORBPublisher{UORBMsg},
    msg::T,
) where {T<:UORBMsg}
    error(
        "publish! requires a typed UORBPublisher{T}. Use create_publisher to construct one.",
    )
end

"""Queue a uORB publish for the next `step_uorb!()` call.

The message is copied into the C side immediately, so the Julia value does not need to live
until the next tick.

`msg` must be an `isbits` struct that matches the uORB C layout exactly.
"""
function _queue_uorb_publish!(handle::LockstepHandle, pub::UORBPublisher, msg::T) where {T}
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

"""Get the uORB instance id assigned to a publisher (assigned on advertise)."""
function uorb_publisher_instance(handle::LockstepHandle, pub::UORBPublisher)
    fn = _resolve_symbol(handle.lib, :px4_lockstep_orb_publisher_instance)
    instance = Ref{Int32}(-1)
    ret = ccall(fn, Cint, (Ptr{Cvoid}, Int32, Ref{Int32}), handle.ptr, pub.id, instance)
    ret == 0 || error("px4_lockstep_orb_publisher_instance failed with code $ret")
    return instance[]
end

"""Internal: create a uORB subscriber by topic name and instance."""
function _create_uorb_subscriber(
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

"""Internal: create a uORB subscriber and validate Julia/PX4 layout compatibility."""
function _create_uorb_subscriber_checked(
    handle::LockstepHandle,
    topic::AbstractString,
    ::Type{T};
    instance::Integer = 0,
) where {T}
    verify_uorb_type!(handle, topic, T)
    sub_any = _create_uorb_subscriber(handle, topic; instance = instance)
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
    return _create_uorb_subscriber_checked(handle, topic, T; instance = instance)
end

"""Return whether a subscription has new data (non-allocating).

Pass a preallocated `updated` Ref to avoid per-call allocations.
"""
function uorb_check!(
    handle::LockstepHandle,
    sub::UORBSubscriber,
    updated::Base.RefValue{Int32},
)::Bool
    updated[] = 0
    ret = ccall(
        handle.fns.orb_check,
        Cint,
        (Ptr{Cvoid}, Int32, Ref{Int32}),
        handle.ptr,
        sub.id,
        updated,
    )
    ret == 0 || error("px4_lockstep_orb_check failed with code $ret")
    return updated[] != 0
end

"""Copy the latest topic data into `out` (no allocation)."""
function uorb_copy!(
    handle::LockstepHandle,
    sub::UORBSubscriber{T},
    out::Ref{T},
) where {T<:UORBMsg}
    isbitstype(T) || error("uORB messages must be isbits structs (got $T)")
    n = UInt32(sizeof(T))
    n == sub.msg_size || error(
        "uORB msg size mismatch for sub $(sub.id): got $n bytes, expected $(sub.msg_size)",
    )
    ret = ccall(
        handle.fns.orb_copy,
        Cint,
        (Ptr{Cvoid}, Int32, Ref{T}, UInt32),
        handle.ptr,
        sub.id,
        out,
        n,
    )
    ret == 0 || error("px4_lockstep_orb_copy failed with code $ret")
    return nothing
end

"""Unsubscribe and free the underlying uORB handle."""
function uorb_unsubscribe!(handle::LockstepHandle, sub::UORBSubscriber)
    ret =
        ccall(handle.fns.orb_unsubscribe, Cint, (Ptr{Cvoid}, Int32), handle.ptr, sub.id)
    ret == 0 || error("px4_lockstep_orb_unsubscribe failed with code $ret")
    return nothing
end
