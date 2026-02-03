# -----------------------------------------------------------------------------
# uORB interface configuration
# -----------------------------------------------------------------------------

abstract type AbstractUORBBackend end

struct RealUORBBackend <: AbstractUORBBackend end

mutable struct MockUORBBackend <: AbstractUORBBackend
    latest::Dict{Tuple{Symbol,Int32},Any}
    gen::Dict{Tuple{Symbol,Int32},UInt64}
end

MockUORBBackend() =
    MockUORBBackend(Dict{Tuple{Symbol,Int32},Any}(), Dict{Tuple{Symbol,Int32},UInt64}())

mutable struct MockPublisher
    key::Symbol
    instance::Int32
end

mutable struct MockSubscriber{T<:UORBMsg}
    key::Symbol
    instance::Int32
    last_gen::UInt64
end

"""Spec for a uORB publisher created by the Julia simulator.

Fields
------
- `key`       : Symbol used internally by the bridge (not a uORB topic name).
- `type`      : Generated uORB message type (subtype of `PX4Lockstep.UORBMsg`).
- `instance`  : uORB instance (-1 = auto).
- `priority`  : publisher priority passed to PX4 (uORB advertise).
- `queue_size`: uORB queue length (nothing -> use `uorb_queue_length(type)`).
"""
Base.@kwdef struct UORBPubSpec
    key::Symbol
    type::DataType
    instance::Int32 = -1
    priority::Int32 = 0
    queue_size::Union{Nothing,Int32} = nothing
end

"""Spec for a uORB subscriber created by the Julia simulator."""
Base.@kwdef struct UORBSubSpec
    key::Symbol
    type::DataType
    instance::UInt32 = UInt32(0)
end

"""Explicit uORB boundary contract between Julia and PX4."""
Base.@kwdef struct PX4UORBInterfaceConfig
    # publishers: what Julia will inject into PX4
    pubs::Vector{UORBPubSpec} = UORBPubSpec[]
    # subscribers: what Julia will read from PX4
    subs::Vector{UORBSubSpec} = UORBSubSpec[]
end

@inline function _backend_create_publisher(
    ::RealUORBBackend,
    handle::LockstepHandle,
    _key::Symbol,
    ::Type{T};
    priority::Int32,
    queue_size,
    instance::Int32,
) where {T<:UORBMsg}
    return create_publisher(
        handle,
        T;
        priority = priority,
        queue_size = queue_size,
        instance = instance,
    )
end

@inline function _backend_create_subscriber(
    ::RealUORBBackend,
    handle::LockstepHandle,
    _key::Symbol,
    ::Type{T};
    instance::Int32,
) where {T<:UORBMsg}
    return create_subscriber(handle, T; instance = instance)
end

@inline function _backend_publish!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    pub::UORBPublisher,
    msg,
)
    return publish!(handle, pub, msg)
end

@inline function _backend_check!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber,
    updated::Base.RefValue{Int32},
)::Bool
    return uorb_check!(handle, sub, updated)
end

@inline function _backend_copy!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber{T},
    buf::Base.RefValue{T},
) where {T<:UORBMsg}
    return uorb_copy!(handle, sub, buf)
end

@inline function _backend_unsubscribe!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber,
)
    return uorb_unsubscribe!(handle, sub)
end

@inline function _backend_create_publisher(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    key::Symbol,
    ::Type{T};
    priority::Int32,
    queue_size,
    instance::Int32,
) where {T<:UORBMsg}
    return MockPublisher(key, instance), instance
end

@inline function _backend_create_subscriber(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    key::Symbol,
    ::Type{T};
    instance::Int32,
) where {T<:UORBMsg}
    return MockSubscriber{T}(key, instance, UInt64(0))
end

@inline function _backend_publish!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    pub::MockPublisher,
    msg,
)
    key = (pub.key, pub.instance)
    backend.latest[key] = msg
    backend.gen[key] = get(backend.gen, key, UInt64(0)) + UInt64(1)
    return nothing
end

@inline function _backend_check!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    sub::MockSubscriber,
    updated::Base.RefValue{Int32},
)::Bool
    key = (sub.key, sub.instance)
    gen = get(backend.gen, key, UInt64(0))
    updated[] = gen > sub.last_gen ? Int32(1) : Int32(0)
    return updated[] != 0
end

@inline function _backend_copy!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    sub::MockSubscriber{T},
    buf::Base.RefValue{T},
) where {T<:UORBMsg}
    key = (sub.key, sub.instance)
    msg = get(backend.latest, key, nothing)
    msg isa T || error("Mock uORB missing message for $(sub.key)[$(sub.instance)]")
    buf[] = msg
    sub.last_gen = get(backend.gen, key, sub.last_gen)
    return nothing
end

@inline function _backend_unsubscribe!(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    _sub::MockSubscriber,
)
    return nothing
end

"""Publish a mock uORB message by key (for tests)."""
function mock_publish!(
    backend::MockUORBBackend,
    key::Symbol,
    msg;
    instance::Int32 = Int32(0),
)
    pub = MockPublisher(key, instance)
    _backend_publish!(
        backend,
        LockstepHandle(Ptr{Cvoid}(0), Ptr{Cvoid}(0), LockstepConfig()),
        pub,
        msg,
    )
    return nothing
end
