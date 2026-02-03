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

# Cache of verified (lib,topic,type) pairs.
const _UORB_CONTRACT_CACHE = Dict{Tuple{Ptr{Cvoid},String,DataType},Bool}()

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
    key = (handle.lib, String(topic), T)
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
