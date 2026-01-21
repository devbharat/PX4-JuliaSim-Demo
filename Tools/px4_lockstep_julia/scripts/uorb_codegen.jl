#!/usr/bin/env julia

using Printf

const USAGE = """
Usage:
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/uorb_codegen.jl \
    --headers <uorb_header_dir> --topics <topic1,topic2,...> [--out <path>] [--suffix <name>]

Examples:
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/uorb_codegen.jl \
    --headers build/px4_sitl_lockstep/uORB/topics \
    --topics battery_status,vehicle_attitude \
    --out Tools/px4_lockstep_julia/src/UORBGenerated.jl
"""

const TYPE_MAP = Dict(
    "uint64_t" => "UInt64",
    "int64_t" => "Int64",
    "uint32_t" => "UInt32",
    "int32_t" => "Int32",
    "uint16_t" => "UInt16",
    "int16_t" => "Int16",
    "uint8_t" => "UInt8",
    "int8_t" => "Int8",
    "float" => "Float32",
    "double" => "Float64",
    "bool" => "Bool",
    "char" => "UInt8",
)

# ---------------------------------------------------------------------------
# Canonicalization + hashing (for contract verification)
# ---------------------------------------------------------------------------

"""Canonicalize a uORB fields string for stable hashing.

We intentionally remove *all* whitespace and any trailing semicolons. This makes
it robust to formatting differences between generated headers and PX4 runtime
metadata.
"""
function canonicalize_uorb_fields(s::AbstractString)
    # Remove all whitespace.
    t = replace(s, r"\s+" => "")

    # Drop trailing semicolons (common in PX4 metadata strings).
    t = replace(t, r";+$" => "")
    return t
end

const _FNV1A_64_OFFSET_BASIS = UInt64(0xcbf29ce484222325)
const _FNV1A_64_PRIME = UInt64(0x100000001b3)

"""Compute a stable FNV-1a 64-bit hash of a string."""
function fnv1a64(s::AbstractString)
    h = _FNV1A_64_OFFSET_BASIS
    for b in codeunits(s)
        h = (h ⊻ UInt64(b)) * _FNV1A_64_PRIME
    end
    return h
end

function camelcase(name::AbstractString)
    parts = split(name, '_')
    return join(uppercasefirst.(lowercase.(parts)))
end

function parse_args(args)
    headers_dir = ""
    topics = String[]
    out_path = ""
    suffix = "Msg"
    i = 1
    while i <= length(args)
        arg = args[i]
        if arg == "--headers" || arg == "--header-dir"
            i += 1
            i <= length(args) || error("Missing value for --headers\n\n$USAGE")
            headers_dir = args[i]
        elseif arg == "--topics"
            i += 1
            i <= length(args) || error("Missing value for --topics\n\n$USAGE")
            topics = filter(!isempty, split(args[i], ','))
        elseif arg == "--topic"
            i += 1
            i <= length(args) || error("Missing value for --topic\n\n$USAGE")
            push!(topics, args[i])
        elseif arg == "--out"
            i += 1
            i <= length(args) || error("Missing value for --out\n\n$USAGE")
            out_path = args[i]
        elseif arg == "--suffix"
            i += 1
            i <= length(args) || error("Missing value for --suffix\n\n$USAGE")
            suffix = args[i]
        elseif arg == "--help" || arg == "-h"
            println(USAGE)
            exit(0)
        else
            error("Unknown argument: $arg\n\n$USAGE")
        end
        i += 1
    end

    isempty(headers_dir) && error("Missing --headers\n\n$USAGE")
    isempty(topics) && error("Missing --topics\n\n$USAGE")
    return headers_dir, topics, out_path, suffix
end

function find_struct_start(lines::Vector{String}, topic::AbstractString)
    sig_cpp = "struct __EXPORT $(topic)_s"
    sig_c = "struct $(topic)_s"
    for (idx, line) in enumerate(lines)
        if occursin(sig_cpp, line) || occursin(sig_c, line)
            return idx
        end
    end
    return nothing
end

struct ParsedField
    name::String
    c_type::String
    arr_len::Union{Nothing, Int}
    j_type::String
end

function parse_queue_length(lines::Vector{String})
    # Typical patterns:
    #   static constexpr uint32_t ORB_QUEUE_LENGTH = 8;
    #   enum { ORB_QUEUE_LENGTH = 8 };
    for raw in lines
        raw = strip(split(raw, "//")[1])
        isempty(raw) && continue
        startswith(raw, "#") && continue
        m = match(r"ORB_QUEUE_LENGTH\s*=\s*(\d+)", raw)
        if m !== nothing
            return parse(Int, m.captures[1])
        end
    end
    return 1
end

function parse_orb_define_metadata(topic::AbstractString, topics_sources_dir::AbstractString)
    path = joinpath(topics_sources_dir, "$(topic).cpp")
    isfile(path) || return nothing, nothing
    for raw in readlines(path)
        m = match(
            r"ORB_DEFINE\(\s*" * topic *
                r"\s*,\s*[^,]+,\s*(\d+)\s*,\s*(\d+)u?,\s*[^,]+,\s*(\d+)\s*\)",
            raw,
        )
        if m !== nothing
            msg_hash = parse(UInt32, m.captures[2])
            queue_len = parse(Int, m.captures[3])
            return msg_hash, queue_len
        end
    end
    return nothing, nothing
end

function parse_struct_fields(lines::Vector{String}, start_idx::Int)
    fields = ParsedField[]
    for i in (start_idx + 1):length(lines)
        raw = strip(lines[i])
        if occursin("};", raw)
            break
        end
        raw = strip(split(raw, "//")[1])
        isempty(raw) && continue
        startswith(raw, "#") && continue
        m = match(r"^([A-Za-z_][A-Za-z_0-9]*)\s+([A-Za-z_][A-Za-z_0-9]*)(?:\[(\d+)\])?;", raw)
        m === nothing && continue
        c_type, name, arr = m.captures
        j_type = get(TYPE_MAP, c_type, nothing)
        j_type === nothing && error("Unknown C type '$c_type' in line: $raw")
        arr_len = nothing
        if arr !== nothing
            arr_len = parse(Int, arr)
            j_type = "NTuple{$(arr_len),$(j_type)}"
        end

        push!(fields, ParsedField(String(name), String(c_type), arr_len, String(j_type)))
    end
    return fields
end

function build_uorb_fields_string(fields::Vector{ParsedField})
    # Match PX4 uORB metadata convention:
    #   <type> <name>; or <type>[N] <name>;
    # Exclude any padding fields inserted for alignment.
    parts = String[]
    for f in fields
        startswith(f.name, "_padding") && continue
        if f.arr_len === nothing
            push!(parts, "$(f.c_type) $(f.name)")
        else
            push!(parts, "$(f.c_type)[$(f.arr_len)] $(f.name)")
        end
    end
    return join(parts, ";")
end

function generate_struct(
    io,
    topic::AbstractString,
    header_path::AbstractString,
    suffix::AbstractString,
    topics_sources_dir::AbstractString,
)
    lines = readlines(header_path)
    start_idx = find_struct_start(lines, topic)
    start_idx === nothing && error("Struct for $topic not found in $header_path")
    queue_len = parse_queue_length(lines)
    msg_hash, queue_len_orb = parse_orb_define_metadata(topic, topics_sources_dir)
    if queue_len_orb !== nothing
        queue_len = queue_len_orb
    end
    fields = parse_struct_fields(lines, start_idx)
    struct_name = camelcase(topic) * suffix
    uorb_fields = build_uorb_fields_string(fields)
    uorb_fields_hash = fnv1a64(canonicalize_uorb_fields(uorb_fields))

    println(io, "struct $(struct_name) <: UORBMsg")
    for f in fields
        println(io, "    $(f.name)::$(f.j_type)")
    end
    println(io, "end")
    println(io)

    # Traits (generated contract)
    println(io, "uorb_topic(::Type{$(struct_name)}) = \"$(topic)\"")
    println(io, "uorb_queue_length(::Type{$(struct_name)}) = $(queue_len)")
    @printf(io, "uorb_fields_hash(::Type{%s}) = 0x%016x\n", struct_name, uorb_fields_hash)
    if msg_hash === nothing
        println(io, "uorb_message_hash(::Type{$(struct_name)}) = 0x00000000")
    else
        @printf(io, "uorb_message_hash(::Type{%s}) = 0x%08x\n", struct_name, msg_hash)
    end
    println(io, "uorb_fields(::Type{$(struct_name)}) = $(repr(uorb_fields))")
    println(io)

    return struct_name
end

function main(args)
    headers_dir, topics, out_path, suffix = parse_args(args)
    io = isempty(out_path) ? stdout : open(out_path, "w")
    try
        headers_display = isabspath(headers_dir) ? relpath(headers_dir, pwd()) : headers_dir
        println(io, "# Auto-generated from PX4 uORB headers")
        println(io, "# headers: $(headers_display)")
        println(io, "# topics: $(join(topics, ", "))")
        println(io)

        # Base marker type for generated uORB message structs
        println(io, "abstract type UORBMsg end")
        println(io)

        # Optional trait fallbacks (useful error messages if a topic isn't generated)
        println(io, "uorb_topic(::Type{T}) where {T<:UORBMsg} = error(\"uorb_topic not defined for \" * string(T))")
        println(io, "uorb_queue_length(::Type{T}) where {T<:UORBMsg} = 1")
        println(io, "uorb_fields_hash(::Type{T}) where {T<:UORBMsg} = 0x0000000000000000")
        println(io, "uorb_message_hash(::Type{T}) where {T<:UORBMsg} = 0x00000000")
        println(io, "uorb_fields(::Type{T}) where {T<:UORBMsg} = \"\"")
        println(io)

        struct_names = String[]
        topics_sources_dir = normpath(joinpath(headers_dir, "..", "..", "msg", "topics_sources"))
        for topic in topics
            header_path = joinpath(headers_dir, "$(topic).h")
            isfile(header_path) || error("Header not found: $header_path")
            push!(
                struct_names,
                generate_struct(io, topic, header_path, suffix, topics_sources_dir),
            )
        end

        # Convenience for bulk verification in later phases
        if length(struct_names) == 1
            println(io, "const UORB_ALL_TYPES = (", struct_names[1], ",)")
        else
            println(io, "const UORB_ALL_TYPES = (", join(struct_names, ", "), ")")
        end
    finally
        if io !== stdout
            close(io)
        end
    end
    return nothing
end

main(ARGS)
