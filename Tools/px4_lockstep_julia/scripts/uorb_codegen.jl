#!/usr/bin/env julia

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

function parse_struct_fields(lines::Vector{String}, start_idx::Int)
    fields = Vector{Tuple{String,String}}()
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
        if arr !== nothing
            j_type = "NTuple{$(arr),$(j_type)}"
        end
        push!(fields, (name, j_type))
    end
    return fields
end

function generate_struct(io, topic::AbstractString, header_path::AbstractString, suffix::AbstractString)
    lines = readlines(header_path)
    start_idx = find_struct_start(lines, topic)
    start_idx === nothing && error("Struct for $topic not found in $header_path")
    fields = parse_struct_fields(lines, start_idx)
    struct_name = camelcase(topic) * suffix
    println(io, "struct $(struct_name)")
    for (name, j_type) in fields
        println(io, "    $(name)::$(j_type)")
    end
    println(io, "end\n")
end

function main(args)
    headers_dir, topics, out_path, suffix = parse_args(args)
    io = isempty(out_path) ? stdout : open(out_path, "w")
    try
        println(io, "# Auto-generated from PX4 uORB headers")
        println(io, "# headers: $(headers_dir)")
        println(io, "# topics: $(join(topics, ", "))\n")
        for topic in topics
            header_path = joinpath(headers_dir, "$(topic).h")
            isfile(header_path) || error("Header not found: $header_path")
            generate_struct(io, topic, header_path, suffix)
        end
    finally
        if io !== stdout
            close(io)
        end
    end
    return nothing
end

main(ARGS)
