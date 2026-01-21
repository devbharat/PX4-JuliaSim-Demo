#!/usr/bin/env julia

const USAGE = """
Usage:
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/verify_uorb_contract.jl \
    [--lib <path/to/libpx4_lockstep.so|.dylib>] [--topics <topic1,topic2,...>]

Examples:
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/verify_uorb_contract.jl

  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/verify_uorb_contract.jl \
    --lib build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.dylib

  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/verify_uorb_contract.jl \
    --topics battery_status,vehicle_attitude
"""

import Pkg

const PROJECT_ROOT = normpath(joinpath(@__DIR__, ".."))
Pkg.activate(PROJECT_ROOT; io = devnull)

using PX4Lockstep

function parse_args(args)
    libpath = nothing
    topics = nothing
    i = 1
    while i <= length(args)
        arg = args[i]
        if arg == "--lib"
            i += 1
            i <= length(args) || error("Missing value for --lib\n\n$USAGE")
            libpath = args[i]
        elseif arg == "--topics"
            i += 1
            i <= length(args) || error("Missing value for --topics\n\n$USAGE")
            topics = filter(!isempty, split(args[i], ','))
        elseif arg == "--help" || arg == "-h"
            println(USAGE)
            exit(0)
        else
            error("Unknown argument: $arg\n\n$USAGE")
        end
        i += 1
    end
    return libpath, topics
end

function topic_type_map(types)
    m = Dict{String,Any}()
    for T in types
        topic = PX4Lockstep.uorb_topic(T)
        m[String(topic)] = T
    end
    return m
end

function main(args)
    libpath, topics = parse_args(args)
    handle = PX4Lockstep.create(; libpath = libpath)
    try
        types = if topics === nothing
            PX4Lockstep.UORB_ALL_TYPES
        else
            map = topic_type_map(PX4Lockstep.UORB_ALL_TYPES)
            out = Any[]
            for topic in topics
                haskey(map, topic) || error("Unknown topic '$topic' (not in UORB_ALL_TYPES)")
                push!(out, map[topic])
            end
            Tuple(out)
        end

        PX4Lockstep.verify_uorb_contract!(handle; types = types)
        println("uORB contract verified for $(length(types)) topic(s).")
    finally
        PX4Lockstep.destroy(handle)
    end
    return nothing
end

main(ARGS)
