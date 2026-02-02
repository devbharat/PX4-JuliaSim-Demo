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
