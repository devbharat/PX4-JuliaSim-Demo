# Julia Performance Design Guidelines (PX4 Lockstep Sim)

**Status:** Draft  
**Last updated:** 2026-01-23  
**Scope:** This document defines performance-oriented design rules for this repository, with a focus on deterministic, low-allocation, low-jitter simulation and PX4 lockstep integration.  
**Note:** This is an internal draft; if it conflicts with `docs/performance_guidelines.md`, treat that file as canonical.

---

## 1. Goals and non-goals

### Goals
- **Deterministic throughput and low jitter** on “tick-rate” code paths (plant RHS, autopilot tick, uORB bridge).
- **Zero allocations in hot paths** (or explicitly justified exceptions).
- **Type-stable hot paths** (no dynamic dispatch once initialized).
- **Maintainability**: performance comes from *structure*, not scattered micro-optimizations.

### Non-goals
- Making all code “maximally optimized”. Cold-path code (parsing configs, plotting, reporting, one-time initialization) can prioritize clarity.
- Premature micro-tuning without measurement.

---

## 2. Vocabulary: hot vs cold paths

### Hot paths (must follow the strict rules)
Code executed at a fixed cadence or inside tight loops, e.g.:
- Plant RHS / dynamics evaluation (integrator callback)
- Actuator update + motor/prop models
- Autopilot tick and uORB I/O
- Event scheduler boundary processing (if evaluated frequently)
- Recording/logging *when enabled at tick rate*

### Cold paths (follow normal clean-code rules; performance is secondary)
- Aircraft/spec builders
- Scenario creation
- One-time symbol resolution / library loading
- Post-processing, plotting, file writing

**Rule of thumb:** If it runs every tick, treat it as hot until proven otherwise.

---

## 3. Performance contracts (hard requirements)

### Contract A — “no allocations”
Hot-path functions must allocate **0 bytes** in steady state.

Enforcement patterns:
- Provide an `@allocated` test for each hot call site (RHS, tick, uORB IO).
- Preallocate all buffers during initialization.

### Contract B — “no runtime reflection”
Hot-path code must not use:
- `applicable`, `hasmethod`, `fieldnames`, `propertynames`, `getproperty`-based introspection
- `eval` / runtime codegen
- `try/catch` for control flow

If capability detection is needed, do it **once at init** and store flags.

### Contract C — “monomorphic hot loops”
All hot loops must operate on **concrete types**:
- No `Any` fields read on hot paths.
- No `Vector{AbstractType}` / `Dict{Symbol,Any}` in hot loops.
- Avoid calling methods that return `Any`.

---

## 4. Type system rules (the biggest lever)

### 4.1 Struct fields must be concrete on hot paths
**Bad**
```julia
mutable struct EngineOutputs
    plant_y::Any
end
```

**Good**
```julia
Base.@kwdef mutable struct EngineOutputs{Y}
    plant_y::Union{Nothing,Y} = nothing
end
```

If a field must be optional: use `Union{Nothing,T}`.

### 4.2 Avoid abstract-typed containers in hot loops
**Bad**
```julia
units::Vector{MotorPropUnit}   # element type is a UnionAll / abstract
events::Vector{AbstractEvent}
subs::Dict{Symbol,UORBSubscriber}
values::Dict{Symbol,Any}
```

**Good**
- Parameterize on concrete element type:
```julia
mutable struct QuadRotorSet{N,U<:MotorPropUnit}
    units::Vector{U}
end
```
- Or freeze the collection into a tuple / NamedTuple:
```julia
struct Subscriptions{S<:NamedTuple}
    subs::S
end
# access: subs.subs.actuator_motors
```

### 4.3 Prefer “freeze config into typed runtime”
If you start with user-facing flexibility (`Dict`, JSON, etc.), convert to a **typed runtime struct** at build/init time.

Pattern:
- `Spec` (flexible, abstract OK) → `Runtime` (concrete, hot-loop safe)

---

## 5. Function barriers and dispatch patterns

### 5.1 Use function barriers to isolate dynamic behavior
If you must accept dynamic inputs, do it once and call a type-stable inner function:

```julia
function step!(sim, cfg::Dict)
    rt = build_runtime(cfg)   # cold path
    return step_runtime!(sim, rt)  # hot path
end
```

### 5.2 Prefer dispatch over reflection
Instead of `hasproperty(x, :rb)` or `isa` ladders, define overloads:
```julia
_rb_state(x::PlantState) = x.rb
_rb_state(x::RigidBodyState) = x
```

### 5.3 Beware specialization blow-up
Type parameters are powerful but can explode compile time/code size if you parameterize on:
- closures / anonymous functions
- large tuples with many unique element types
- “unbounded” user-generated types

Mitigation:
- Keep runtime structs parametric on *stable component types* (motor model type, prop model type), not on ad-hoc closures.
- For event actions/conditions, consider an enum + data struct, or a single `Function` behind a function barrier when the action set is truly dynamic.

---

## 6. Allocations: rules and standard techniques

### 6.1 Preallocate message buffers and reuse them
Avoid per-tick `Ref{T}()` construction (or any heap allocation).

Pattern:
```julia
struct UORBSubSlot{T}
    sub::UORBSubscriber{T}
    buf::Base.RefValue{T}   # persistent
end
```

Then only call `uorb_copy!` into `buf`.

### 6.2 Avoid `push!` growth in long runs
For fixed-rate traces (known length), preallocate vectors and write by index.
- If the timeline is known (e.g., time axis), allocate once.
- Keep debug “growable” recorders separate from production recorders.

### 6.3 Avoid temporary arrays in math
- Prefer `StaticArrays` for tiny fixed-size vectors/matrices.
- Use `@views` for slicing (but validate it doesn’t inhibit inference).
- Use in-place operations where appropriate (`mul!`, `copyto!`, `uorb_copy!`).

---

## 7. StaticArrays guidelines

### Use `StaticArrays` when
- Size is small and fixed (e.g., 3, 4, 6, 12).
- You want stack allocation and aggressive inlining.

Avoid `StaticArrays` when
- Size is large or variable (code size and compile time grow quickly).
- You’re doing large linear algebra (BLAS-friendly `Matrix`/`Vector` is better).

Broadcasting over `SVector` generally does not allocate, but:
- Prefer plain scalar ops (`*`, `+`, `-`) instead of `.*`/`.+=` for scalar–vector operations to reduce broadcast machinery.
- Keep expressions simple to help LLVM.

---

## 8. C/FFI (PX4 lockstep) rules

### 8.1 Cache function pointers at initialization
Never call `dlsym`/symbol resolution in hot loops.

Pattern:
```julia
struct LockstepFns
    step_uorb::Ptr{Cvoid}
    uorb_check::Ptr{Cvoid}
    uorb_copy::Ptr{Cvoid}
end

mutable struct LockstepHandle
    ptr::Ptr{Cvoid}
    lib::Ptr{Cvoid}
    fns::LockstepFns
end
```

### 8.2 Avoid `try/catch` in hot wrappers
- Resolve and validate at init.
- In hot calls, assume pointers exist and are valid.

Put error paths behind `@noinline` helpers to keep hot path small.

### 8.3 Keep argument types concrete and predictable
- Avoid passing `Any` or abstract Julia objects across the FFI boundary.
- Prefer passing `Ptr`, `Ref`, primitive numeric types, and small immutable structs.
- Avoid per-call conversion of `Symbol`/`String` in hot paths.

---

## 9. Exceptions, assertions, and validation

### Hot path
- No `try/catch` for expected conditions.
- No heavy `@assert` checks per tick.
- Validate invariants once when building the runtime objects.

### Cold path
- Use assertions and argument checks freely to ensure correctness.

---

## 10. Logging and recording

### 10.1 Separate “debug recorder” from “production recorder”
- Debug: flexible (`Dict{Symbol,Any}`), easy to extend, OK to allocate.
- Production: schema-based, concrete vectors/structs, no allocations per sample.

### 10.2 Avoid keyword-heavy logging in tick loops
Prefer:
```julia
struct LogSample
    t_us::UInt64
    # concrete fields...
end

log!(sink, sample::LogSample)
```

### 10.3 Make logging opt-in and cheap when disabled
- Guard with `if log_enabled` checks that the compiler can constant-propagate when possible.
- Avoid constructing log payloads if no sinks are active.

---

## 11. Event scheduling

### 11.1 Avoid scanning all events each boundary
If events grow large:
- Store `AtTime` events sorted and advance an index cursor (O(1) amortized).
- Keep `When` events separate (scan only those).

### 11.2 Validate event signatures at insertion time
Do not call `applicable(...)` every boundary. Validate once during event registration.

---

## 12. Measurement and tooling (required workflow)

### 12.1 Minimum required checks for PRs that touch hot paths
- `@code_warntype` on the hot functions shows no inference failures.
- `@allocated` equals 0 for hot-path call sites in steady state.
- A micro-benchmark demonstrates no regression.

### 12.2 Recommended tools (keep dependencies minimal)
- **Built-in:** `@time`, `@allocated`, `@code_warntype`, `Profile`
- **Dev dependency:** `BenchmarkTools` for stable benchmarking
- Optional (only if needed): `SnoopCompile` for invalidations / compile-time issues, `JET` for inference diagnostics

### 12.3 Benchmark hygiene
- Always interpolate globals in `@btime`: `@btime f($x, $y)`
- Benchmark steady state (after initialization / warmup).
- Measure both time and allocations.

---

## 13. Code review checklist (copy/paste)

### Hot-path checklist
- [ ] No `Any` fields read in hot path.
- [ ] No abstract-typed fields (`::AbstractFoo`) used in hot path.
- [ ] No `Vector{AbstractFoo}` / `Dict{Symbol,Any}` accessed in hot loops.
- [ ] No runtime reflection (`applicable`, `hasmethod`, etc.) in hot loop.
- [ ] No `try/catch` in hot loop.
- [ ] `@allocated` == 0 for the critical function(s).
- [ ] `@code_warntype` is clean for the critical function(s).
- [ ] FFI calls use cached function pointers and reuse buffers.

### Cold-path checklist
- [ ] Correctness and clarity prioritized.
- [ ] Any dynamic structures are converted to typed runtime structures before entering hot loops.

---

## 14. Appendix: examples to emulate and avoid

### Good patterns to emulate
- Type-driven actuator dynamics via multiple dispatch (direct/first-order/second-order).
- Parametric runtime structs that store concrete component types.
- “Output type guardrails” where the runtime caches a concrete output type.

### Patterns to avoid on hot paths
- Abstract wind stored as `mean::AbstractWind` (should be `GustStep{W}` with `mean::W`).
- Parametric element types erased in containers (`Vector{MotorPropUnit}` instead of `Vector{U}`).
- Dict-based topic access in per-tick uORB reads (freeze into typed subscription sets).
- Per-call symbol resolution for `ccall` targets.

---

## 15. Practical default: how to decide what to optimize
1. Identify hot paths (tick-rate loops).
2. Enforce contracts (type stability + zero allocations).
3. Remove reflection/dynamic dispatch.
4. Only then do math-level tuning.

If you’re tempted to add an optimization that complicates the API, add a **fast path** behind the same interface and keep the slow path for flexibility/testing.
