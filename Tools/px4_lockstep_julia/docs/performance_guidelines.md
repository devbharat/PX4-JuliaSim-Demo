# Performance Guidelines (PX4 Lockstep Sim)

**Status:** Canonical
**Last updated:** 2026-02-02
**Scope:** Performance-oriented design rules for deterministic, low-allocation, low-jitter simulation and PX4 lockstep integration.

---

## 1. Goals and non-goals

### Goals
- **Deterministic throughput and low jitter** on tick-rate code paths (plant RHS, autopilot tick, uORB bridge).
- **Allocation-free hot paths** (0 bytes in steady state; exceptions must be explicitly justified).
- **Type-stable hot paths** (no dynamic dispatch once initialized).
- **Maintainability**: performance comes from structure, not scattered micro-optimizations.

### Non-goals
- Making all code maximally optimized. Cold-path code (parsing configs, plotting, reporting, one-time initialization) can prioritize clarity.
- Premature micro-tuning without measurement.

---

## 2. Vocabulary: hot vs cold paths

### Hot paths (must follow strict rules)
Code executed at a fixed cadence or inside tight loops, e.g.:
- Plant RHS / dynamics evaluation (integrator callback)
- Actuator update + motor/prop models
- Autopilot tick and uORB I/O
- Event scheduler boundary processing (if evaluated frequently)
- Recording/logging when enabled at tick rate

### Cold paths (performance is secondary)
- Aircraft/spec builders
- Scenario creation
- One-time symbol resolution / library loading
- Post-processing, plotting, file writing

**Rule of thumb:** If it runs every tick, treat it as hot until proven otherwise.

---

## 3. Performance contracts (hard requirements)

### Contract A — No allocations
Hot-path functions must allocate **0 bytes** in steady state.

Enforcement patterns:
- Provide an `@allocated` test for each hot call site (RHS, tick, uORB IO).
- Preallocate all buffers during initialization.

### Contract B — No runtime reflection
Hot-path code must not use:
- `applicable`, `hasmethod`, `fieldnames`, `propertynames`, `getproperty`-based introspection
- `eval` / runtime codegen
- `try/catch` for control flow

If capability detection is needed, do it **once at init** and store flags.

### Contract C — Monomorphic hot loops
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
- unbounded user-generated types

Mitigation:
- Keep runtime structs parametric on stable component types (motor model type, prop model type), not on ad-hoc closures.
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

### Avoid `StaticArrays` when
- Size is large or variable (code size and compile time grow quickly).
- You’re doing large linear algebra (BLAS-friendly `Matrix`/`Vector` is better).

Broadcasting over `SVector` generally does not allocate, but:
- Prefer plain scalar ops (`*`, `+`, `-`) instead of `.*`/`.+=` for scalar–vector operations to reduce broadcast machinery.
- Keep expressions simple to help LLVM.

---

## 8. C/FFI (PX4 lockstep) rules

### 8.1 Cache function pointers at initialization
Never call `dlsym`/symbol resolution in hot loops.

### 8.2 Avoid per-tick C heap allocations
Preallocate and reuse buffers on both Julia and C sides where possible.

### 8.3 Validate ABI sizes at init only
Size/layout checks should happen at init; they must not run in the hot path.

---

## 9. Practical checklist (quick review)

- Hot paths allocate 0 bytes (`@allocated` tests or equivalent).
- Hot loops are monomorphic (no `Any` fields, no abstract containers).
- No runtime reflection or `try/catch` for control flow in hot paths.
- No RNG inside RHS; randomness only at discrete boundaries.
- uORB buffers are preallocated and reused.

