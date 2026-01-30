# Paper + simulator TODOs

This paper is intended to stay **code-faithful** (no claims that diverge from the implementation). The PDF builds cleanly, but the items below are not fully chased down yet or are explicit TODO notes in the codebase.

## A) Paper improvements (content + traceability)

1. **Record/replay: full pipeline description** **(done)**
   - Add a concrete “live run → record → replay” walkthrough.
   - Spell out what is recorded (streams, schemas), how traces are aligned to time axes, and how replay sources are constructed.
   - Anchor to: `src/Workflows/RecordReplay.jl`, `src/sim/Recording/Recorder.jl`, `src/sim/Recording/Traces.jl`, and the API wrappers in `src/sim/API.jl`.

2. **uORB injection appendix** **(done)**
   - List each injected topic, its publish period, and the exact frame/conversion used.
   - Add a note about schedule alignment: if an injection period is not a multiple of the PX4 step, publishes can be skipped (the API warns/throws).
   - Anchor to: `src/sim/Autopilots/UORBInjection.jl` and the validation in `src/sim/API.jl`.

3. **Configuration/schema reference** **(done)**
   - Document the TOML schema (all fields) and how it maps to `AircraftSpec`.
   - Provide a concise table of defaults that come from code (not from docs).
   - Anchor to: `src/sim/Aircraft/TOMLIO.jl`, `src/sim/Aircraft/Spec.jl`, `src/sim/Aircraft/Build.jl`, and `src/Workflows/assets/aircraft/*.toml`.

4. **Model validation section** **(partially done)**
   - Added a validation/regression section summarizing the existing analytic and contract tests in the codebase.
   - Still TODO: add a closed-loop PX4 mission validation (requires the lockstep shared library + a recorded mission trace artifact) and extend coverage to contact energy dissipation.

5. **Results/benchmarks**
   - Add runtime performance metrics (per-step cost, typical event rates, integrator choice trade-offs).
   - Add determinism checks (bytewise trace identity under same seed/config) for a canonical scenario.
   - Add cross-machine replay drift envelopes (bitwise if possible; otherwise report numerical tolerances).

6. **Related work citations**
   - The related-work section is currently high-level and intentionally uncited.
   - Add citations for: canonical PX4 SITL simulators/lockstep interfaces; deterministic hybrid/discrete-event simulation; record/replay reproducibility systems.

## B) Codebase TODOs already present in source

1. **Recorder enforcement + scalable backend**
   - `InMemoryRecorder` currently records non-decreasing timestamps but does not enforce axis membership at record time; the file explicitly calls this out as a TODO.
   - Planned: add an HDF5-backed recorder/writer for long runs.
   - Anchor to: `src/sim/Recording/Recorder.jl`.

2. **Convenience replay entrypoint**
   - The API has a TODO to provide `replay_recording(path_or_recording; ...)` that loads a recording, constructs replay sources automatically, runs the engine, and returns a comparison summary.
   - Anchor to: `src/sim/API.jl`.

## C) Things worth double-checking (not necessarily bugs)

1. **Injection period alignment**
   - Ensure your chosen `dt_autopilot_s` is an integer divisor of every uORB injection period you enable; otherwise you can silently miss scheduled publishes.

2. **Avionics constant-power approximation in bus solve**
   - During the bus-voltage solve, avionics constant-power loads are approximated as constant current computed at the equivalent open-circuit voltage. This is deliberate for monotonicity/determinism, but it introduces a small mismatch versus true constant-power behavior at low voltage.

3. **Contact model produces forces only at COM**
   - No contact moments, no multi-point landing gear, and no terrain heightfields.

4. **No regeneration / charging**
   - Motor current is clamped to non-negative and battery SOC integrates discharge only.
