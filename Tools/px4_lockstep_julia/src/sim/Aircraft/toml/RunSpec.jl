"""Load, parse, and run an aircraft spec TOML.

This is a convenience wrapper around:
  `load_spec(path) |> build_engine(spec; ...)`

The optional `[run]` table can provide defaults for mode/record paths.
Passing explicit `mode`/`recording_*` arguments overrides the `[run]` section.
"""
function run_spec(
    path::AbstractString;
    mode = nothing,
    recording_in = nothing,
    recording_out = nothing,
    strict::Bool = true,
    base_spec = nothing,
    telemetry = nothing,
    log_sinks = nothing,
    report_timing::Bool = false,
)
    cfg = _load_toml_with_extends(path; strict = strict)
    base_dir = dirname(abspath(path))
    base = _resolve_base_spec(path, base_spec; strict = strict)
    spec = spec_from_toml_dict(cfg; base_dir = base_dir, strict = strict, base_spec = base)

    # Read run defaults.
    run_tbl = get(cfg, "run", Dict{String,Any}())
    if !(run_tbl isa AbstractDict)
        error("run must be a table")
    end
    strict && _known_keys!(
        run_tbl,
        Set(["mode", "recording_in", "recording_out", "log_csv"]),
        "run",
    )

    mode_sym = if mode === nothing
        if haskey(run_tbl, "mode")
            Symbol(lowercase(_as_string(run_tbl["mode"], "run.mode")))
        else
            :live
        end
    else
        if mode isa Symbol
            mode
        elseif mode isa AbstractString
            Symbol(lowercase(String(mode)))
        else
            error("run_spec: mode must be Symbol or String (got $(typeof(mode)))")
        end
    end
    mode_sym in (:live, :record, :replay) || error("run_spec: unknown mode=$mode_sym")

    if mode_sym !== :replay && spec.px4.libpath === nothing
        error("px4.libpath is required for live/record runs; set it in the TOML.")
    end

    rec_in = recording_in
    if rec_in === nothing && haskey(run_tbl, "recording_in")
        rec_in = _resolve_path(
            base_dir,
            _as_string(run_tbl["recording_in"], "run.recording_in");
            must_exist = true,
        )
    end

    rec_out = recording_out
    if rec_out === nothing && haskey(run_tbl, "recording_out")
        rec_out = _resolve_path(
            base_dir,
            _as_string(run_tbl["recording_out"], "run.recording_out");
            must_exist = false,
        )
    end

    tel = telemetry === nothing ? spec.telemetry : telemetry
    logs = log_sinks === nothing ? spec.log_sinks : log_sinks
    if logs === nothing && haskey(run_tbl, "log_csv")
        log_path = _resolve_path(
            base_dir,
            _as_string(run_tbl["log_csv"], "run.log_csv");
            must_exist = false,
        )
        log_path === nothing || (logs = Logging.CSVLogSink(log_path))
    end

    return build_engine(
        spec;
        mode = mode_sym,
        recording_in = rec_in,
        recording_out = rec_out,
        telemetry = tel,
        log_sinks = logs,
        report_timing = report_timing,
    )
end
