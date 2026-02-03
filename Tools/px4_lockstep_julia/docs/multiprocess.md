# Multiprocess Lockstep Runs

This document captures what is safe (and unsafe) when running multiple Julia
processes against `libpx4_lockstep` in parallel (e.g., one process per
integration tier).

## Summary

**Multiple processes are allowed**, but **running two lockstep processes at the
same time can be flaky** unless you isolate shared resources. The C harness
only forbids multiple `px4_lockstep` handles inside a *single* process. Each
process gets its own uORB manager, parameter store, and dataman instance.

The main risks come from **shared filesystem state** and **optional distributed
uORB** builds. The lockstep defaults avoid these, but it is easy to opt into
shared resources when enabling extra modules. If you see flaky behavior with
concurrent lockstep runs, serialize them.

## What is isolated per process

- **Lockstep handle count**: C++ enforces one handle per process (`g_lockstep_active`).
- **uORB**: in-process `uORB::Manager` singleton (no shared OS state by default).
- **Parameters**: in-process param system initialized at `px4_platform_init()`.
- **Dataman**: RAM backend by default (no file I/O).

## Shared resources and collision risks

### 1) Dataman file backend (high risk if enabled)

If `dataman_use_ram` is disabled, dataman writes to
`PX4_STORAGEDIR/dataman`. Multiple processes will then read/write the same
file, which can corrupt mission/state data.

Default is **RAM** in the Julia wrapper, so this is safe unless you override it.

### 2) PX4 storage directory (moderate risk if extra modules are enabled)

Many PX4 modules write to `PX4_STORAGEDIR` (logs, time persistence, ftp temp
files, etc.). The lockstep harness does **not** start those modules, but if you
enable additional PX4 modules in lockstep, they can collide on these files.

Examples of files under `PX4_STORAGEDIR`:
- `dataman` (dataman file backend)
- `time_save.bin` (time_persistor)
- `log/` and `logdata.txt` (logger/mavlink_log_handler)
- `etc/geofence.txt` (navigator geofence)

### 3) MUORB (distributed uORB) builds (moderate risk if enabled)

If the build enables `CONFIG_MODULES_MUORB_APPS`, uORB becomes distributed and
may communicate across processes. The lockstep board config does not enable
MUORB, so default lockstep builds are safe.

### 4) Stdout/stderr (low risk; just noisy)

PX4 logging prints to stdout/stderr. Multiple processes will interleave logs.
Redirect each process to its own log file for readability.

## Do / Don’t

**Do**
- Keep `dataman_use_ram=1` (default in `LockstepConfig`).
- Keep the lockstep harness minimal (no logger/mavlink/time_persistor/etc.).
- Redirect logs per process if running many tiers in parallel.
- Use per-process temp dirs for recordings (the tests already use `mktempdir()`).

**Don’t**
- Don’t set `dataman_use_ram=0` when running in parallel unless you also ensure
  a *unique* `PX4_STORAGEDIR` per process (requires separate builds).
- Don’t enable MUORB or other distributed uORB modes for parallel lockstep runs.
- Don’t run multiple lockstep handles in the **same** Julia process.
- Don’t overlap **multiple lockstep processes** unless you have isolated shared
  filesystem resources; serialize if tests become flaky.

## Recommended parallel pattern (integration tiers)

If you want to shard integration tiers across multiple Julia processes:

1. **One process per tier** (or per group of tiers).
2. Keep `dataman_use_ram=1`.
3. Keep lockstep build minimal.
4. Redirect logs per process.

This gives true parallelism without cross-talk, and avoids the per-process
handle restriction in `libpx4_lockstep`.
