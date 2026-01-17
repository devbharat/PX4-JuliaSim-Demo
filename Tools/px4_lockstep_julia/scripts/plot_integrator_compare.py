#!/usr/bin/env python3
"""Plot PX4 lockstep integrator comparison outputs.

Reads the summary CSV produced by `compare_integrators_iris_mission` and optionally
per-solver replay logs to visualize trajectories and error time series.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt


def load_log(path: Path) -> dict[str, list[float]]:
    with path.open("r", newline="") as handle:
        header = None
        rows: list[str] = []
        for line in handle:
            if line.startswith("#"):
                continue
            if header is None:
                header = line
            else:
                rows.append(line)
        if header is None:
            raise ValueError(f"log has no columns: {path}")
        reader = csv.DictReader([header, *rows])
        data: dict[str, list[float]] = {key: [] for key in reader.fieldnames or []}
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                if value is None or value == "":
                    data[key].append(float("nan"))
                else:
                    data[key].append(float(value))
    if not data:
        raise ValueError(f"log has no columns: {path}")
    return data


def load_summary(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            parsed: dict[str, float | str] = {}
            for key, value in row.items():
                if key == "solver":
                    parsed[key] = value or ""
                else:
                    parsed[key] = float(value) if value not in (None, "") else float("nan")
            rows.append(parsed)
    return rows


def slugify(label: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_")
    return slug or "solver"


def _safe_series(data: dict[str, list[float]], key: str) -> list[float] | None:
    values = data.get(key)
    if values is None or len(values) == 0:
        return None
    return values


def _align_by_time_us(
    ref: dict[str, list[float]],
    sol: dict[str, list[float]],
) -> tuple[list[float], dict[str, list[float]], dict[str, list[float]]]:
    """Align two logs by integer `time_us` (microseconds).

    Why: the float `time_s` axis is formatted/rounded in CSV. If a solver produces
    a different number of samples (or a slightly different time grid), aligning by
    index will create artificial "error drift".

    Returns:
      (t_s, ref_aligned, sol_aligned)

    Falls back to index alignment (min length) when `time_us` is missing.
    """

    ref_tus = _safe_series(ref, "time_us")
    sol_tus = _safe_series(sol, "time_us")
    ref_ts = _safe_series(ref, "time_s")

    if ref_tus is None or sol_tus is None or ref_ts is None:
        # Best-effort index alignment.
        n = min(len(ref_ts) if ref_ts is not None else 0, len(sol.get("time_s", [])))
        t = list(ref_ts[:n]) if ref_ts is not None else list(range(n))
        ref_a = {k: v[:n] for k, v in ref.items()}
        sol_a = {k: v[:n] for k, v in sol.items()}
        return t, ref_a, sol_a

    ref_idx = {int(t): i for i, t in enumerate(ref_tus)}
    sol_idx = {int(t): i for i, t in enumerate(sol_tus)}
    common = sorted(set(ref_idx.keys()).intersection(sol_idx.keys()))

    if not common:
        # No overlap; fall back to index alignment.
        n = min(len(ref_ts), len(sol.get("time_s", [])))
        t = list(ref_ts[:n])
        ref_a = {k: v[:n] for k, v in ref.items()}
        sol_a = {k: v[:n] for k, v in sol.items()}
        return t, ref_a, sol_a

    t = [ref_ts[ref_idx[tus]] for tus in common]

    def pick(data: dict[str, list[float]], idx_map: dict[int, int]) -> dict[str, list[float]]:
        out: dict[str, list[float]] = {}
        for k, series in data.items():
            # Assume all series share the same indexing as time_us.
            out[k] = [series[idx_map[tus]] for tus in common]
        return out

    return t, pick(ref, ref_idx), pick(sol, sol_idx)


def _vector_norm(xs: list[float], ys: list[float], zs: list[float]) -> list[float]:
    return [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(xs, ys, zs)]


def _quat_angle(q_ref: tuple[float, float, float, float], q: tuple[float, float, float, float]) -> float:
    dot = abs(sum(a * b for a, b in zip(q_ref, q)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def plot_summary(rows: list[dict[str, float | str]], output: Path, show: bool) -> None:
    if not rows:
        print("Summary CSV has no rows; skipping summary plot.")
        return

    labels = [str(row.get("solver", "")) for row in rows]
    metrics = [
        ("pos_max_m", "pos max (m)"),
        ("vel_max_mps", "vel max (m/s)"),
        ("att_max_rad", "att max (rad)"),
        ("bodyrate_max_rad_s", "ω max (rad/s)"),
        ("rotor_omega_max_rad_s", "rotor ω max"),
        ("battV_max_v", "|ΔV| max (V)"),
        ("battI_max_a", "|ΔI| max (A)"),
    ]

    fig, axes = plt.subplots(2, 4, figsize=(14, 6))
    axes = axes.flatten()

    for idx, (key, title) in enumerate(metrics):
        ax = axes[idx]
        values = [float(row.get(key, float("nan"))) for row in rows]
        values = [0.0 if math.isnan(v) else v for v in values]
        ax.bar(labels, values)
        ax.set_title(title)
        ax.tick_params(axis="x", labelrotation=30)
        ax.grid(True, axis="y", alpha=0.3)

    for ax in axes[len(metrics) :]:
        ax.axis("off")

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_trajectories(
    ref: dict[str, list[float]],
    solver_logs: dict[str, dict[str, list[float]]],
    output: Path,
    show: bool,
) -> None:
    if not solver_logs:
        print("No solver logs found; skipping trajectory plot.")
        return

    t_ref = _safe_series(ref, "time_s")
    x_ref = _safe_series(ref, "pos_x")
    y_ref = _safe_series(ref, "pos_y")
    z_ref = _safe_series(ref, "pos_z")
    vx_ref = _safe_series(ref, "vel_x")
    vy_ref = _safe_series(ref, "vel_y")
    vz_ref = _safe_series(ref, "vel_z")

    if not all([t_ref, x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref]):
        print("Reference log missing trajectory fields; skipping trajectory plot.")
        return

    alt_ref = [-z for z in z_ref]
    speed_ref = [math.hypot(vx, vy) for vx, vy in zip(vx_ref, vy_ref)]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax_path, ax_alt, ax_speed, ax_vz = axes.flatten()

    ax_path.plot(y_ref, x_ref, color="black", label="ref")
    for label, data in solver_logs.items():
        x = _safe_series(data, "pos_x")
        y = _safe_series(data, "pos_y")
        if x and y:
            ax_path.plot(y, x, label=label)
    ax_path.set_xlabel("east (m)")
    ax_path.set_ylabel("north (m)")
    ax_path.set_title("XY Path")
    ax_path.axis("equal")
    ax_path.grid(True, alpha=0.3)
    ax_path.legend()

    ax_alt.plot(t_ref, alt_ref, color="black", label="ref")
    for label, data in solver_logs.items():
        t = _safe_series(data, "time_s")
        z = _safe_series(data, "pos_z")
        if t and z:
            ax_alt.plot(t, [-zi for zi in z], label=label)
    ax_alt.set_xlabel("time (s)")
    ax_alt.set_ylabel("m")
    ax_alt.set_title("Altitude")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend()

    ax_speed.plot(t_ref, speed_ref, color="black", label="ref")
    for label, data in solver_logs.items():
        t = _safe_series(data, "time_s")
        vx = _safe_series(data, "vel_x")
        vy = _safe_series(data, "vel_y")
        if t and vx and vy:
            speed = [math.hypot(vx_i, vy_i) for vx_i, vy_i in zip(vx, vy)]
            ax_speed.plot(t, speed, label=label)
    ax_speed.set_xlabel("time (s)")
    ax_speed.set_ylabel("m/s")
    ax_speed.set_title("Horizontal Speed")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    ax_vz.plot(t_ref, vz_ref, color="black", label="ref")
    for label, data in solver_logs.items():
        t = _safe_series(data, "time_s")
        vz = _safe_series(data, "vel_z")
        if t and vz:
            ax_vz.plot(t, vz, label=label)
    ax_vz.set_xlabel("time (s)")
    ax_vz.set_ylabel("m/s")
    ax_vz.set_title("Vertical Velocity")
    ax_vz.grid(True, alpha=0.3)
    ax_vz.legend()

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def _error_series(
    ref: dict[str, list[float]],
    sol: dict[str, list[float]],
) -> dict[str, list[float]]:
    t_ref = _safe_series(ref, "time_s")
    t_sol = _safe_series(sol, "time_s")
    if not t_ref or not t_sol:
        return {}
    n = min(len(t_ref), len(t_sol))

    def slice_series(key: str, data: dict[str, list[float]]) -> list[float] | None:
        series = _safe_series(data, key)
        if not series:
            return None
        return series[:n]

    px_r = slice_series("pos_x", ref)
    py_r = slice_series("pos_y", ref)
    pz_r = slice_series("pos_z", ref)
    px_s = slice_series("pos_x", sol)
    py_s = slice_series("pos_y", sol)
    pz_s = slice_series("pos_z", sol)

    vx_r = slice_series("vel_x", ref)
    vy_r = slice_series("vel_y", ref)
    vz_r = slice_series("vel_z", ref)
    vx_s = slice_series("vel_x", sol)
    vy_s = slice_series("vel_y", sol)
    vz_s = slice_series("vel_z", sol)

    p_r = slice_series("p", ref)
    q_r = slice_series("q", ref)
    r_r = slice_series("r", ref)
    p_s = slice_series("p", sol)
    q_s = slice_series("q", sol)
    r_s = slice_series("r", sol)

    qw_r = slice_series("q_w", ref)
    qx_r = slice_series("q_x", ref)
    qy_r = slice_series("q_y", ref)
    qz_r = slice_series("q_z", ref)
    qw_s = slice_series("q_w", sol)
    qx_s = slice_series("q_x", sol)
    qy_s = slice_series("q_y", sol)
    qz_s = slice_series("q_z", sol)

    errors: dict[str, list[float]] = {}
    if px_r and py_r and pz_r and px_s and py_s and pz_s:
        errors["pos"] = [
            math.sqrt((xr - xs) ** 2 + (yr - ys) ** 2 + (zr - zs) ** 2)
            for xr, yr, zr, xs, ys, zs in zip(px_r, py_r, pz_r, px_s, py_s, pz_s)
        ]

    if vx_r and vy_r and vz_r and vx_s and vy_s and vz_s:
        errors["vel"] = _vector_norm(
            [vr - vs for vr, vs in zip(vx_r, vx_s)],
            [vr - vs for vr, vs in zip(vy_r, vy_s)],
            [vr - vs for vr, vs in zip(vz_r, vz_s)],
        )

    if p_r and q_r and r_r and p_s and q_s and r_s:
        errors["rates"] = _vector_norm(
            [vr - vs for vr, vs in zip(p_r, p_s)],
            [vr - vs for vr, vs in zip(q_r, q_s)],
            [vr - vs for vr, vs in zip(r_r, r_s)],
        )

    if qw_r and qx_r and qy_r and qz_r and qw_s and qx_s and qy_s and qz_s:
        errors["att"] = [
            _quat_angle((wr, xr, yr, zr), (ws, xs, ys, zs))
            for wr, xr, yr, zr, ws, xs, ys, zs in zip(
                qw_r, qx_r, qy_r, qz_r, qw_s, qx_s, qy_s, qz_s
            )
        ]

    return errors


def plot_errors(
    ref: dict[str, list[float]],
    solver_logs: dict[str, dict[str, list[float]]],
    output: Path,
    show: bool,
) -> None:
    if not solver_logs:
        print("No solver logs found; skipping error plot.")
        return

    t_ref = _safe_series(ref, "time_s")
    if not t_ref:
        print("Reference log missing time_s; skipping error plot.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    ax_pos, ax_vel, ax_att, ax_rates = axes.flatten()

    for label, data in solver_logs.items():
        # Align by integer time_us axis when available. This guards against
        # subtle drift/formatting differences when comparing logs produced
        # by different runs.
        t, ref_a, data_a = _align_by_time_us(ref, data)
        errors = _error_series(ref_a, data_a)
        if "pos" in errors:
            ax_pos.plot(t[: len(errors["pos"])], errors["pos"], label=label)
        if "vel" in errors:
            ax_vel.plot(t[: len(errors["vel"])], errors["vel"], label=label)
        if "att" in errors:
            ax_att.plot(t[: len(errors["att"])], errors["att"], label=label)
        if "rates" in errors:
            ax_rates.plot(t[: len(errors["rates"])], errors["rates"], label=label)

    ax_pos.set_title("Position error")
    ax_pos.set_ylabel("m")
    ax_vel.set_title("Velocity error")
    ax_vel.set_ylabel("m/s")
    ax_att.set_title("Attitude error")
    ax_att.set_ylabel("rad")
    ax_rates.set_title("Body rate error")
    ax_rates.set_ylabel("rad/s")

    for ax in axes.flatten():
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", required=True, help="Summary CSV from integrator compare")
    parser.add_argument(
        "--log-dir",
        default="",
        help="Directory with *_ref_log.csv and *_<solver>_log.csv (optional)",
    )
    parser.add_argument(
        "--prefix",
        default="",
        help="Log file prefix (defaults to summary filename prefix)",
    )
    parser.add_argument("--summary-output", default="", help="Summary plot output path")
    parser.add_argument("--traj-output", default="", help="Trajectory plot output path")
    parser.add_argument("--error-output", default="", help="Error plot output path")
    parser.add_argument("--show", action="store_true", help="Display plot window")
    args = parser.parse_args()

    summary_path = Path(args.summary)
    if not summary_path.exists():
        raise SystemExit(f"Summary CSV not found: {summary_path}")

    rows = load_summary(summary_path)

    stem = summary_path.stem
    prefix = args.prefix or (stem[:-8] if stem.endswith("_summary") else stem)

    summary_output = Path(args.summary_output) if args.summary_output else summary_path.with_name(
        f"{prefix}_summary_plot.png"
    )
    plot_summary(rows, summary_output, args.show)
    print(f"Saved summary plot to {summary_output}")

    if not args.log_dir:
        return 0

    log_dir = Path(args.log_dir)
    if not log_dir.exists():
        raise SystemExit(f"Log directory not found: {log_dir}")

    ref_log = log_dir / f"{prefix}_ref_log.csv"
    if not ref_log.exists():
        print(f"Reference log not found: {ref_log}")
        print("Tip: rerun compare with IRIS_LOG_DIR set to emit replay logs.")
        return 0

    ref_data = load_log(ref_log)

    solver_logs: dict[str, dict[str, list[float]]] = {}
    for row in rows:
        label = str(row.get("solver", ""))
        if not label:
            continue
        slug = slugify(label)
        log_path = log_dir / f"{prefix}_{slug}_log.csv"
        if not log_path.exists():
            print(f"Solver log not found for {label}: {log_path}")
            continue
        solver_logs[label] = load_log(log_path)

    traj_output = Path(args.traj_output) if args.traj_output else log_dir / f"{prefix}_trajectories.png"
    error_output = Path(args.error_output) if args.error_output else log_dir / f"{prefix}_errors.png"

    plot_trajectories(ref_data, solver_logs, traj_output, args.show)
    print(f"Saved trajectory plot to {traj_output}")
    plot_errors(ref_data, solver_logs, error_output, args.show)
    print(f"Saved error plot to {error_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
