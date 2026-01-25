#!/usr/bin/env python3
"""Overlay multiple PX4 lockstep sim logs.

Reads one or more sim_log CSVs produced by PX4Lockstep.Sim and writes a PNG summary
plot with curves overlaid for trend inspection (e.g. altitude vs current).

Schema comment lines (e.g. "# schema_version=...") are ignored.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re

import matplotlib.pyplot as plt


def load_log(path: Path) -> dict[str, list[float]]:
    with path.open("r", newline="") as handle:
        header = None
        rows = []
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


def _avg_series(data: dict[str, list[float]], keys: list[str]) -> list[float] | None:
    series = [data[key] for key in keys if key in data]
    if not series:
        return None
    n = len(series[0])
    if any(len(values) != n for values in series):
        return None
    out: list[float] = []
    for idx in range(n):
        vals = [values[idx] for values in series]
        valid = [v for v in vals if v == v]
        out.append(sum(valid) / len(valid) if valid else float("nan"))
    return out


_ALT_RE = re.compile(r"alt[_-]?([+-]?\d+(?:\.\d+)?)m", re.IGNORECASE)


def _mean(values: list[float]) -> float | None:
    valid = [v for v in values if v == v]
    if not valid:
        return None
    return sum(valid) / len(valid)


def _parse_altitude(path: Path) -> float | None:
    match = _ALT_RE.search(path.stem)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def plot_overlay(logs: list[tuple[Path, dict[str, list[float]]]], output: Path, show: bool) -> None:
    fig, axes = plt.subplots(5, 2, figsize=(14, 17), sharex="col")
    ax_rho, ax_rho_mean = axes[0]
    ax_omega, ax_omega_mean = axes[1]
    ax_duty, ax_duty_mean = axes[2]
    ax_current, ax_current_mean = axes[3]
    ax_rem, ax_rem_mean = axes[4]

    means = []

    for path, data in logs:
        label = path.stem
        alt_m = _parse_altitude(path)
        if alt_m is not None:
            label = f"{alt_m:.0f} m"
        t = data.get("time_s")
        if not t:
            continue

        rho = data.get("rho")
        if rho and len(rho) == len(t):
            ax_rho.plot(t, rho, label=label)
        rho_mean = _mean(rho) if rho else None

        omega_avg = _avg_series(data, ["rotor_w1", "rotor_w2", "rotor_w3", "rotor_w4"])
        if omega_avg and len(omega_avg) == len(t):
            ax_omega.plot(t, omega_avg, label=label)
        omega_mean = _mean(omega_avg) if omega_avg else None

        duty_avg = _avg_series(data, ["m1", "m2", "m3", "m4"])
        if duty_avg and len(duty_avg) == len(t):
            ax_duty.plot(t, duty_avg, label=label)
        duty_mean = _mean(duty_avg) if duty_avg else None

        batt_a = data.get("batt_a")
        if batt_a and len(batt_a) == len(t):
            ax_current.plot(t, batt_a, label=label)
        batt_mean = _mean(batt_a) if batt_a else None

        batt_rem = data.get("batt_rem")
        if batt_rem and len(batt_rem) == len(t):
            ax_rem.plot(t, batt_rem, label=label)
        batt_rem_mean = _mean(batt_rem) if batt_rem else None

        means.append((alt_m, label, rho_mean, omega_mean, duty_mean, batt_mean, batt_rem_mean))

    ax_rho.set_ylabel("kg/m^3")
    ax_rho.set_title("Air Density (rho)")
    ax_rho.grid(True, alpha=0.3)
    ax_rho.legend()

    ax_omega.set_ylabel("rad/s")
    ax_omega.set_title("Mean Rotor Speed (rotor_w_avg)")
    ax_omega.grid(True, alpha=0.3)
    ax_omega.legend()

    ax_duty.set_ylabel("norm")
    ax_duty.set_title("Mean Motor Duty (m_avg)")
    ax_duty.grid(True, alpha=0.3)
    ax_duty.legend()

    ax_current.set_xlabel("time (s)")
    ax_current.set_ylabel("A")
    ax_current.set_title("Battery Current")
    ax_current.grid(True, alpha=0.3)
    ax_current.legend()

    ax_rem.set_xlabel("time (s)")
    ax_rem.set_ylabel("fraction")
    ax_rem.set_title("Battery Remaining")
    ax_rem.grid(True, alpha=0.3)
    ax_rem.legend()

    have_alt = all(m[0] is not None for m in means) and len(means) > 0
    if have_alt:
        means_sorted = sorted(means, key=lambda item: item[0])
        x = [item[0] for item in means_sorted]
        x_label = "altitude MSL (m)"
    else:
        means_sorted = list(means)
        x = list(range(1, len(means_sorted) + 1))
        x_label = "run index"

    rho_mean_vals = [item[2] for item in means_sorted]
    omega_mean_vals = [item[3] for item in means_sorted]
    duty_mean_vals = [item[4] for item in means_sorted]
    batt_mean_vals = [item[5] for item in means_sorted]
    batt_rem_mean_vals = [item[6] for item in means_sorted]

    ax_rho_mean.plot(x, rho_mean_vals, marker="o")
    ax_rho_mean.set_ylabel("kg/m^3")
    ax_rho_mean.set_title("Mean rho vs altitude")
    ax_rho_mean.grid(True, alpha=0.3)

    ax_omega_mean.plot(x, omega_mean_vals, marker="o")
    ax_omega_mean.set_ylabel("rad/s")
    ax_omega_mean.set_title("Mean rotor_w_avg vs altitude")
    ax_omega_mean.grid(True, alpha=0.3)

    ax_duty_mean.plot(x, duty_mean_vals, marker="o")
    ax_duty_mean.set_ylabel("norm")
    ax_duty_mean.set_title("Mean duty vs altitude")
    ax_duty_mean.grid(True, alpha=0.3)

    ax_current_mean.plot(x, batt_mean_vals, marker="o")
    ax_current_mean.set_ylabel("A")
    ax_current_mean.set_title("Mean batt_a vs altitude")
    ax_current_mean.grid(True, alpha=0.3)

    ax_rem_mean.plot(x, batt_rem_mean_vals, marker="o")
    ax_rem_mean.set_ylabel("fraction")
    ax_rem_mean.set_title("Mean batt_rem vs altitude")
    ax_rem_mean.grid(True, alpha=0.3)
    ax_rem_mean.set_xlabel(x_label)

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_trajectory_overlay(
    logs: list[tuple[Path, dict[str, list[float]]]], output: Path, show: bool
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax_alt, ax_xy = axes

    for path, data in logs:
        label = path.stem
        alt_m = _parse_altitude(path)
        if alt_m is not None:
            label = f"{alt_m:.0f} m"

        t = data.get("time_s")
        z = data.get("pos_z")
        x = data.get("pos_x")
        y = data.get("pos_y")
        if t and z and len(t) == len(z):
            alt = [-zi for zi in z]
            ax_alt.plot(t, alt, label=label)
        if x and y and len(x) == len(y):
            ax_xy.plot(y, x, label=label)

    ax_alt.set_xlabel("time (s)")
    ax_alt.set_ylabel("m")
    ax_alt.set_title("Altitude")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend()

    ax_xy.set_xlabel("east (m)")
    ax_xy.set_ylabel("north (m)")
    ax_xy.set_title("XY Path")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", action="append", default=[], help="Path to sim_log.csv")
    parser.add_argument(
        "--output", default="sim_plot_overlay.png", help="Output plot path"
    )
    parser.add_argument(
        "--traj-output",
        default="",
        help="Optional output path for trajectory/altitude overlay plot",
    )
    parser.add_argument("--show", action="store_true", help="Display plot window")
    args = parser.parse_args()

    if not args.log:
        raise SystemExit("Provide at least one --log path")

    logs: list[tuple[Path, dict[str, list[float]]]] = []
    for path_str in args.log:
        path = Path(path_str)
        if not path.exists():
            raise SystemExit(f"Log not found: {path}")
        data = load_log(path)
        logs.append((path, data))

    plot_overlay(logs, Path(args.output), args.show)
    print(f"Saved plot to {args.output}")
    if args.traj_output:
        plot_trajectory_overlay(logs, Path(args.traj_output), args.show)
        print(f"Saved plot to {args.traj_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
