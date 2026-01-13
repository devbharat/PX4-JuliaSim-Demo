#!/usr/bin/env python3
"""Plot and summarize PX4 lockstep sim logs.

Reads `sim_log.csv` produced by `PX4Lockstep.Sim` and writes a PNG summary plot.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def load_log(path: Path) -> dict[str, list[float]]:
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
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
        raise ValueError("log has no columns")
    return data


def summarize(data: dict[str, list[float]]) -> None:
    t = data.get("time_s", [])
    if not t:
        print("No samples found in log.")
        return
    pos_z = data.get("pos_z", [])
    vel_x = data.get("vel_x", [])
    vel_y = data.get("vel_y", [])
    alt = [-z for z in pos_z]
    speeds = [math.hypot(vx, vy) for vx, vy in zip(vel_x, vel_y)]
    print(f"Samples: {len(t)}")
    print(f"Duration: {t[-1] - t[0]:.2f} s")
    print(f"Altitude: min {min(alt):.2f} m, max {max(alt):.2f} m")
    if speeds:
        print(f"Horizontal speed: max {max(speeds):.2f} m/s")
    mission_seq = data.get("mission_seq", [])
    mission_count = data.get("mission_count", [])
    if mission_seq and mission_count:
        print(f"Mission progress: {max(mission_seq):.0f}/{max(mission_count):.0f}")
    batt_v = data.get("batt_v", [])
    batt_a = data.get("batt_a", [])
    batt_rem = data.get("batt_rem", [])
    if batt_v:
        print(f"Battery voltage: min {min(batt_v):.2f} V, max {max(batt_v):.2f} V")
    if batt_a:
        print(f"Battery current: max {max(batt_a):.2f} A")
    if batt_rem:
        print(f"Battery remaining: min {min(batt_rem):.2f}")


def plot(data: dict[str, list[float]], output: Path, show: bool) -> None:
    t = data["time_s"]
    x = data["pos_x"]
    y = data["pos_y"]
    z = data["pos_z"]
    vx = data["vel_x"]
    vy = data["vel_y"]
    vz = data["vel_z"]

    x_sp = data.get("pos_sp_x")
    y_sp = data.get("pos_sp_y")
    z_sp = data.get("pos_sp_z")
    vz_sp = data.get("vel_sp_z")
    vx_sp = data.get("vel_sp_x")
    vy_sp = data.get("vel_sp_y")

    batt_v = data.get("batt_v")
    batt_a = data.get("batt_a")
    batt_rem = data.get("batt_rem")
    batt_warn = data.get("batt_warn")

    alt = [-zi for zi in z]
    speed_xy = [math.hypot(vx_i, vy_i) for vx_i, vy_i in zip(vx, vy)]

    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    ax_alt, ax_xy, ax_vz, ax_path, ax_batt, ax_soc = axes.flatten()

    ax_alt.plot(t, alt, label="alt")
    if z_sp and len(z_sp) == len(t):
        ax_alt.plot(t, [-zi for zi in z_sp], label="alt_sp")
    ax_alt.set_xlabel("time (s)")
    ax_alt.set_ylabel("m")
    ax_alt.set_title("Altitude")
    ax_alt.grid(True, alpha=0.3)

    ax_xy.plot(t, x, label="x")
    ax_xy.plot(t, y, label="y")
    if x_sp and y_sp and len(x_sp) == len(t):
        ax_xy.plot(t, x_sp, label="x_sp")
        ax_xy.plot(t, y_sp, label="y_sp")
    ax_xy.set_xlabel("time (s)")
    ax_xy.set_ylabel("m")
    ax_xy.set_title("Position (N/E)")
    ax_xy.legend()
    ax_xy.grid(True, alpha=0.3)

    ax_vz.plot(t, vz, label="vz")
    if vz_sp and len(vz_sp) == len(t):
        ax_vz.plot(t, vz_sp, label="vz_sp")
    ax_vz.plot(t, speed_xy, label="speed_xy")
    if vx_sp and vy_sp and len(vx_sp) == len(t):
        speed_xy_sp = [math.hypot(vx_i, vy_i) for vx_i, vy_i in zip(vx_sp, vy_sp)]
        ax_vz.plot(t, speed_xy_sp, label="speed_xy_sp")
    ax_vz.set_xlabel("time (s)")
    ax_vz.set_ylabel("m/s")
    ax_vz.set_title("Velocity")
    ax_vz.legend()
    ax_vz.grid(True, alpha=0.3)

    ax_path.plot(y, x)
    ax_path.set_xlabel("east (m)")
    ax_path.set_ylabel("north (m)")
    ax_path.set_title("XY Path")
    ax_path.axis("equal")
    ax_path.grid(True, alpha=0.3)

    batt_handles: list[plt.Line2D] = []
    batt_labels: list[str] = []
    if batt_v and len(batt_v) == len(t):
        (line_v,) = ax_batt.plot(t, batt_v, label="voltage_v")
        batt_handles.append(line_v)
        batt_labels.append("voltage_v")
    ax_batt2 = None
    if batt_a and len(batt_a) == len(t):
        ax_batt2 = ax_batt.twinx()
        (line_a,) = ax_batt2.plot(t, batt_a, color="tab:red", label="current_a")
        batt_handles.append(line_a)
        batt_labels.append("current_a")
        ax_batt2.set_ylabel("A")
    if batt_handles:
        ax_batt.legend(batt_handles, batt_labels)
    ax_batt.set_xlabel("time (s)")
    ax_batt.set_ylabel("V")
    ax_batt.set_title("Battery Voltage/Current")
    ax_batt.grid(True, alpha=0.3)

    if batt_rem and len(batt_rem) == len(t):
        ax_soc.plot(t, batt_rem, label="remaining")
    if batt_warn and len(batt_warn) == len(t):
        ax_soc.step(t, batt_warn, where="post", label="warning")
    ax_soc.set_xlabel("time (s)")
    ax_soc.set_ylabel("fraction / warning")
    ax_soc.set_title("Battery Remaining/Warning")
    ax_soc.legend()
    ax_soc.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", default="sim_log.csv", help="Path to sim_log.csv")
    parser.add_argument("--output", default="sim_plot.png", help="Output plot path")
    parser.add_argument("--show", action="store_true", help="Display plot window")
    args = parser.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        raise SystemExit(f"Log not found: {log_path}")

    data = load_log(log_path)
    summarize(data)
    plot(data, Path(args.output), args.show)
    print(f"Saved plot to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
