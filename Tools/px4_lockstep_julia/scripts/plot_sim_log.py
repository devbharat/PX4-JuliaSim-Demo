#!/usr/bin/env python3
"""Plot and summarize PX4 lockstep sim logs.

Reads `sim_log.csv` produced by `PX4Lockstep.Sim` and writes a PNG summary plot.
Schema comment lines (e.g. `# schema_version=...`) are ignored.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

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
            raise ValueError("log has no columns")
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
    batt_temp = data.get("batt_temp_c", [])
    if batt_v:
        print(f"Battery voltage: min {min(batt_v):.2f} V, max {max(batt_v):.2f} V")
    if batt_a:
        print(f"Battery current: max {max(batt_a):.2f} A")
    if batt_rem:
        print(f"Battery remaining: min {min(batt_rem):.2f}")
    if batt_temp:
        finite = [v for v in batt_temp if not math.isnan(v)]
        if finite:
            print(f"Battery temp: min {min(finite):.2f} C, max {max(finite):.2f} C")


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
        valid = [v for v in vals if not math.isnan(v)]
        out.append(sum(valid) / len(valid) if valid else float("nan"))
    return out


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


def plot_battery(data: dict[str, list[float]], output: Path, show: bool) -> None:
    t = data.get("time_s", [])
    if not t:
        return

    batt_v = data.get("batt_v", [])
    batt_a = data.get("batt_a", [])
    batt_temp = data.get("batt_temp_c", [])
    batt_rem = data.get("batt_rem", [])
    batt_warn = data.get("batt_warn", [])

    batt_p = []
    if batt_v and batt_a and len(batt_v) == len(t) and len(batt_a) == len(t):
        batt_p = [v * a for v, a in zip(batt_v, batt_a)]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax_vi, ax_soc, ax_temp = axes

    handles: list[plt.Line2D] = []
    labels: list[str] = []
    if batt_v and len(batt_v) == len(t):
        (line_v,) = ax_vi.plot(t, batt_v, label="voltage_v")
        handles.append(line_v)
        labels.append("voltage_v")
    ax_vi2 = None
    if batt_a and len(batt_a) == len(t):
        ax_vi2 = ax_vi.twinx()
        (line_a,) = ax_vi2.plot(t, batt_a, color="tab:red", label="current_a")
        handles.append(line_a)
        labels.append("current_a")
        ax_vi2.set_ylabel("A")
    if batt_p and len(batt_p) == len(t):
        ax_vi3 = ax_vi.twinx()
        ax_vi3.spines["right"].set_position(("axes", 1.1))
        (line_p,) = ax_vi3.plot(t, batt_p, color="tab:purple", label="power_w")
        handles.append(line_p)
        labels.append("power_w")
        ax_vi3.set_ylabel("W")
    if handles:
        ax_vi.legend(handles, labels)
    ax_vi.set_ylabel("V")
    ax_vi.set_title("Battery Voltage / Current / Power")
    ax_vi.grid(True, alpha=0.3)

    if batt_rem and len(batt_rem) == len(t):
        ax_soc.plot(t, batt_rem, label="remaining")
    if batt_warn and len(batt_warn) == len(t):
        ax_soc.step(t, batt_warn, where="post", label="warning")
    ax_soc.set_ylabel("fraction / warning")
    ax_soc.set_title("Battery Remaining / Warning")
    ax_soc.legend()
    ax_soc.grid(True, alpha=0.3)

    if batt_temp and len(batt_temp) == len(t):
        ax_temp.plot(t, batt_temp, label="temp_c", color="tab:green")
    ax_temp.set_xlabel("time (s)")
    ax_temp.set_ylabel("C")
    ax_temp.set_title("Battery Temperature")
    ax_temp.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_contact(data: dict[str, list[float]], output: Path, show: bool) -> None:
    t = data.get("time_s", [])
    if not t:
        return

    landed = data.get("landed_phy", [])
    impact_count = data.get("impact_count", [])
    impact_dv_z = data.get("impact_dv_z", [])
    impact_acc_z = data.get("impact_acc_est_z", [])
    acc_z = data.get("acc_z", [])
    spec_bz = data.get("spec_bz", [])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax_state, ax_impulse, ax_accel = axes

    if landed and len(landed) == len(t):
        ax_state.step(t, landed, where="post", label="landed_phy")
    if impact_count and len(impact_count) == len(t):
        ax_state.step(t, impact_count, where="post", label="impact_count")
    ax_state.set_ylabel("flag / count")
    ax_state.set_title("Contact State")
    ax_state.legend()
    ax_state.grid(True, alpha=0.3)

    handles: list[plt.Line2D] = []
    labels: list[str] = []
    if impact_dv_z and len(impact_dv_z) == len(t):
        # Keep dv around for potential future use but avoid plotting to reduce clutter.
        pass
    if impact_acc_z and len(impact_acc_z) == len(t):
        (line_acc,) = ax_impulse.plot(t, impact_acc_z, color="tab:orange", label="impact_acc_est_z")
        handles.append(line_acc)
        labels.append("impact_acc_est_z")
        ax_impulse.set_ylabel("m/s^2")
    if handles:
        ax_impulse.legend(handles, labels)
    ax_impulse.set_title("Impact Accel (Z)")
    ax_impulse.grid(True, alpha=0.3)

    if acc_z and len(acc_z) == len(t):
        ax_accel.plot(t, acc_z, label="acc_z (NED)")
    if spec_bz and len(spec_bz) == len(t):
        ax_accel.plot(t, spec_bz, label="spec_bz (body)")
    ax_accel.set_xlabel("time (s)")
    ax_accel.set_ylabel("m/s^2")
    ax_accel.set_title("Acceleration / Specific Force")
    ax_accel.legend()
    ax_accel.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_inflow(data: dict[str, list[float]], output: Path, show: bool) -> None:
    t = data["time_s"]
    vz = data.get("vel_z", [])
    air_bz = data.get("air_bz", [])
    batt_v = data.get("batt_v", [])

    duty_avg = _avg_series(data, ["m1", "m2", "m3", "m4"])
    thrust_avg = _avg_series(data, ["rotor_T1", "rotor_T2", "rotor_T3", "rotor_T4"])
    omega_avg = _avg_series(data, ["rotor_w1", "rotor_w2", "rotor_w3", "rotor_w4"])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax_air, ax_thrust, ax_omega = axes

    if air_bz and len(air_bz) == len(t):
        vax = [-v for v in air_bz]
        ax_air.plot(t, vax, label="Vax (-air_bz)")
    if vz and len(vz) == len(t):
        ax_air.plot(t, vz, label="vz (NED)")
    ax_air.set_ylabel("m/s")
    ax_air.set_title("Axial Inflow vs Vertical Speed")
    ax_air.legend()
    ax_air.grid(True, alpha=0.3)

    if thrust_avg and len(thrust_avg) == len(t):
        ax_thrust.plot(t, thrust_avg, label="rotor_T_avg")
    ax_thrust2 = None
    if duty_avg and len(duty_avg) == len(t):
        ax_thrust2 = ax_thrust.twinx()
        ax_thrust2.plot(t, duty_avg, color="tab:red", label="duty_avg")
        ax_thrust2.set_ylabel("duty")
    ax_thrust.set_ylabel("N")
    ax_thrust.set_title("Mean Rotor Thrust vs Duty")
    handles = ax_thrust.get_lines()
    labels = [line.get_label() for line in handles]
    if ax_thrust2 is not None:
        handles += ax_thrust2.get_lines()
        labels += [line.get_label() for line in ax_thrust2.get_lines()]
    if handles:
        ax_thrust.legend(handles, labels)
    ax_thrust.grid(True, alpha=0.3)

    if omega_avg and len(omega_avg) == len(t):
        ax_omega.plot(t, omega_avg, label="rotor_w_avg")
    ax_omega2 = None
    if batt_v and len(batt_v) == len(t):
        ax_omega2 = ax_omega.twinx()
        ax_omega2.plot(t, batt_v, color="tab:red", label="batt_v")
        ax_omega2.set_ylabel("V")
    ax_omega.set_ylabel("rad/s")
    ax_omega.set_title("Rotor Speed vs Battery Voltage")
    handles = ax_omega.get_lines()
    labels = [line.get_label() for line in handles]
    if ax_omega2 is not None:
        handles += ax_omega2.get_lines()
        labels += [line.get_label() for line in ax_omega2.get_lines()]
    if handles:
        ax_omega.legend(handles, labels)
    ax_omega.grid(True, alpha=0.3)
    ax_omega.set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", default="sim_log.csv", help="Path to sim_log.csv")
    parser.add_argument("--output", default="sim_plot.png", help="Output plot path")
    parser.add_argument(
        "--inflow-output",
        default="",
        help="Optional output path for inflow/propulsion debug plot",
    )
    parser.add_argument(
        "--battery-output",
        default="",
        help="Optional output path for a battery/thermal summary plot",
    )
    parser.add_argument(
        "--contact-output",
        default="",
        help="Optional output path for a contact/landing summary plot",
    )
    parser.add_argument("--show", action="store_true", help="Display plot window")
    args = parser.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        raise SystemExit(f"Log not found: {log_path}")

    data = load_log(log_path)
    summarize(data)
    output_path = Path(args.output)
    plot(data, output_path, args.show)
    if args.inflow_output:
        plot_inflow(data, Path(args.inflow_output), args.show)
    battery_output = args.battery_output
    if args.show and not battery_output:
        battery_output = output_path.with_name(f"{output_path.stem}_battery{output_path.suffix}")
    if battery_output:
        plot_battery(data, Path(battery_output), args.show)
    contact_output = args.contact_output
    if args.show and not contact_output:
        contact_output = output_path.with_name(f"{output_path.stem}_contact{output_path.suffix}")
    if contact_output:
        plot_contact(data, Path(contact_output), args.show)
    print(f"Saved plot to {args.output}")
    if battery_output:
        print(f"Saved battery plot to {battery_output}")
    if contact_output:
        print(f"Saved contact plot to {contact_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
