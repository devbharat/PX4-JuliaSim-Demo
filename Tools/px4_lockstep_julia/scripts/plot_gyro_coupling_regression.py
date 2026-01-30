#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot gyro coupling regression bars.")
    parser.add_argument(
        "--csv",
        default="Tools/px4_lockstep_julia/examples/verification/out/gyro_coupling.csv",
        help="Input CSV with component,expected,simulated columns.",
    )
    parser.add_argument(
        "--output",
        default="Tools/px4_lockstep_julia/docs/Report/Latex/figs/gyro_coupling_plot.png",
        help="Output PNG path.",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    components = []
    expected = []
    simulated = []
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            components.append(row["component"])
            expected.append(float(row["expected"]))
            simulated.append(float(row["simulated"]))

    x = list(range(len(components)))
    width = 0.38
    fig, ax = plt.subplots(figsize=(5.4, 3.2), dpi=150)
    ax.bar([i - width / 2 for i in x], expected, width, label="expected", color="tab:blue")
    ax.bar([i + width / 2 for i in x], simulated, width, label="simulated", color="tab:orange")
    ax.axhline(0.0, color="black", linewidth=0.7)
    ax.set_xticks(x, components)
    ax.set_ylabel("ω̇ (rad/s²)")
    ax.set_title("Gyro coupling regression (tilted rotor axis)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(frameon=False)
    fig.tight_layout()

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
