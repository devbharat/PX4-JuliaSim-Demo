#!/usr/bin/env python3
"""Ingest Example 3290 mAh (8S1P) battery test artifacts into the simulator asset format.

This script is intentionally **standard-library only** (no numpy/pandas) to keep
dependencies minimal.

Inputs (defaults are in-repo):
- Raw OCV curve: example_cell_ocv_soc.csv
- DCIR dashboard HTML: IMPROVED_DCIR_Dashboard.html

Outputs:
- cells/<cell_id>/ocv_soc.csv
- cells/<cell_id>/resistance_surface_cell.csv
- packs/<pack_id>/dcir_extracted_pulses_pack.csv
- packs/<pack_id>/resistance_surface_pack.csv
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# ----------------------------
# Generic helpers
# ----------------------------

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def interp1(x: Sequence[float], y: Sequence[float], xq: float) -> float:
    """1D linear interpolation with clamping.

    Assumes `x` is strictly increasing (or at least non-decreasing).
    """
    if not x:
        raise ValueError("interp1: empty x")
    if len(x) != len(y):
        raise ValueError("interp1: x and y length mismatch")
    if xq <= x[0]:
        return float(y[0])
    if xq >= x[-1]:
        return float(y[-1])

    i = bisect.bisect_left(x, xq)
    if i <= 0:
        return float(y[0])
    if i >= len(x):
        return float(y[-1])
    x0 = float(x[i - 1])
    x1 = float(x[i])
    y0 = float(y[i - 1])
    y1 = float(y[i])
    if x1 == x0:
        return y1
    t = (xq - x0) / (x1 - x0)
    return (1.0 - t) * y0 + t * y1


def median(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("median: empty list")
    return float(statistics.median(values))


# ----------------------------
# OCV curve processing
# ----------------------------

@dataclass(frozen=True)
class OCVCurve:
    """OCV curve stored as SOC (remaining, 0..1) -> OCV (V, cell)."""

    soc: List[float]
    ocv_v: List[float]

    def ocv(self, soc: float) -> float:
        return interp1(self.soc, self.ocv_v, clamp01(soc))

    def soc_from_ocv(self, ocv_v: float) -> float:
        # Invert the monotone curve by interpolating SOC over OCV.
        # Sort by OCV so we can interpolate SOC(OCV).
        pairs = sorted(zip(self.ocv_v, self.soc), key=lambda p: p[0])
        ocv_sorted = [p[0] for p in pairs]
        soc_sorted = [p[1] for p in pairs]
        return clamp01(interp1(ocv_sorted, soc_sorted, ocv_v))


def load_example_ocv_raw(path: Path) -> OCVCurve:
    """Load Example-provided raw OCV CSV.

    Expected columns:
      - used_soc (0..100)   # 0 = full, 100 = empty
      - cell_ocv (V)
    """
    with path.open("r", newline="") as f:
        rdr = csv.DictReader(f)
        cols = set(rdr.fieldnames or [])
        if not {"used_soc", "cell_ocv"} <= cols:
            raise ValueError(
                f"Unexpected OCV CSV columns {sorted(cols)}; expected at least used_soc, cell_ocv"
            )
        used_soc: List[float] = []
        ocv_v: List[float] = []
        for row in rdr:
            if row.get("used_soc") in (None, ""):
                continue
            if row.get("cell_ocv") in (None, ""):
                continue
            used_soc.append(float(row["used_soc"]))
            ocv_v.append(float(row["cell_ocv"]))

    # Convert "used SOC" (DoD) -> "SOC remaining"
    soc = [clamp01(1.0 - u / 100.0) for u in used_soc]

    # Sort by SOC and drop duplicates (keep last).
    pairs = sorted(zip(soc, ocv_v), key=lambda p: p[0])
    soc_u: List[float] = []
    ocv_u: List[float] = []
    for s, v in pairs:
        if soc_u and s == soc_u[-1]:
            ocv_u[-1] = float(v)
        else:
            soc_u.append(float(s))
            ocv_u.append(float(v))

    if len(soc_u) < 2:
        raise ValueError("OCV curve collapsed to <2 unique SOC points")

    return OCVCurve(soc=soc_u, ocv_v=ocv_u)


def resample_ocv(curve: OCVCurve, step: float) -> OCVCurve:
    """Resample onto a uniform SOC grid for asset storage."""
    if step <= 0.0 or step > 1.0:
        raise ValueError(f"step must be in (0,1], got {step}")

    grid: List[float] = []
    s = 0.0
    # Ensure inclusion of 1.0
    while s < 1.0 - 1e-12:
        grid.append(round(s, 10))
        s += step
    grid.append(1.0)

    ocv_grid = [curve.ocv(si) for si in grid]
    return OCVCurve(soc=grid, ocv_v=ocv_grid)


def write_ocv_csv(path: Path, curve: OCVCurve) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["soc", "ocv_v"])
        for s, v in zip(curve.soc, curve.ocv_v):
            w.writerow([f"{s:.6f}", f"{v:.6f}"])


# ----------------------------
# DCIR dashboard extraction
# ----------------------------

PLOT_ID_RE = re.compile(r"Plotly\.newPlot\('([^']+)'")
PLOT_META_RE = re.compile(r"battery-(.+?)-temp-([0-9]+)-soc-([0-9]+)-plot")


@dataclass(frozen=True)
class PulseSample:
    battery: str
    temp_c: float
    soc_label_v: float  # label from dashboard (nominal pack voltage setpoint)
    current_a: float
    pulse_index: int

    v_pre_v: float
    v_min_v: float
    v_0p05_v: float
    v_0p10_v: float

    r0_ohm_pack: float
    rdc_ohm_pack: float
    r1_ohm_pack: float

    soc_est: float  # SOC estimated by inverting the OCV curve on V_pre/series


def extract_plot_ids(html_text: str) -> List[str]:
    return PLOT_ID_RE.findall(html_text)


def extract_plotdata(html_text: str, plot_id: str) -> List[dict]:
    idx = html_text.find(f"Plotly.newPlot('{plot_id}'")
    if idx < 0:
        raise ValueError(f"Plot id not found: {plot_id}")

    start = html_text.rfind("var plotData = ", 0, idx)
    if start < 0:
        raise ValueError(f"plotData not found for plot id: {plot_id}")
    start += len("var plotData = ")

    end = html_text.find("var plotLayout", start)
    if end < 0:
        raise ValueError(f"plotLayout not found for plot id: {plot_id}")

    blob = html_text[start:end].strip().rstrip(";")
    # Dashboard encodes JSON; handle any JS NaN just in case.
    blob = re.sub(r"\bNaN\b", "null", blob)
    return json.loads(blob)


def group_traces_by_current(plotdata: List[dict]) -> Dict[int, Dict[str, dict]]:
    out: Dict[int, Dict[str, dict]] = {}
    for tr in plotdata:
        name = str(tr.get("name", ""))
        m = re.match(r"(\d+)A$", name)
        if m:
            I = int(m.group(1))
            out.setdefault(I, {})["trace"] = tr
            continue
        m = re.match(r"(\d+)A Pre-Pulse$", name)
        if m:
            I = int(m.group(1))
            out.setdefault(I, {})["pre"] = tr
            continue
        m = re.match(r"(\d+)A Min Voltage$", name)
        if m:
            I = int(m.group(1))
            out.setdefault(I, {})["min"] = tr
            continue
    return out


def extract_pulses(
    *,
    html_text: str,
    ocv_curve: OCVCurve,
    series: int,
) -> List[PulseSample]:
    plot_ids = extract_plot_ids(html_text)
    if not plot_ids:
        raise ValueError("No Plotly plots found in dashboard HTML")

    samples: List[PulseSample] = []

    for pid in plot_ids:
        mm = PLOT_META_RE.match(pid)
        if not mm:
            # ignore non-battery plots (if any)
            continue

        battery = mm.group(1)
        temp_c = float(mm.group(2))
        soc_label_v = float(mm.group(3))

        plotdata = extract_plotdata(html_text, pid)
        traces = group_traces_by_current(plotdata)

        for I, parts in traces.items():
            if not ("trace" in parts and "pre" in parts and "min" in parts):
                continue

            x = [float(v) for v in parts["trace"].get("x", [])]
            y = [float(v) for v in parts["trace"].get("y", [])]
            if len(x) < 2:
                continue

            pre_x = [float(v) for v in parts["pre"].get("x", [])]
            pre_y = [float(v) for v in parts["pre"].get("y", [])]
            min_x = [float(v) for v in parts["min"].get("x", [])]
            min_y = [float(v) for v in parts["min"].get("y", [])]

            n = min(len(pre_x), len(pre_y), len(min_x), len(min_y))
            for k in range(n):
                t0 = pre_x[k]
                v_pre = pre_y[k]
                v_min = min_y[k]

                v_05 = interp1(x, y, t0 + 0.05)
                v_10 = interp1(x, y, t0 + 0.10)

                # R0 from the average of 0.05s and 0.1s after the step
                r0 = ((v_pre - v_05) / I + (v_pre - v_10) / I) / 2.0
                rdc = (v_pre - v_min) / I
                r1 = max(rdc - r0, 0.0)

                soc_est = ocv_curve.soc_from_ocv(v_pre / float(series))

                samples.append(
                    PulseSample(
                        battery=str(battery),
                        temp_c=temp_c,
                        soc_label_v=soc_label_v,
                        current_a=float(I),
                        pulse_index=k + 1,
                        v_pre_v=v_pre,
                        v_min_v=v_min,
                        v_0p05_v=v_05,
                        v_0p10_v=v_10,
                        r0_ohm_pack=r0,
                        rdc_ohm_pack=rdc,
                        r1_ohm_pack=r1,
                        soc_est=soc_est,
                    )
                )

    if not samples:
        raise ValueError("No pulse samples extracted (unexpected dashboard structure?)")

    return samples


def write_pulses_csv(path: Path, samples: Sequence[PulseSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "battery",
                "temp_c",
                "soc_label_v",
                "current_a",
                "pulse_index",
                "v_pre_v",
                "v_min_v",
                "v_0p05_v",
                "v_0p10_v",
                "r0_ohm_pack",
                "rdc_ohm_pack",
                "r1_ohm_pack",
                "soc_est",
            ]
        )
        for s in samples:
            w.writerow(
                [
                    s.battery,
                    f"{s.temp_c:.1f}",
                    f"{s.soc_label_v:.0f}",
                    f"{s.current_a:.0f}",
                    s.pulse_index,
                    f"{s.v_pre_v:.5f}",
                    f"{s.v_min_v:.5f}",
                    f"{s.v_0p05_v:.5f}",
                    f"{s.v_0p10_v:.5f}",
                    f"{s.r0_ohm_pack:.6f}",
                    f"{s.rdc_ohm_pack:.6f}",
                    f"{s.r1_ohm_pack:.6f}",
                    f"{s.soc_est:.6f}",
                ]
            )


def aggregate_surface(
    samples: Sequence[PulseSample],
    *,
    soc_bin_step: float,
    out_series: int,
    out_parallel: int,
    measured_series: int,
    measured_parallel: int,
    filter_r_ohm_max: float = 1.0,
    filter_vmin_min: float = 1.0,
) -> List[dict]:
    """Aggregate pulse samples into a (temp, soc_bin) resistance surface.

    - SOC bins are simple fixed-width bins with centers at (bin + step/2).
    - Uses **median** aggregation (robust to outliers).
    - Supports producing either pack-level or cell-level values via scaling.

    Scaling logic:
      r_pack = r_cell * series / parallel
      -> r_cell = r_pack * parallel / series

    The raw extracted values are pack-level for the *measured* pack configuration.
    """
    if soc_bin_step <= 0.0 or soc_bin_step > 1.0:
        raise ValueError("soc_bin_step must be in (0,1]")

    n_bins = int(round(1.0 / soc_bin_step))
    if abs(n_bins * soc_bin_step - 1.0) > 1e-9:
        raise ValueError("soc_bin_step must evenly divide 1.0 (e.g. 0.05, 0.02, 0.01)")

    # Group -> list of values
    groups: Dict[Tuple[float, float], Dict[str, List[float]]] = {}

    for s in samples:
        if s.v_min_v < filter_vmin_min:
            continue
        if not (0.0 < s.rdc_ohm_pack < filter_r_ohm_max):
            continue
        if not (0.0 < s.r0_ohm_pack < filter_r_ohm_max):
            continue

        soc = clamp01(s.soc_est)
        idx = int(min(n_bins - 1, max(0, soc / soc_bin_step)))
        soc_center = (idx + 0.5) * soc_bin_step
        key = (float(s.temp_c), float(soc_center))

        g = groups.setdefault(key, {"r0": [], "rdc": [], "r1": []})
        g["r0"].append(s.r0_ohm_pack)
        g["rdc"].append(s.rdc_ohm_pack)
        g["r1"].append(s.r1_ohm_pack)

    # Scaling: first convert from measured-pack resistance -> cell resistance,
    # then scale to the desired output pack config.
    # NOTE: this assumes resistance is dominated by cells (no separate wiring/BMS term).
    def scale_pack_to_out(r_pack_measured: float) -> float:
        r_cell = r_pack_measured * float(measured_parallel) / float(measured_series)
        r_out = r_cell * float(out_series) / float(out_parallel)
        return r_out

    rows: List[dict] = []
    for (temp_c, soc_center), vals in sorted(groups.items()):
        rows.append(
            dict(
                temp_c=temp_c,
                soc=soc_center,
                r0_ohm=scale_pack_to_out(median(vals["r0"])),
                rdc_ohm=scale_pack_to_out(median(vals["rdc"])),
                r1_ohm=scale_pack_to_out(median(vals["r1"])),
                n=len(vals["rdc"]),
            )
        )
    return rows


def write_surface_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["temp_c", "soc", "r0_ohm", "rdc_ohm", "r1_ohm", "n"])
        for r in rows:
            w.writerow(
                [
                    f"{r['temp_c']:.1f}",
                    f"{r['soc']:.6f}",
                    f"{r['r0_ohm']:.6f}",
                    f"{r['rdc_ohm']:.6f}",
                    f"{r['r1_ohm']:.6f}",
                    int(r["n"]),
                ]
            )


# ----------------------------
# Main
# ----------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--raw-ocv-csv",
        type=Path,
        default=Path("src/Workflows/assets/battery/raw/example_3290mah_8s1p/example_cell_ocv_soc.csv"),
        help="Path to raw Example OCV CSV (used_soc + cell_ocv).",
    )
    ap.add_argument(
        "--dcir-dashboard-html",
        type=Path,
        default=Path("src/Workflows/assets/battery/raw/example_3290mah_8s1p/IMPROVED_DCIR_Dashboard.html"),
        help="Path to the DCIR dashboard HTML file (Plotly).",
    )
    ap.add_argument(
        "--cell-id",
        type=str,
        default="example_3290mah_hv",
        help="Cell asset ID folder under assets/battery/cells/.",
    )
    ap.add_argument(
        "--pack-id",
        type=str,
        default="example_8s1p_3290mah",
        help="Pack asset ID folder under assets/battery/packs/.",
    )
    ap.add_argument(
        "--series",
        type=int,
        default=8,
        help="Series cell count for both the measured pack and the output pack asset.",
    )
    ap.add_argument(
        "--parallel",
        type=int,
        default=1,
        help="Parallel cell count for both the measured pack and the output pack asset.",
    )
    ap.add_argument(
        "--ocv-step",
        type=float,
        default=0.005,
        help="SOC grid step (fraction) used for the stored OCV curve (default: 0.005 = 0.5%% SOC).",
    )
    ap.add_argument(
        "--soc-bin-step",
        type=float,
        default=0.05,
        help="SOC bin size used for the resistance surface (default: 0.05 = 5%% SOC bins).",
    )

    args = ap.parse_args()

    if args.series <= 0 or args.parallel <= 0:
        raise ValueError("--series and --parallel must be positive")

    repo_root = Path(__file__).resolve().parents[2]
    assets_root = repo_root / "src/Workflows/assets/battery"
    cell_dir = assets_root / "cells" / args.cell_id
    pack_dir = assets_root / "packs" / args.pack_id

    # OCV
    ocv_raw = load_example_ocv_raw(repo_root / args.raw_ocv_csv)
    ocv_curve = resample_ocv(ocv_raw, step=float(args.ocv_step))
    write_ocv_csv(cell_dir / "ocv_soc.csv", ocv_curve)

    # DCIR / R0 extraction
    html_text = (repo_root / args.dcir_dashboard_html).read_text(encoding="utf-8", errors="ignore")
    pulses = extract_pulses(html_text=html_text, ocv_curve=ocv_curve, series=int(args.series))

    write_pulses_csv(pack_dir / "dcir_extracted_pulses_pack.csv", pulses)

    # Resistance surfaces:
    # - pack-level surface (for this pack config)
    surface_pack = aggregate_surface(
        pulses,
        soc_bin_step=float(args.soc_bin_step),
        out_series=int(args.series),
        out_parallel=int(args.parallel),
        measured_series=int(args.series),
        measured_parallel=int(args.parallel),
    )
    write_surface_csv(pack_dir / "resistance_surface_pack.csv", surface_pack)

    # - cell-level surface (scaled down from pack)
    surface_cell = aggregate_surface(
        pulses,
        soc_bin_step=float(args.soc_bin_step),
        out_series=1,
        out_parallel=1,
        measured_series=int(args.series),
        measured_parallel=int(args.parallel),
    )
    write_surface_csv(cell_dir / "resistance_surface_cell.csv", surface_cell)

    print(f"Wrote OCV curve: {cell_dir / 'ocv_soc.csv'}")
    print(f"Wrote cell resistance surface: {cell_dir / 'resistance_surface_cell.csv'}")
    print(f"Wrote extracted pulse table: {pack_dir / 'dcir_extracted_pulses_pack.csv'}")
    print(f"Wrote pack resistance surface: {pack_dir / 'resistance_surface_pack.csv'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
