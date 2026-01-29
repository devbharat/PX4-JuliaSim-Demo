# Battery assets

This folder contains **battery-related data assets** used by the PX4 Lockstep SITL simulator.

## Layout

- `cells/<cell_id>/`
  - `meta.toml` — metadata about the cell (chemistry, capacity, voltage limits, references)
  - `ocv_soc.csv` — cell open-circuit voltage curve (SOC → OCV)
  - `resistance_surface_cell.csv` — (optional) cell resistance surface (SOC, temperature → R0/Rdc)

- `packs/<pack_id>/`
  - `meta.toml` — pack configuration (series/parallel) + a reference to the cell asset
  - `resistance_surface_pack.csv` — (optional) pack-level resistance surface (SOC, temperature → R0/Rdc)

- `raw/`
  - Raw vendor/test artifacts used to generate the processed assets (CSV/XLSX/HTML/PDF/etc).

## Conventions

### SOC
All processed curves in `cells/*/` use:

- `soc` in **fraction remaining** (`0.0` empty → `1.0` full)

If a raw dataset uses **“used SOC” / DoD** (`0% used = full`, `100% used = empty`), convert via:

- `soc_remaining = 1 - used_soc/100`

### Voltage scaling
Cell assets always store **cell voltage**.
Pack voltage is computed by:

- `v_pack = series * v_cell`

### Resistance scaling
If resistance is stored at the **cell level**, pack resistance is computed by:

- `r_pack = r_cell * series / parallel`

If resistance is stored at the **pack level** (because the raw measurement was pack-level),
scaling to other series/parallel counts may be inaccurate if wiring/BMS resistance dominates.
Prefer storing cell-level resistance when possible.


### Which resistance column drives bus droop?

Many battery test datasets expose multiple resistance metrics, for example:

- `r0_ohm` — early-dip "ohmic" resistance (e.g. measured at 0.05–0.1 s)
- `rdc_ohm` — end-of-pulse "DCIR" (best proxy for steady loaded voltage sag)
- `r1_ohm` — derived polarization term (`rdc_ohm - r0_ohm`) for future 1-RC fits

In the simulator, the **series droop resistance** used by the bus voltage solver is whatever
column is selected by:

- asset meta: `[resistance_surface_*].r0_col`
- or a BatterySpec / TOML override: `r0_surface_r0_col`

If your goal is to match **loaded voltage under thrust**, prefer `rdc_ohm` and set `r1=c1=0`
(unless you have a validated (R1,C1) fit).

If your goal is to match the instantaneous voltage step at a load transient, prefer `r0_ohm`
(and consider adding a polarization branch).

## Regenerating processed assets

See `scripts/battery_assets/` for ingestion scripts that convert raw artifacts → processed assets.
