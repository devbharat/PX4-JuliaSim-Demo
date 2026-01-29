# Battery asset ingestion scripts

These scripts convert *raw* battery test artifacts (CSV/XLSX/HTML, etc.) into the
processed asset format stored under:

- `src/Workflows/assets/battery/cells/*`
- `src/Workflows/assets/battery/packs/*`

## Example 3290 mAh (8S1P) dataset

Raw inputs are stored under:

- `src/Workflows/assets/battery/raw/example_3290mah_8s1p/`

The main ingestion script:

- `ingest_example_3290mah_8s1p.py`

It produces:

- `cells/example_3290mah_hv/ocv_soc.csv`
- `cells/example_3290mah_hv/resistance_surface_cell.csv`
- `packs/example_8s1p_3290mah/resistance_surface_pack.csv`
- `packs/example_8s1p_3290mah/dcir_extracted_pulses_pack.csv` (raw extracted pulse metrics)

## Running

From the repo root:

```bash
python3 scripts/battery_assets/ingest_example_3290mah_8s1p.py
```

The script uses in-repo defaults for the raw input paths, but you can override them:

```bash
python3 scripts/battery_assets/ingest_example_3290mah_8s1p.py \
  --raw-ocv-csv src/Workflows/assets/battery/raw/example_3290mah_8s1p/example_cell_ocv_soc.csv \
  --dcir-dashboard-html src/Workflows/assets/battery/raw/example_3290mah_8s1p/IMPROVED_DCIR_Dashboard.html \
  --series 8 \
  --parallel 1
```

## Notes

- The DCIR dashboard is treated as the source of truth for R0/Rdc extraction because it
  already contains the **key markers** (pre-pulse and minimum voltage) for each pulse.
- Resistance surfaces are generated as **median values** in SOC bins (default: 5% SOC bins)
  for each available ambient temperature.
