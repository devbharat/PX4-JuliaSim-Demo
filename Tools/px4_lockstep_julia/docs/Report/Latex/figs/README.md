# Figures: how to regenerate

This folder holds figures referenced by the LaTeX report. The commands below are intended to be run from the repo root.

## Determinism (Section 01b)
- **determinism_drift.png**
  - Run: generate two identical replays and diff traces.
  - Example:
    ```bash
    Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh
    # TODO: add a small helper to diff two replays from the same recording and plot |Δpos|,|Δvel|,|Δatt|.
    ```
- **replay_rtf_long.png**
  - Replay-only realtime factor for a long mission (no PX4 calls during replay sweep).
  - Example:
    ```bash
    Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare_replay_rtf_long.sh
    MPLCONFIGDIR=/tmp/mplconfig python - <<'PY'
    import csv
    import matplotlib.pyplot as plt
    from pathlib import Path
    rows = []
    with open('Tools/px4_lockstep_julia/examples/replay/out_long/iris_long_rtf.csv') as f:
        rows = list(csv.DictReader(f))
    solvers = [row['solver'] for row in rows]
    rtf = [float(row['rtf']) for row in rows]
    fig, ax = plt.subplots(figsize=(5.0, 3.2), dpi=150)
    ax.bar(solvers, rtf, color='tab:green', alpha=0.85)
    ax.set_ylabel('Realtime factor (sim_s / wall_s)')
    ax.set_title('Replay-only integrator RTF (no PX4 calls)')
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    out = Path('Tools/px4_lockstep_julia/docs/Report/Latex/figs/replay_rtf_long.png')
    fig.savefig(out)
    print(f'Saved {out}')
    PY
    ```

## Runtime / Engine (Section 01)
- **event_axis_union.png**
  - Run: small script that prints and plots the union axis for a short horizon.
  - Example:
    ```bash
    # TODO: add a tiny script under Tools/px4_lockstep_julia/examples/verification/plot_event_axis_union.jl
    ```
- **contact_step_zoom.png**
  - Run: long‑mission script with contact enabled, then plot contact traces.
  - Example:
    ```bash
    Tools/px4_lockstep_julia/scripts/run_iris_lockstep_example_surface_rc_thermal_long.sh
    python Tools/px4_lockstep_julia/scripts/plot_sim_log.py \
      --log Tools/sim_log.csv \
      --contact-output Tools/sim_plot_contact.png
    ```

## Frames / Coordinates (Section 02)
- **ned_lla_error.png**
  - Run: compare `ned_to_lla` approximation vs a WGS84 reference at increasing distance.
  - Example:
    ```bash
    # TODO: add a small script under Tools/px4_lockstep_julia/examples/verification/ned_lla_error.jl
    ```

## Plant State / Integrators (Section 03)
- **adaptive_steps_hist.png**
  - Run: collect adaptive RK45 substep sizes with quantize_us on/off.
  - Example:
    ```bash
    # TODO: add a small script under Tools/px4_lockstep_julia/examples/verification/plot_adaptive_steps.jl
    ```

## Rigid Body (Section 04)
- **gyro_coupling_plot.png**
  - Run: extract the tilted‑axis gyro test into a plot.
  - Example:
    ```bash
    JULIA_DEPOT_PATH=Tools/px4_lockstep_julia/.julia_depot \
      julia --project=Tools/px4_lockstep_julia \
      Tools/px4_lockstep_julia/examples/verification/gyro_coupling_regression.jl
    MPLCONFIGDIR=/tmp/mplconfig \
      python Tools/px4_lockstep_julia/scripts/plot_gyro_coupling_regression.py
    ```

## Actuation / Propulsion (Section 05)
- **actuator_step_response.png**
  - Run: plot direct vs second‑order actuator response.
  - Example:
    ```bash
    # TODO: add a small script under Tools/px4_lockstep_julia/examples/verification/plot_actuator_response.jl
    ```
- **motor_spinup.png**
  - Run: motor spin‑up/torque response.
  - Example:
    ```bash
    # TODO: add a small script under Tools/px4_lockstep_julia/examples/verification/plot_motor_spinup.jl
    ```

## Power / Battery (Section 06)
- **battery_ocv_curve.png**
  - Run: plot cell + pack OCV vs SOC.
  - Source data: `src/Workflows/assets/battery/.../ocv_soc.csv`.
- **battery_r0_surface.png**
  - Run: plot R0/DCIR vs SOC and temperature (heatmap).
  - Source data: `src/Workflows/assets/battery/.../resistance_surface_*.csv`.
- **battery_runtime_trace.png**
  - Run: long mission with thermal enabled.
  - Example:
    ```bash
    Tools/px4_lockstep_julia/scripts/run_iris_lockstep_example_surface_rc_thermal_long.sh
    python Tools/px4_lockstep_julia/scripts/plot_sim_log.py \
      --log Tools/sim_log.csv \
      --battery-output Tools/sim_plot_battery.png
    ```
- **bus_solve_residual.png**
  - Run: capture bus‑solve residuals over a randomized sweep.
  - Example:
    ```bash
    # TODO: add a diagnostic script or dump from contract tests.
    ```

## Environment / Contact (Section 07)
- **contact_touchdown.png**
  - Run: contact trace plot from a long mission.
  - Example:
    ```bash
    python Tools/px4_lockstep_julia/scripts/plot_sim_log.py \
      --log Tools/sim_log.csv \
      --contact-output Tools/sim_plot_contact.png
    ```
- **wind_ou_trace.png**
  - Run: OU wind component trace with configured sigma/tau.
  - Example:
    ```bash
    # TODO: add a small OU wind trace plot script.
    ```

## Estimator / Scenario (Section 08)
- **estimator_delay.png**
  - Run: compare true vs delayed estimate on a step.
  - Example:
    ```bash
    # TODO: add a small script using DelayedEstimator.
    ```
- **scenario_timeline.png**
  - Run: plot scenario event schedule (arm, mission start, faults).
  - Example:
    ```bash
    # TODO: add a small script that reads scenario and plots event times.
    ```

## PX4 Bridge (Section 09)
- **uorb_schedule_alignment.png**
  - Run: show aligned vs misaligned injection periods relative to dt_ap.
  - Example:
    ```bash
    # TODO: add a small schedule validator plot.
    ```

## Validation (Section 11)
- **integrator_sweep_errors.png**
  - Run: `Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh` and plot error series.
- **mission_validation.png**
  - Run: long mission with lockstep library present; plot XY + altitude.
- **contact_energy_dissipation.png**
  - Run: drop‑test script and plot kinetic energy vs time around impact.

## Record/Replay (Section 12)
- **record_vs_replay.png**
  - Run: overlay record vs replay and plot error.

## Extension Points (Section 13)
- **custom_injector_effect.png**
  - Run: custom periodic injection example and log its effect on a topic.
