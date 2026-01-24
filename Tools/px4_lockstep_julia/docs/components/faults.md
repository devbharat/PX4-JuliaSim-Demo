# Fault Signals

## Role

`src/sim/Faults.jl` defines a compact `FaultState` that represents motor disables,
battery disconnect, and sensor fault masks as a first-class bus signal.

## Key decisions and rationale

- **Bitmask encoding:** motor and sensor faults are encoded in bitmasks to keep the
  fault signal small and deterministic.
- **Sample-and-hold semantics:** faults are treated as held inputs between event
  boundaries, making them record/replay friendly.
- **No mutation side channels:** scenarios publish faults instead of mutating propulsion
  or battery objects directly, reducing hidden coupling.

## Integration contracts

- Plant dynamics must treat `motor_disable_mask` as a duty clamp and
  `battery_connected` as a bus disconnect.
- Fault semantics are applied consistently by the canonical runtime regardless of
  whether a fixed `timeline.phys` axis is present.
- `SENSOR_FAULT_EST_FREEZE` is consumed by the estimator source; other bits are reserved.

## Caveats

- Battery disconnect holds SOC constant but Thevenin polarization voltage relaxes
  toward zero.
- More complex failures (jammed rotor, partial thrust loss) require new bus signals.
