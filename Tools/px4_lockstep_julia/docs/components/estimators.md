# Estimators

## Role

`src/sim/Estimators.jl` converts truth state into a PX4-facing estimate with optional
noise, bias, and delay.

## Key Decisions and Rationale

- **Cadence-driven updates:** estimators evolve with an explicit `dt_hint`, not a
  floating-point `t` delta, to avoid cross-platform drift.
- **Quantized delay:** `DelayedEstimator` stores a ring buffer with delays expressed as
  exact integer multiples of the estimator period and fails fast on mismatched `dt`.
- **AR(1) bias + additive noise:** `NoisyEstimator` models slow drift and noise on
  position/velocity/yaw/rates without implementing a full sensor stack.

## Integration Contracts

- Estimator updates happen only on the autopilot cadence.
- Delays must be exact multiples of the estimator step.
- RNG is injected at estimator boundaries only.

## Caveats

- `DelayedEstimator` fails fast if the runtime `dt_hint` does not match the configured
  estimator period.
- Bias/noise models approximate EKF outputs, not raw sensors; they are not drop-in
  sensor models.
- Non-finite or non-positive bias time constants effectively disable bias evolution.
