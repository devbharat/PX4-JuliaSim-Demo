# Known limitations (current main branch)

This list captures intentional simplifications and missing physics in the current
PX4Lockstep Julia sim. It is not exhaustive, but it highlights the largest model
gaps so users know what is and is not being simulated.

## Rigid-body dynamics

- **Diagonal inertia only**: the airframe uses `inertia_diag_kgm2` and the rigid-body
  model assumes a diagonal inertia matrix (no products of inertia).
- **No rotor gyroscopic coupling**: rotor angular momentum is not fed into body
  dynamics (only thrust and reaction torques are applied).

## Aerodynamics / vehicle

- **Multirotor aero is low fidelity**: only simple linear drag + thrust moments, no
  blade flapping, ground effect, or rotor–rotor interference.
- **Fixed-wing surfaces are not modeled** in the core multirotor model; winged
  airframes require dedicated plant models (not present on `main`).

## Propulsion and power

- **Quasi-static motor current** (inductance ignored).
- **Battery model is Thevenin only**; more complex chemistries are not represented.
- **Bus solve uses approximations** (e.g., avionics load treated as constant current
  during the voltage solve).

## Environment

- **Uniform gravity** and a **simplified ISA atmosphere** are used by default.
- **Wind is modelled as a stochastic process** (OU wind) without terrain/CFD effects.

## Sensors and estimation

- **Estimator is not a full sensor stack**: it is a noise model intended to
  approximate EKF output statistics, not raw sensor dynamics.

## Contacts and terrain

- **Flat ground contact only** in the default plant contact model.

## Airframe support

- **Only multirotor airframes are supported on `main`**. Additional airframe types
  require dedicated plant models and are not yet in this branch.
