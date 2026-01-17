# Component Design Docs

This directory collects component-level design notes for `PX4Lockstep.jl`.

## Status

- [x] `PX4Lockstep` C ABI wrapper (`src/PX4Lockstep.jl`)
- [x] Fixed-step semantics via `Sim.Runtime.Engine` + `timeline.phys`
- [x] Event-driven integration (`src/sim/Runtime/Engine.jl`)
- [x] Integrators (`src/sim/Integrators.jl`)
- [x] Plant state model (`src/sim/Plant.jl`)
- [x] Rigid-body core (`src/sim/RigidBody.jl`)
- [x] Environment models (`src/sim/Environment.jl`)
- [x] Vehicles (`src/sim/Vehicles.jl`)
- [x] Propulsion (`src/sim/Propulsion.jl`)
- [x] Powertrain (`src/sim/Powertrain.jl`)
- [x] Scenario/events (`src/sim/Scenario.jl`, `src/sim/Events.jl`)
- [x] Scheduling (`src/sim/Scheduling.jl`)
- [x] Estimators (`src/sim/Estimators.jl`)
- [x] Noise utilities (`src/sim/Noise.jl`)
- [x] Contacts (`src/sim/Contacts.jl`)
- [x] Logging (`src/sim/Logging.jl`)
- [x] Verification utilities (`src/sim/Verification.jl`)
- [x] Autopilot bridge (`src/sim/Autopilots.jl`)
- [x] Fault signals (`src/sim/Faults.jl`)
- [x] Plant output protocol (`src/sim/PlantInterface.jl`)
- [x] Record/replay primitives (`src/sim/Recording/`, `src/sim/Sources/`, `src/sim/Runtime/`)
