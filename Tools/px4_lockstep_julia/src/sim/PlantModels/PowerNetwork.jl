"""PlantModels.PowerNetwork

Simple algebraic power-network abstraction used by plant-integrated models.

This is intentionally **not** a circuit simulator. It captures only:

* which physical propulsors (motors) are powered by which bus
* which batteries feed which bus
* per-bus constant avionics load (W)

The coupled plant model uses this wiring to:
* solve a bus voltage for each bus (algebraically, deterministically)
* distribute bus load current across the batteries on that bus

Type parameters:
* `N`: number of physical propulsors (motors)
* `B`: number of batteries
* `K`: number of buses

All fields are `StaticArrays` so the plant RHS can remain allocation-free.
"""

using StaticArrays

export PowerNetwork

"""Power-network wiring and parameters.

Fields
------
* `bus_for_motor[i]`: bus index (1..K) powering physical motor `i`
* `bus_for_battery[j]`: bus index (1..K) fed by battery `j`
* `avionics_load_w[k]`: constant avionics load on bus `k` (W)
* `share_mode`: current-sharing rule between batteries on a bus
  - `:inv_r0` (default): share proportional to `1/R0`
  - `:equal`: equal share

Notes
-----
The topology is intentionally simple: each battery belongs to a single bus.
Cross-feed / diode OR-ing is intentionally out of scope.
"""
Base.@kwdef struct PowerNetwork{N,B,K}
    bus_for_motor::SVector{N,Int}
    bus_for_battery::SVector{B,Int}
    avionics_load_w::SVector{K,Float64} = zero(SVector{K,Float64})

    share_mode::Symbol = :inv_r0
end
