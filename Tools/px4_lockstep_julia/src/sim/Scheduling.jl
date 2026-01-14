"""PX4Lockstep.Sim.Scheduling

Deterministic multi-rate scheduling helpers.

The simulator is a fixed-step (constant `dt`) hybrid system. Controllers, estimators and
logging often run at lower rates than the physics loop.

**Important:** Scheduling must never be driven by Float64 time comparisons, because
floating point accumulation can cause cadence jitter (e.g. a "0.01" task occasionally
firing at 0.008/0.012 when `dt=0.002`).

This module therefore schedules periodic tasks using **integer physics step counters**.
"""
module Scheduling

export StepTrigger, due

"""A periodic trigger based on integer physics steps.

`StepTrigger(period_steps; offset_steps=0)` is considered due when:

    step >= offset_steps && (step - offset_steps) % period_steps == 0

This yields *exact* cadences when `period_steps` is an integer.
"""
struct StepTrigger
    period_steps::Int
    offset_steps::Int

    function StepTrigger(period_steps::Int; offset_steps::Int = 0)
        period_steps > 0 || throw(ArgumentError("period_steps must be > 0"))
        offset_steps >= 0 || throw(ArgumentError("offset_steps must be >= 0"))
        return new(period_steps, offset_steps)
    end
end

@inline function due(tr::StepTrigger, step::Int)::Bool
    step < tr.offset_steps && return false
    return ((step - tr.offset_steps) % tr.period_steps) == 0
end

end # module Scheduling
