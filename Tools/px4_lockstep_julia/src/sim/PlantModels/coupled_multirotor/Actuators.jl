############################
# Actuator ODE helpers
############################

@inline function _actuator_derivs(
    ::Vehicles.DirectActuators,
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    # Direct actuators are algebraic; their state must be snapped at event boundaries.
    return zero(y), zero(ydot)
end

@inline function _actuator_derivs(
    a::Vehicles.FirstOrderActuators{N},
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    τ = a.τ
    if !(isfinite(τ) && τ > 0.0)
        # Degenerate: treat as direct.
        return zero(y), zero(ydot)
    end
    return (u_cmd - y) / τ, zero(ydot)
end

@inline function _actuator_derivs(
    a::Vehicles.SecondOrderActuators{N},
    y::SVector{N,Float64},
    ydot::SVector{N,Float64},
    u_cmd::SVector{N,Float64},
) where {N}
    ωn = a.ωn
    ζ = a.ζ

    # Rate limiting semantics:
    # * The legacy discrete model clamps `ydot` after each update.
    # * In continuous time this is a differential inclusion; it is approximated by:
    #   - computing dynamics using a clamped effective `ydot` (prevents runaway), and
    #   - projecting `ydot` back into bounds after each accepted integration interval
    #     (via `plant_project`).
    rl = a.rate_limit
    ydot_eff = ydot
    if isfinite(rl)
        ydot_eff = map(v -> clamp(v, -rl, rl), ydot)
    end

    # ẏ = ydot
    y_dot = ydot_eff

    # ÿ = ωn²(u - y) - 2ζωn ẏ
    ydd = (ωn * ωn) .* (u_cmd .- y) .- (2.0 * ζ * ωn) .* ydot_eff

    # If at the limit and accelerating further into saturation, freeze acceleration.
    if isfinite(rl)
        ydd = SVector{N,Float64}(ntuple(i -> begin
            yi = ydot_eff[i]
            ai = ydd[i]
            if yi >= rl && ai > 0.0
                0.0
            elseif yi <= -rl && ai < 0.0
                0.0
            else
                ai
            end
        end, N))
    end

    return y_dot, ydd
end


