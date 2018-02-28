"""
Interpolations.jl requires that one(::Type{T}) be defined for any data
type we want to interpolate. Rather than defining one(::Type{Vector}) here,
which might have unforeseen consequences in other packages, we'll create
a very simple wrapper type that just knows one() and *
"""
struct InterpolatableArray{A <: AbstractArray}
    data::A
end

Base.one(::Type{InterpolatableArray{A}}) where {A} = 1
Base.:*(n::Number, a::InterpolatableArray) = n * a.data

rbd.normalize_configuration!(joint_type::JointType, q) = nothing
function rbd.normalize_configuration!(joint_type::QuaternionFloating, q)
    n = norm(q[1:4])
    for i = 1:4
        q[i] /= n
    end
end

"""
    animate(vis::MechanismVisualizer,
            times::Vector{Float64},
            configurations::Vector{Vector{Float64}};
            fps::Float64=60, realtimerate::Float64=1.)

Animate the given mechanism passing through a time-coded series of
configurations by linearly interpolating the configuration vectors.
"""
function animate(vis::MechanismVisualizer,
                 times::Vector{Float64},
                 configurations::AbstractVector{<:AbstractVector{Float64}};
                 fps::Float64 = 60., realtimerate::Float64 = 1.)
    @assert fps > 0
    @assert 0 < realtimerate < Inf

    state = vis.state
    interp_values = [InterpolatableArray(c) for c in configurations]
    interpolated_configurations = interpolate((times,), interp_values, Gridded(Linear()))
    t0, tf = first(times), last(times)
    framenum = 0
    walltime0 = time()
    @throttle framenum while true
        t = min(tf, t0 + (time() - walltime0) * realtimerate)
        q = interpolated_configurations[t]
        set_configuration!(state, q)
        rbd.normalize_configuration!(state)
        _render_state!(vis)
        framenum += 1
        t == tf && break
    end max_rate = fps
end

animate(mechanism::Mechanism, times::Vector{Float64},
        configurations::Vector{Vector{Float64}}) =
    animate(Visualizer(mechanism), mechanism, times, configurations)
