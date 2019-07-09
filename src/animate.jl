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
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    t0, tf = first(times), last(times)
    framenum = 0
    walltime0 = time()
    @throttle framenum while true
        t = min(tf, t0 + (time() - walltime0) * realtimerate)
        q = interpolated_configurations(t)
        set_configuration!(state, q)
        rbd.normalize_configuration!(state)
        _render_state!(vis)
        framenum += 1
        t == tf && break
    end max_rate = fps
end

function MeshCat.setanimation!(mvis::MechanismVisualizer,
                      times::AbstractVector{<:Real},
                      configurations::AbstractVector{<:AbstractVector{<:Real}};
                      fps::Integer=30,
                      play::Bool=true,
                      repetitions::Integer=1)
    q0 = copy(configuration(state(mvis)))
    @assert axes(times) == axes(configurations)
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    animation = Animation()
    num_frames = floor(Int, (times[end] - first(times)) * fps)
    for frame in 0:num_frames
        time = first(times) + frame / fps
        let mvis = mvis, interpolated_configurations = interpolated_configurations
            atframe(animation,  frame) do
                set_configuration!(mvis, interpolated_configurations(time))
            end
        end
    end
    setanimation!(visualizer(mvis), animation, play=play, repetitions=repetitions)
    set_configuration!(state(mvis), q0)
end
