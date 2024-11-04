"""
    animate(mvis::MechanismVisualizer,
            times::Vector{Float64},
            configurations::Vector{Vector{Float64}};
            fps::Float64=60, realtimerate::Float64=1.)

Animate the given mechanism passing through a time-coded series of
configurations by linearly interpolating the configuration vectors.
"""
function animate(mvis::MechanismVisualizer,
                 times::Vector{Float64},
                 configurations::AbstractVector{<:AbstractVector{Float64}};
                 fps::Float64 = 60., realtimerate::Float64 = 1.)
    @assert fps > 0
    @assert 0 < realtimerate < Inf

    state = mvis.state
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    t0, tf = first(times), last(times)
    framenum = 0
    walltime0 = time()
    @throttle framenum while true
        t = min(tf, t0 + (time() - walltime0) * realtimerate)
        q = interpolated_configurations(t)
        set_configuration!(state, q)
        rbd.normalize_configuration!(state)
        _render_state!(mvis)
        framenum += 1
        t == tf && break
    end max_rate = fps
end

function MeshCat.Animation(mvis::MechanismVisualizer,
                           times::AbstractVector{<:Real},
                           configurations::AbstractVector{<:AbstractVector{<:Real}};
                           fps::Integer=30)
    @assert axes(times) == axes(configurations)
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    animation = Animation(mvis.visualizer["/meshcat"])
    num_frames = floor(Int, (times[end] - first(times)) * fps)
    for frame in 0 : num_frames
        time = first(times) + frame / fps
        let mvis = mvis, interpolated_configurations = interpolated_configurations, time=time
            atframe(animation,  frame) do
                set_configuration!(mvis, interpolated_configurations(time))
            end
        end
    end
    return animation
end

MeshCat.setanimation!(mvis::MechanismVisualizer, args...; kw...) =
    setanimation!(visualizer(mvis), args...; kw...)

function MeshCat.setanimation!(mvis::MechanismVisualizer,
                      times::AbstractVector{<:Real},
                      configurations::AbstractVector{<:AbstractVector{<:Real}};
                      fps::Integer=30,
                      play::Bool=true,
                      repetitions::Integer=1)
    Base.depwarn("""
    `setanimation!(mvis, times, configurations; ..)` is deprecated. Instead, you can construct an `Animation` and then call `setanimation!` with the result.

    For example, if you previously did:

    ```
    setanimation!(mvis, times, configurations, fps=30, play=true)
    ```

    You should now do:

    ```
    animation = Animation(mvis, times, configurations, fps=25)
    setanimation!(mvis, animation, play=true)
    ```
    """, :setanimation_t_q)
    animation = Animation(mvis, times, configurations, fps=fps)
    setanimation!(mvis, animation, play=play, repetitions=repetitions)
end
