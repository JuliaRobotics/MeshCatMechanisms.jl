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
        q = interpolated_configurations[t]
        set_configuration!(state, q)
        rbd.normalize_configuration!(state)
        _render_state!(vis)
        framenum += 1
        t == tf && break
    end max_rate = fps
end

using MeshCat: AnimationClip, TransformTrack, Animation, SetAnimation, Path

function setanimation(mvis::MechanismVisualizer,
                      times::Vector{Float64},
                      configurations::AbstractVector{<:AbstractVector{<:Real}},
                      fps::Integer=30)
    state = MechanismState(mechanism(mvis))
    clips = Dict{Path, AnimationClip{TransformTrack}}()
    tree = mechanism(mvis).tree
    @assert indices(times) == indices(configurations)
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    num_frames = floor(Int, (times[end] - first(times)) * fps)
    for frame in 0:num_frames
        time = first(times) + frame / fps
        set_configuration!(state, interpolated_configurations[time])
        for body in vertices(tree)
            if body === root(tree)
                continue
            end
            parent = source(edge_to_parent(body, tree), tree)
            tform = to_affine_map(relative_transform(state, default_frame(body), default_frame(parent)))
            path = mvis[body].path
            clip = get!(clips, path) do
                AnimationClip{TransformTrack}(tracks=[TransformTrack([])], fps=fps)
            end
            track = first(clip.tracks)
            push!(track.frames, frame => tform)
        end
    end
    anim = Animation(collect(clips))
    cmd = SetAnimation(anim, true, 1)
    send(mvis.visualizer.core, cmd)
    return cmd
end
