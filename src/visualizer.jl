struct MechanismVisualizer{M <: MechanismState}
    state::M
    visualizer::Visualizer
    modcount::Int

    function MechanismVisualizer(state::M,
                                 frame_to_visuals::Associative{CartesianFrame3D}=create_visuals(state.mechanism),
                                 vis::Visualizer=Visualizer()) where {M <: MechanismState}
        mvis = new{M}(state, vis, rbd.modcount(state.mechanism))
        _set_mechanism!(mvis, frame_to_visuals)
        _render_state!(mvis)
        mvis
    end
end

MechanismVisualizer(m::Mechanism, args...) = MechanismVisualizer(MechanismState{Float64}(m), args...)
MechanismVisualizer(m::Mechanism, fname::AbstractString, args...; kw...) =
    MechanismVisualizer(m, parse_urdf_visuals(fname, m; kw...), args...)
MechanismVisualizer(m::MechanismState, fname::AbstractString, args...; kw...) =
    MechanismVisualizer(m, parse_urdf_visuals(fname, m; kw...), args...)

to_affine_map(tform::Transform3D) = AffineMap(rotation(tform), translation(tform))

function _set_mechanism!(mvis::MechanismVisualizer, frame_to_visuals)
    mechanism = mvis.state.mechanism
    vis = mvis.visualizer
    tree = mechanism.tree # TODO: tree accessor?
    for vertex in rbd.Graphs.vertices(tree)
        body = vertex
        body_ancestors = rbd.Graphs.ancestors(vertex, tree)
        for definition in rbd.frame_definitions(body)
            frame = definition.from
            path = Symbol.(vcat(string.(reverse(body_ancestors)), string(frame)))
            frame_vis = vis[path...]
            if frame in keys(frame_to_visuals)
                settransform!(frame_vis, to_affine_map(definition))
                for (i, (object, tform)) in enumerate(frame_to_visuals[frame])
                    obj_vis = frame_vis[Symbol("geometry_", i)]
                    setobject!(obj_vis, object)
                    settransform!(obj_vis, tform)
                end
            end
        end
    end
end

function _render_state!(mvis::MechanismVisualizer, state::MechanismState=mvis.state)
    @assert mvis.state.mechanism === state.mechanism
    if rbd.modcount(state.mechanism) != mvis.modcount
        error("Mechanism has been modified after creating the visualizer. Please create a new MechanismVisualizer")
    end
    mechanism = mvis.state.mechanism
    vis = mvis.visualizer
    tree = mechanism.tree # TODO: tree accessor?
    for vertex in rbd.Graphs.vertices(tree)
        body = vertex
        if body == rbd.Graphs.root(tree)
            settransform!(vis, to_affine_map(transform_to_root(state, body)))
        else
            body_ancestors = rbd.Graphs.ancestors(vertex, tree)
            path = Symbol.(string.(reverse(body_ancestors)))
            tform = relative_transform(state, default_frame(body), default_frame(body_ancestors[2]))
            settransform!(vis[path...], to_affine_map(tform))
        end
    end
end

function RigidBodyDynamics.set_configuration!(mvis::MechanismVisualizer, args...)
    set_configuration!(mvis.state, args...)
    _render_state!(mvis)
end
