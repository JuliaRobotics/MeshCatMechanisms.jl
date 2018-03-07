struct MechanismVisualizer{M <: MechanismState}
    state::M
    visualizer::Visualizer
    modcount::Int

    function MechanismVisualizer(state::M,
                                 frame_to_visuals::AbstractVector{<:VisualElement}=create_skeleton(state.mechanism),
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
    MechanismVisualizer(m, parse_urdf_visuals(fname, m.mechanism; kw...), args...)

to_affine_map(tform::Transform3D) = AffineMap(rotation(tform), translation(tform))

function _set_mechanism!(mvis::MechanismVisualizer, elements::AbstractVector{<:VisualElement})
    for (i, element) in enumerate(elements)
        _set_element!(mvis, element, "geometry_$i")
    end
end

function _set_element!(mvis::MechanismVisualizer, element::VisualElement, name::AbstractString)
    mechanism = mvis.state.mechanism
    vis = mvis.visualizer
    tree = mechanism.tree
    # TODO: much of this information can be cached if this
    # method becomes a performance bottleneck
    body = rbd.body_fixed_frame_to_body(mechanism, element.frame)
    body_ancestors = rbd.Graphs.ancestors(body, tree)
    path = vcat(string.(reverse(body_ancestors)), string(element.frame))
    frame_vis = vis[path...]
    definition = rbd.frame_definition(body, element.frame)
    settransform!(frame_vis, to_affine_map(definition))
    setobject!(frame_vis[name], element.geometry, MeshLambertMaterial(color=element.color))
    settransform!(frame_vis[name], element.transform)
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
            path = string.(reverse(body_ancestors))
            tform = relative_transform(state, default_frame(body), default_frame(body_ancestors[2]))
            settransform!(vis[path...], to_affine_map(tform))
        end
    end
end

function rbd.set_configuration!(mvis::MechanismVisualizer, args...)
    set_configuration!(mvis.state, args...)
    _render_state!(mvis)
end

MeshCat.IJuliaCell(mvis::MechanismVisualizer) = MeshCat.IJuliaCell(mvis.visualizer)