struct MechanismVisualizer{M <: MechanismState}
    state::M
    visualizer::Visualizer
    modcount::Int

    function MechanismVisualizer(state::M,
                                 source::AbstractGeometrySource=Skeleton(),
                                 vis::Visualizer=Visualizer()) where {M <: MechanismState}
        mvis = new{M}(state, vis, rbd.modcount(state.mechanism))
        elements = visual_elements(state.mechanism, source)
        _set_mechanism!(mvis, elements)
        _render_state!(mvis)
        mvis
    end
end

MechanismVisualizer(m::Mechanism, args...) = MechanismVisualizer(MechanismState{Float64}(m), args...)

mechanism(mvis::MechanismVisualizer) = mvis.state.mechanism

to_affine_map(tform::Transform3D) = AffineMap(rotation(tform), translation(tform))

function _set_mechanism!(mvis::MechanismVisualizer, elements::AbstractVector{<:VisualElement})
    for (i, element) in enumerate(elements)
        setelement!(mvis, element, "geometry_$i")
    end
end

Base.getindex(mvis::MechanismVisualizer, x...) = getindex(mvis.visualizer, x...)

# TODO: much of this information can be cached if this
# method becomes a performance bottleneck.
# We can probably just put `@memoize` from Memoize.jl right here.
function Base.getindex(mvis::MechanismVisualizer, frame::CartesianFrame3D)
    body = rbd.body_fixed_frame_to_body(mechanism(mvis), frame)
    mvis[body][string(frame)]
end

function Base.getindex(mvis::MechanismVisualizer, body::RigidBody)
    path = _path(mechanism(mvis), body)
    mvis[path...]
end

function setelement!(mvis::MechanismVisualizer, element::VisualElement, name::AbstractString="<element>")
    setelement!(mvis, element.frame, element.geometry, MeshLambertMaterial(color=element.color), name)
    settransform!(mvis[element.frame][name], element.transform)
end

function setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, object::AbstractObject, name::AbstractString="<element>")
    body = rbd.body_fixed_frame_to_body(mechanism(mvis), frame)
    definition = rbd.frame_definition(body, frame)
    frame_vis = mvis[frame]
    settransform!(frame_vis, to_affine_map(definition))
    setobject!(frame_vis[name], object)
end

setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, args...) = setelement!(mvis, frame, Object(geometry), args...)
setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, material::AbstractMaterial, args...) = setelement!(mvis, frame, Object(geometry, material), args...)

# Special cases for visualizing frames and points
setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, scale::Real=0.5, args...) = setelement!(mvis, frame, Triad(scale), args...)
setelement!(mvis::MechanismVisualizer, point::Point3D, radius::Real=0.05, args...) = setelement!(mvis, point.frame, HyperSphere(Point(point.v[1], point.v[2], point.v[3]), convert(eltype(point.v), radius)), args...)

function _path(mechanism, body)
    body_ancestors = ancestors(body, mechanism.tree)
    path = string.(reverse(body_ancestors))
end


function _render_state!(mvis::MechanismVisualizer, state::MechanismState=mvis.state)
    @assert mvis.state.mechanism === state.mechanism
    if rbd.modcount(state.mechanism) != mvis.modcount
        error("Mechanism has been modified after creating the visualizer. Please create a new MechanismVisualizer")
    end
    vis = mvis.visualizer
    tree = mechanism(mvis).tree # TODO: tree accessor?
    for body in vertices(tree)
        if body == root(tree)
            settransform!(vis, to_affine_map(transform_to_root(state, body)))
        else
            parent = source(edge_to_parent(body, tree), tree)
            tform = relative_transform(state, default_frame(body), default_frame(parent))
            settransform!(mvis[body], to_affine_map(tform))
        end
    end
end

function rbd.set_configuration!(mvis::MechanismVisualizer, args...)
    set_configuration!(mvis.state, args...)
    _render_state!(mvis)
end

rbd.configuration(mvis::MechanismVisualizer, args...) = configuration(mvis.state, args...)

MeshCat.IJuliaCell(mvis::MechanismVisualizer, args...; kw...) = MeshCat.IJuliaCell(mvis.visualizer, args...; kw...)
Base.open(mvis::MechanismVisualizer, args...; kw...) = open(mvis.visualizer, args...; kw...)
Base.wait(mvis::MechanismVisualizer) = wait(mvis.visualizer)
