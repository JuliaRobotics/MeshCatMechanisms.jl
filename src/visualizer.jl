struct MechanismVisualizer{M <: MechanismState, V <: AbstractVisualizer}
    state::M
    visualizer::V
    modcount::Int

    function MechanismVisualizer(state::M, vis::V) where {M <: MechanismState, V <: AbstractVisualizer}
        new{M, V}(state, vis, rbd.modcount(state.mechanism))
    end
end

function MechanismVisualizer(state::MechanismState, source::AbstractGeometrySource=Skeleton(), vis::AbstractVisualizer=Visualizer())
    vis = MechanismVisualizer(state, vis)
    setelement!(vis, source)
    vis
end

function setelement!(vis::MechanismVisualizer, source::AbstractGeometrySource)
    elements = visual_elements(mechanism(vis), source)
    _set_mechanism!(vis, elements)
    _render_state!(vis)
end

MechanismVisualizer(m::Mechanism, args...) = MechanismVisualizer(MechanismState{Float64}(m), args...)

state(mvis::MechanismVisualizer) = mvis.state
mechanism(mvis::MechanismVisualizer) = mvis.state.mechanism
visualizer(mvis::MechanismVisualizer) = mvis.visualizer

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

"""
    setelement!(mvis::MechanismVisualizer, element::VisualElement, name::AbstractString="<element>")

Attach the given visual element to the visualizer.
The element's frame will determine how its geometry is attached to the scene
tree, so that any other geometries attached to the same body will all move together.
"""
function setelement!(mvis::MechanismVisualizer, element::VisualElement, name::AbstractString="<element>")
    setelement!(mvis, element.frame, element.geometry, MeshLambertMaterial(color=element.color), name)
    settransform!(mvis[element.frame][name], element.transform)
end

"""
    setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, object::AbstractObject, name::AbstractString="<element>")

Attach the given geometric object (geometry + material) to the visualizer at the given frame
"""
function setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, object::AbstractObject, name::AbstractString="<element>")
    body = rbd.body_fixed_frame_to_body(mechanism(mvis), frame)
    definition = rbd.frame_definition(body, frame)
    frame_vis = mvis[frame]
    settransform!(frame_vis, to_affine_map(definition))
    setobject!(frame_vis[name], object)
end

"""
    setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, name::AbstractString="<element>")

Attach the given geometry to the visualizer at the given frame, using its default material.
"""
setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, name::AbstractString="<element>") = setelement!(mvis, frame, Object(geometry), name)

"""
    setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, material::AbstractMaterial, name::AbstractString="<element>")

Construct an object with the given geometry and material and attach it to the visualizer
"""
setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, geometry::GeometryLike, material::AbstractMaterial, name::AbstractString="<element>") = setelement!(mvis, frame, Object(geometry, material), name)

# Special cases for visualizing frames and points
"""
    setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, scale::Real=0.5, name::AbstractString="<element>")

Add a Triad geometry with the given scale to the visualizer at the specified frame
"""
setelement!(mvis::MechanismVisualizer, frame::CartesianFrame3D, scale::Real=0.5, name::AbstractString="<element>") = setelement!(mvis, frame, Triad(scale), name)

"""
    setelement!(mvis::MechanismVisualizer, point::Point3D, radius::Real=0.05, name::AbstractString="<element>")

Add a HyperSphere geometry with the given radius to the visualizer at the given point
"""
setelement!(mvis::MechanismVisualizer, point::Point3D, radius::Real=0.05, name::AbstractString="<element>") = setelement!(mvis, point.frame, HyperSphere(Point(point.v[1], point.v[2], point.v[3]), convert(eltype(point.v), radius)), name)

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
            continue
        else
            parent = source(edge_to_parent(body, tree), tree)
            tform = relative_transform(state, default_frame(body), default_frame(parent))
            settransform!(mvis[body], to_affine_map(tform))
        end
    end
end

"""
    set_configuration!(mvis::MechanismVisualizer, args...)

Set the configuration of the mechanism visualizer and re-render it.

# Examples
```julia-repl
julia> set_configuration!(vis, [1., 2., 3.])
```

```julia-repl
julia> set_configuration!(vis, findjoint(robot, "shoulder"), 1.0)
```
"""
function rbd.set_configuration!(mvis::MechanismVisualizer, args...)
    set_configuration!(mvis.state, args...)
    _render_state!(mvis)
end

rbd.configuration(mvis::MechanismVisualizer, args...) = configuration(mvis.state, args...)

MeshCat.IJuliaCell(mvis::MechanismVisualizer, args...; kw...) = MeshCat.IJuliaCell(mvis.visualizer, args...; kw...)
Base.open(mvis::MechanismVisualizer, args...; kw...) = open(mvis.visualizer, args...; kw...)
Base.wait(mvis::MechanismVisualizer) = wait(mvis.visualizer)

function Base.copyto!(mvis::MechanismVisualizer, state::Union{MechanismState, AbstractVector})
    copyto!(mvis.state, state)
    _render_state!(mvis)
end
