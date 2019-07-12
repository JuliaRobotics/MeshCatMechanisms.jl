module MeshCatMechanisms

export MechanismVisualizer,
       animate,
       MeshCatSink,
       setelement!,
       manipulate!

# Re-export from MeshCat.jl
export IJuliaCell,
       Triad,
       setanimation!

# Re-export from MechanismGeometries.jl
export VisualElement,
       Skeleton,
       URDFVisuals,
       visual_elements

# Re-export from RigidBodyDynamics.jl
export set_configuration!,
       Point3D

using MeshCat
using CoordinateTransformations
using RigidBodyDynamics
const rbd = RigidBodyDynamics
using RigidBodyDynamics.Graphs: ancestors, edge_to_parent, source, vertices, root
using Interpolations: interpolate, Gridded, Linear
using LoopThrottle: @throttle
using MechanismGeometries: visual_elements, VisualElement, Skeleton, URDFVisuals, AbstractGeometrySource, MeshFile
using GeometryTypes: HyperSphere, Point

include("visualizer.jl")
include("animate.jl")
include("ode_callback.jl")
include("manipulate.jl")
using .Manipulate

end # module
