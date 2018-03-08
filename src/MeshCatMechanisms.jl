__precompile__()

module MeshCatMechanisms

export MechanismVisualizer,
       animate,
       MeshCatSink,
       Skeleton,
       URDFVisuals,
       visual_elements

using MeshCat
using MeshCat: AbstractMaterial, AbstractObject, MeshMaterial
using CoordinateTransformations
using RigidBodyDynamics
const rbd = RigidBodyDynamics
using Interpolations: interpolate, Gridded, Linear
using LoopThrottle: @throttle
using MechanismGeometries: visual_elements, VisualElement, Skeleton, URDFVisuals, AbstractGeometrySource

include("visualizer.jl")
include("animate.jl")
include("ode_callback.jl")

end # module
