__precompile__()

module MeshCatMechanisms

export MechanismVisualizer,
       animate,
       MeshCatSink,
       parse_urdf_visuals,
       create_skeleton

using MeshCat
using MeshCat: AbstractMaterial, AbstractObject, MeshMaterial
using CoordinateTransformations
using RigidBodyDynamics
const rbd = RigidBodyDynamics
using Interpolations: interpolate, Gridded, Linear
using LoopThrottle: @throttle
using MechanismGeometries: VisualElement, parse_urdf_visuals, create_skeleton

include("visualizer.jl")
include("animate.jl")
include("ode_callback.jl")

end # module
