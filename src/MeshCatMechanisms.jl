__precompile__()

module MeshCatMechanisms

export MechanismVisualizer,
       animate,
       MeshCatSink

using LightXML
using MeshCat
using MeshCat: AbstractMaterial, AbstractObject, MeshMaterial
using CoordinateTransformations
using GeometryTypes
using RigidBodyDynamics
using RigidBodyDynamics.Graphs
import RigidBodyDynamics: OdeIntegrators
const rbd = RigidBodyDynamics
import MeshIO
using FileIO: load
using ColorTypes: RGBA
using Interpolations: interpolate, Gridded, Linear
using LoopThrottle: @throttle
using MechanismGeometries

include("visualizer.jl")
include("animate.jl")
include("parse_urdf.jl")
include("ode_callback.jl")

end # module
