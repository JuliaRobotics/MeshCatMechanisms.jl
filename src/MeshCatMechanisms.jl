__precompile__()

module MeshCatMechanisms

export MechanismVisualizer

using LightXML
using MeshCat
using MeshCat: AbstractMaterial, AbstractObject, MeshMaterial
using CoordinateTransformations
using GeometryTypes
using RigidBodyDynamics
using RigidBodyDynamics.Graphs
const rbd = RigidBodyDynamics
import MeshIO
using FileIO: load
using ColorTypes: RGBA
using Interpolations: interpolate, Gridded, Linear
using LoopThrottle: @throttle

include("visualizer.jl")
include("animate.jl")
include("parse_urdf.jl")

end # module
