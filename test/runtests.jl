using MeshCatMechanisms
using Base.Test
using MeshCat: Visualizer, setobject!, settransform!
using RigidBodyDynamics: Mechanism, MechanismState, parse_urdf, set_configuration!
using CoordinateTransformations: Translation
using ValkyrieRobot: Valkyrie

vis = Visualizer()

if !haskey(ENV, "CI")
	open(vis)
	wait(vis)
end

@testset "acrobot" begin
	urdf = "urdf/Acrobot.urdf"
	robot = parse_urdf(Float64, urdf)
	mvis = MechanismVisualizer(robot, urdf, vis[:acrobot, :robot])
	state = MechanismState(robot)
	set_configuration!(state, [1.0, -0.5])
	setstate!(mvis, state)
end

@testset "acrobot with fixed elbow" begin
	urdf = "urdf/Acrobot_fixed.urdf"
	robot = parse_urdf(Float64, urdf)
	mvis = MechanismVisualizer(robot, urdf, vis[:acrobot_fixed, :robot])
	settransform!(vis[:acrobot_fixed], Translation(0, 1, 0))
	state = MechanismState(robot)
	set_configuration!(state, [1.0])
	setstate!(mvis, state)
end

@testset "valkyrie" begin
	val = Valkyrie();
	mvis = MechanismVisualizer(val.mechanism, ValkyrieRobot.urdfpath(), vis[:valkyrie, :robot], 
	                           package_path=[dirname(dirname(ValkyrieRobot.urdfpath()))])
	settransform!(vis[:valkyrie], Translation(0, 2, 1))
	state = MechanismState(val.mechanism)
	setstate!(mvis, state)
end

