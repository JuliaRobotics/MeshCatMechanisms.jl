using Test
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using CoordinateTransformations: Translation
using ValkyrieRobot
using NBInclude
using StaticArrays

vis = Visualizer()

@testset "MeshCatMechanisms" begin
    @testset "URDF mechanism" begin
        urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
        robot = parse_urdf(urdf, remove_fixed_tree_joints=false)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
        if !haskey(ENV, "CI")
            open(mvis)
            wait(mvis)
        end
        set_configuration!(mvis, [0, -0.5])
        set_configuration!(mvis, findjoint(robot, "shoulder"), 1.0)
        @test configuration(mvis) == [1.0, -0.5]
        @test configuration(mvis, findjoint(robot, "shoulder")) == [1.0]
        copyto!(mvis, [0.1, -0.6, 0.0, 0.0])
        @test configuration(mvis) == [0.1, -0.6]

        setelement!(mvis, default_frame(bodies(robot)[end]))
        setelement!(mvis, Point3D(default_frame(bodies(robot)[3]), 0.2, 0.2, 0.2))

        @testset "simulation and animation" begin
            state = MechanismState(robot, randn(2), randn(2))
            t, q, v = simulate(state, 5.0);
            animate(mvis, t, q)
            setanimation!(mvis, t, q)
        end
    end

    @testset "URDF with fixed joints" begin
        urdf = joinpath(@__DIR__, "urdf", "Acrobot_fixed.urdf")
        robot = parse_urdf(urdf)
        delete!(vis)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
        set_configuration!(mvis, [0.5])

        @testset "simulation and animation" begin
            state = MechanismState(robot, randn(1), randn(1))
            t, q, v = simulate(state, 5.0);
            animate(mvis, t, q)
            setanimation!(mvis, t, q)
        end
    end

    @testset "Valkyrie" begin
        val = Valkyrie();
        delete!(vis)
        mvis = MechanismVisualizer(
           val.mechanism,
           URDFVisuals(ValkyrieRobot.urdfpath(), package_path=[ValkyrieRobot.packagepath()]),
           vis[:val_urdf])
    end

    @testset "valkyrie inertias" begin
        val = Valkyrie();
        mvis = MechanismVisualizer(
           val.mechanism,
           Skeleton(),
           vis[:val_inertia])
        settransform!(vis[:val_inertia], Translation(0, 2, 0))
    end

    @testset "visualization during simulation" begin
        urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
        robot = parse_urdf(urdf, remove_fixed_tree_joints=false)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis[:acrobot][:robot])
        settransform!(vis[:acrobot], Translation(0, -2, 0))
        result = DynamicsResult{Float64}(robot)
        function damped_dynamics!(vd::AbstractArray, sd::AbstractArray, t, state)
            damping = 2.
            τ = -damping * velocity(state)
            dynamics!(result, state, τ)
            copyto!(vd, result.v̇)
            copyto!(sd, result.ṡ)
            nothing
        end
        state = MechanismState(robot, randn(2), randn(2))
        integrator = MuntheKaasIntegrator(state, damped_dynamics!, runge_kutta_4(Float64), MeshCatSink(mvis))
        integrate(integrator, 10., 1e-3, max_realtime_rate = 1.)
    end

    @testset "position bounds resolution" begin
        j = Joint("foo", Revolute(SVector(1., 0, 0)))
        RigidBodyDynamics.position_bounds(j) .= RigidBodyDynamics.Bounds(-1.234, 0.0)
        MeshCatMechanisms.Manipulate.sliders(j)
        MeshCatMechanisms.Manipulate.widget(j)
    end

    @testset "manipulation" begin
        delete!(vis)
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
        mvis = MechanismVisualizer(mechanism)
        widget = manipulate!(mvis)
    end

    @testset "manipulation - negative bounds issue" begin
        mechanism = rand_chain_mechanism(Float64, Revolute{Float64})
        joint = first(joints(mechanism))
        joint.position_bounds .= RigidBodyDynamics.Bounds(-2.0, -1.0)
        mvis = MechanismVisualizer(mechanism)
        widget = manipulate!(mvis)
    end

    @testset "examples" begin
        dir = joinpath(@__DIR__, "..", "examples")
        for file in readdir(dir)
            base, ext = splitext(file)
            if ext == ".jl"
                println("Running $file")
                include(joinpath(dir, file))
            elseif ext == ".ipynb"
                println("Running notebook $file")
                @nbinclude(joinpath(dir, file))
            end
        end
    end
end
