using Base.Test
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using ValkyrieRobot
using NBInclude

vis = Visualizer()
if !haskey(ENV, "CI")
    open(vis)
    wait(vis)
end

@testset "MeshCatMechanisms" begin
    @testset "URDF mechanism" begin
        urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
        robot = parse_urdf(Float64, urdf)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
        set_configuration!(mvis, [1.0, -0.5])

        @testset "simulation and animation" begin
            state = MechanismState(robot, randn(2), randn(2))
            t, q, v = simulate(state, 5.0);
            animate(mvis, t, q)
        end
    end

    @testset "URDF with fixed joints" begin
        urdf = joinpath(@__DIR__, "urdf", "Acrobot_fixed.urdf")
        robot = parse_urdf(Float64, urdf)
        RigidBodyDynamics.remove_fixed_tree_joints!(robot)
        delete!(vis)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
        set_configuration!(mvis, [0.5])

        @testset "simulation and animation" begin
            state = MechanismState(robot, randn(1), randn(1))
            t, q, v = simulate(state, 5.0);
            animate(mvis, t, q)
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
        robot = parse_urdf(Float64, urdf)
        delete!(vis)
        mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
        result = DynamicsResult{Float64}(robot)
        function damped_dynamics!(vd::AbstractArray, sd::AbstractArray, t, state)
            damping = 2.
            τ = -damping * velocity(state)
            dynamics!(result, state, τ)
            copy!(vd, result.v̇)
            copy!(sd, result.ṡ)
            nothing
        end
        state = MechanismState(robot, randn(2), randn(2))
        integrator = MuntheKaasIntegrator(state, damped_dynamics!, runge_kutta_4(Float64), MeshCatSink(mvis))
        integrate(integrator, 10., 1e-3, max_realtime_rate = 1.)
    end

    @testset "notebook" begin
        nbinclude(joinpath(@__DIR__, "..", "mechanism-demo.ipynb"))
    end
end
