mutable struct MeshCatSink{M <: MechanismState} <: rbd.OdeIntegrators.OdeResultsSink
    vis::MechanismVisualizer{M}
    min_wall_Δt::Float64
    last_update_wall_time::Float64

    function MeshCatSink(vis::MechanismVisualizer{M}; max_fps::Float64 = 60.) where {M <: MechanismState}
        new{M}(vis, 1 / max_fps, -Inf)
    end
end

function rbd.OdeIntegrators.initialize(sink::MeshCatSink, t, state)
    sink.last_update_wall_time = -Inf
    rbd.OdeIntegrators.process(sink, t, state)
end

function rbd.OdeIntegrators.process(sink::MeshCatSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        set_configuration!(sink.vis, configuration(state))
        sink.last_update_wall_time = time()
    end
    nothing
end
