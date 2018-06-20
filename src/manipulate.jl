using RigidBodyDynamics: Bounds, position_bounds, lower, upper
import InteractBase
using InteractBase: slider, Widget, observe, vbox, widget
using WebIO: Node

function remove_infs(b::Bounds, default=Float64(π))
    Bounds(isfinite(lower(b)) ? lower(b) : -default,
           isfinite(upper(b)) ? upper(b) : default)
end

slider_range(joint::Joint) = remove_infs.(position_bounds(joint))
function slider_range(joint::Joint{T, <: QuaternionFloating}) where {T}
    defaults = [1., 1, 1, 1, 10, 10, 10]
    remove_infs.(position_bounds(joint), defaults)
end

slider_labels(joint::Joint) = [string("q", i) for i in 1:num_positions(joint)]
slider_labels(joint::Joint{T, <:QuaternionFloating}) where {T} = ["rw", "rx", "ry", "rz", "x", "y", "z"]

function sliders(joint::Joint, values=zeros(num_positions(joint));
                 bounds=slider_range(joint),
                 labels=slider_labels(joint),
                 resolution=0.01, prefix="")
    map(bounds, labels, values) do b, label, value
        slider(lower(b):resolution:upper(b),
               value=value,
               label=string(prefix, label))
    end
end

function combined_observable(::Joint, sliders::AbstractVector{<:Widget})
    map((args...) -> vcat(args...), observe.(sliders)...)
end

function combined_observable(::Joint{T, <:QuaternionFloating}, sliders::AbstractVector{<:Widget}) where T
    map(observe.(sliders)...) do rw, rx, ry, rz, x, y, z
        n = norm([rw, rx, ry, rz])
        if n == 0
            n = 1.0
        end
        [rw / n, rx / n, ry / n, rz / n, x, y, z]
    end
end

function InteractBase.widget(joint::Joint, initial_value=zeros(num_positions(joint)); prefix=string(joint, '.'))
    s = sliders(joint, initial_value, prefix=prefix)
    observable = combined_observable(joint, s)
    node = Node(:div, vbox(s...))
    Widget{:rbd_joint}(node, observable)
end

function manipulate!(callback::Function, state::MechanismState)
    joint_list = joints(state.mechanism)
    widgets = widget.(joint_list, configuration.(state, joint_list))
    obs = map(observe.(widgets)...) do signals...
        for i in 1:length(joint_list)
            set_configuration!(state, joint_list[i], signals[i])
        end
        callback(state)
    end
    node = Node(:div, vbox(widgets...))
    Widget{:rbd_manipulator}(node, obs)
end

function manipulate!(callback::Function, vis::MechanismVisualizer)
    manipulate!(vis.state) do x
        set_configuration!(vis, configuration(x))
        callback(x)
    end
end

manipulate!(vis::MechanismVisualizer) = manipulate!(x -> nothing, vis)
