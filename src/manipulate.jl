module Manipulate

export manipulate!

using MeshCatMechanisms
using RigidBodyDynamics
using RigidBodyDynamics: Bounds, position_bounds, lower, upper
using InteractBase: slider, Widget, observe, vbox
using OrderedCollections: OrderedDict

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

function sliders(joint::Joint, values=clamp.(zeros(num_positions(joint)), position_bounds(joint));
                 bounds=slider_range(joint),
                 labels=slider_labels(joint),
                 resolution=0.01, prefix="")
    map(bounds, labels, values) do b, label, value
        num_steps = ceil(Int, (upper(b) - lower(b)) / resolution)
        r = range(lower(b), stop=upper(b), length=num_steps)
        slider(range(lower(b), stop=upper(b), length=num_steps),
               value=clamp(value, first(r), last(r)),
               label=string(prefix, label))
    end
end

function combined_observable(joint::Joint, sliders::AbstractVector)
    map(observe.(sliders)...) do args...
        q = vcat(args...)
        normalize_configuration!(q, joint)
        q
    end
end

function widget(joint::Joint{T, <:Fixed}, args...) where T
    Widget{:rbd_joint}()
end

function widget(joint::Joint, initial_value=clamp.(zeros(num_positions(joint)), position_bounds(joint)); prefix=string(joint, '.'))
    s = sliders(joint, initial_value, prefix=prefix)
    keys = Symbol.(slider_labels(joint))
    w = Widget{:rbd_joint}(OrderedDict(zip(keys, s)))
    w.output = combined_observable(joint, s)
    w.layout = x -> vbox(s...)
    w
end

function manipulate!(callback::Function, state::MechanismState)
    joint_list = joints(state.mechanism)
    widgets = widget.(joint_list, configuration.(state, joint_list))
    keys = Symbol.(joint_list)
    w = Widget{:rbd_manipulator}(OrderedDict(zip(keys, widgets)))
    w.output = map(observe.(widgets)...) do signals...
        for i in 1:length(joint_list)
            if num_positions(joint_list[i]) > 0
                set_configuration!(state, joint_list[i], signals[i])
            end
        end
        callback(state)
    end
    w.layout = x -> vbox(widgets...)
    w
end

function manipulate!(callback::Function, vis::MechanismVisualizer)
    manipulate!(vis.state) do x
        set_configuration!(vis, configuration(x))
        callback(x)
    end
end

manipulate!(vis::MechanismVisualizer) = manipulate!(x -> nothing, vis)

end