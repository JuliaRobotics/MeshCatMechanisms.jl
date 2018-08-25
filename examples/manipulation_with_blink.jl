# This example demonstrates basic manipulation of a visualized
# mechanism in a standalone window.

#  For a similar demo rendered in Jupyter instead, see the
#  `interactive_manipulation.ipynb` notebook.

# Import our packages
using MeshCatMechanisms
using RigidBodyDynamics

# Blink provides the standalone window we'll use to display
# the visualizer and controls
using Blink
# Install the Blink.AtomShell if it isn't already installed
AtomShell.isinstalled() || AtomShell.install()

# Create a random mechanism and its associated visualizer

# seed the random number generator so we get repeatable results
if VERSION < v"0.7.0-DEV.3406"
    srand(75)
else
    import Random
    Random.seed!(75)
end
mechanism = rand_chain_mechanism(Float64,
    [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
mvis = MechanismVisualizer(mechanism, Skeleton(randomize_colors=true))

# Open the visualizer in a new window
#
# We don't open windows when we're running this test
# on the Travis CI build servers
if !haskey(ENV, "CI")
    open(mvis, Window())
end

# Create sliders to manipulate the visualizer's configuration
widget = manipulate!(mvis)

# Render those sliders in a new window
#
# We don't open windows when we're running this test
# on the Travis CI build servers
if !haskey(ENV, "CI")
    body!(Window(), widget)
end
