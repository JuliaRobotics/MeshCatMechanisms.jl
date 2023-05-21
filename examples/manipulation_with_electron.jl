# This example demonstrates basic manipulation of a visualized
# mechanism in a standalone window.

#  For a similar demo rendered in Jupyter instead, see the
#  `interactive_manipulation.ipynb` notebook.

# Import our packages
using MeshCatMechanisms
using RigidBodyDynamics

# Electron provides the standalone window we'll use to display
# the visualizer and controls
import Electron

# Create a random mechanism and its associated visualizer

# seed the random number generator so we get repeatable results
import Random
Random.seed!(75)
mechanism = rand_chain_mechanism(Float64,
    [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
mvis = MechanismVisualizer(mechanism, Skeleton(randomize_colors=true))

# Open the visualizer in a new window
open(mvis, Electron.Application())

# Create sliders to manipulate the visualizer's configuration
widget = manipulate!(mvis)
body!(Electron.Application(), widget)
