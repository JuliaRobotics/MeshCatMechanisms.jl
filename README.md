# MeshCatMechanisms

[![Build Status](https://travis-ci.org/rdeits/MeshCatMechanisms.jl.svg?branch=master)](https://travis-ci.org/rdeits/MeshCatMechanisms.jl)
[![codecov.io](http://codecov.io/github/rdeits/MeshCatMechanisms.jl/coverage.svg?branch=master)](http://codecov.io/github/rdeits/MeshCatMechanisms.jl?branch=master)

MeshCatMechanisms.jl adds support for visualizing mechanisms and robots from [RigidBodyDynamics.jl](https://github.com/tkoolen/RigidBodyDynamics.jl) with [MeshCat.jl](https://github.com/rdeits/MeshCat.jl).

Features:

* Parsing geometry directly from URDF files
* Animation of robot trajectories from RigidBodyDynamics.jl simulations
* Live rendering of simulation progress using the `OdeIntegrators.OdeResultsSink` interface

# Related Projects

MeshCatMechanisms.jl provides similar functionality to [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl), but is built on top of the lighter-weight MeshCat viewer instead of [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl).

# Usage

See [mechanism-demo.ipynb](mechanism-demo.ipynb)

# Examples

![](https://user-images.githubusercontent.com/591886/36703991-41b6991a-1b2c-11e8-8804-24c56ddd94cc.png)
