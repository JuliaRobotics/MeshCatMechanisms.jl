# MeshCatMechanisms

[![Build Status](https://travis-ci.org/JuliaRobotics/MeshCatMechanisms.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MeshCatMechanisms.jl)
[![codecov.io](http://codecov.io/github/JuliaRobotics/MeshCatMechanisms.jl/coverage.svg?branch=master)](http://codecov.io/github/JuliaRobotics/MeshCatMechanisms.jl?branch=master)

MeshCatMechanisms.jl adds support for visualizing mechanisms and robots from [RigidBodyDynamics.jl](https://github.com/tkoolen/RigidBodyDynamics.jl) with [MeshCat.jl](https://github.com/rdeits/MeshCat.jl). All geometries are constructed using [MechanismGeometries.jl](https://github.com/rdeits/MechanismGeometries.jl).

Features:

* Parsing geometry directly from URDF files
* Animation of robot trajectories from RigidBodyDynamics.jl simulations
* Live rendering of simulation progress using the `OdeIntegrators.OdeResultsSink` interface
* Interactive manipulation of the mechanism configuration using [InteractBase.jl](https://github.com/piever/InteractBase.jl)

## Related Projects

MeshCatMechanisms.jl provides similar functionality to [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl), but is built on top of the lighter-weight MeshCat viewer instead of [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl).

# Installation

Stable release:

```julia
Pkg.add("MeshCatMechanisms")
```

Latest and greatest:

```julia
Pkg.add("MeshCatMechanisms")
Pkg.clone("https://github.com/rdeits/MechanismGeometries.jl")
Pkg.checkout("MeshCatMechanisms")
Pkg.checkout("MeshCat")
```

# Usage

See [examples/demo.ipynb](examples/demo.ipynb)

# Examples

![](https://user-images.githubusercontent.com/591886/36703991-41b6991a-1b2c-11e8-8804-24c56ddd94cc.png)
