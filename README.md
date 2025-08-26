# WinchModels
[![Build Status](https://github.com/aenarete/WinchModels.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/aenarete/WinchModels.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/aenarete/WinchModels.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/aenarete/WinchModels.jl)

## Introduction
This package shall implement different models of ground stations for airborne
wind energy systems. A ground station has the following components:
- motor/generator
- brake (optional)
- gearbox (optional)
- drum 

Currently implemented is a model of the 20kW ground station from Delft University of Technology.
In addition, a generic, torque controlled winch without brake is implemented. The torque controlled winch can
operate both with a set_torque or with a set_speed. If a set_speed is provided, then a PI controller for is used
that compares the actual speed and the set speed and controls the torque. The async generator cannot be used
with a set_torque, only with a set_speed which is equal to the synchronous speed of the motor/ generator. 

## Mathematical background
[Torque controlled winch](docs/winch.md).  

## Installation
First, install Julia 1.10 or higher. Then launch Julia and install this package using the package manager.
```julia
using Pkg
pkg"add WinchModels"
```

## Exported types
```julia
AbstractWinchModel
AsyncMachine
TorqueControlledMachine
```

## Main functions
```julia
calc_acceleration(wm::AsyncMachine, speed, force;            set_torque=nothing, set_speed=nothing, use_brake = false)
calc_acceleration(wm::TorqueControlledMachine, speed, force; set_torque=nothing, set_speed=nothing, use_brake = false)
calc_force(wm::AsyncMachine, set_speed, speed)
```
<p align="center"><img src="./docs/working_principle.png" width="800" /></p>

### Plot of function "calc_force"
<p align="center"><img src="./docs/tether_force.png" width="600" /></p>

## Helper functions
```julia
calc_reactance
calc_inductance
calc_resistance
calc_coulomb_friction
calc_viscous_friction
smooth_sign
```
### Plot of function "smooth_sign"
<p align="center"><img src="./docs/smooth_sign.png" width="400" /></p>

## Performance
```julia
using WinchModels, BenchmarkTools, KiteUtils

set = se()
wm = AsyncMachine(set)
@benchmark calc_acceleration(wm, 7.9, 8.0, 100.0)
```
On i7-7700K 17ns for Julia, 1050ns with Python.

## Licence
This project is licensed under the MIT License. Please see the below WAIVER in association with the license.

## WAIVER
Technische Universiteit Delft hereby disclaims all copyright interest in the package “KiteModels.jl” (models for airborne wind energy systems) written by the Author(s).

Prof.dr. H.G.C. (Henri) Werij, Dean of Aerospace Engineering

## See also
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=Uwe+Fecner&pageSize=50&ordering=rating&descending=true)
- The application [KiteViewer](https://github.com/ufechner7/KiteViewer)
- the package [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
- the packages [KiteModels](https://github.com/ufechner7/KiteModels.jl) and [KitePodModels](https://github.com/aenarete/KitePodModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the package [KiteControllers](https://github.com/aenarete/KiteControllers.jl) and [KiteViewers](https://github.com/aenarete/KiteViewers.jl)



