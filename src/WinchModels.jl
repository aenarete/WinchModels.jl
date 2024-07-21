#= MIT License

Copyright (c) 2020, 2021, 2022 Uwe Fechner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. =#

module WinchModels
using KiteUtils, Parameters

export AbstractWinchModel, AsyncMachine                                    # types
export calc_reactance, calc_inductance, calc_resistance                    # helper functions
export calc_coulomb_friction, calc_viscous_friction, smooth_sign           # helper functions
export calc_acceleration, calc_force                                       # main functions

"""
    abstract type AbstractWinchModel

All winch models must inherit from this type. All methods that are defined on this type.
with all winch models. All exported methods must work on this type. 
"""
abstract type AbstractWinchModel end
const AWM = AbstractWinchModel

include("async_generator.jl")

end