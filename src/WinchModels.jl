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

export Settings
export AbstractWinchModel, AsyncMachine, MatlabMachine, TorqueControlledMachine # types
export calc_reactance, calc_inductance, calc_resistance                    # helper functions
export calc_coulomb_friction, calc_viscous_friction, smooth_sign           # helper functions
export calc_acceleration, calc_force, update_winch_state!                  # main functions

"""
    abstract type AbstractWinchModel

All winch models must inherit from this type. All methods that are defined on this type.
with all winch models. All exported methods must work on this type. 
"""
abstract type AbstractWinchModel end
const AWM = AbstractWinchModel

"""
    update_brake!(wm::AbstractWinchModel, set_speed)

Switch the brake on if `abs(set_speed)` is below `0.9 * wm.v_min` and off if it is above
`1.1 * wm.v_min`; in between it keeps its state (hysteresis). Returns `wm.brake`.
"""
function update_brake!(wm::AWM, set_speed)
    if abs(set_speed) < 0.9 * wm.v_min
        wm.brake = true
    elseif abs(set_speed) > 1.1 * wm.v_min
        wm.brake = false
    end
    wm.brake
end

"""
    limit_set_speed(wm::AbstractWinchModel, set_speed)

Return `set_speed`, limited such that it differs by at most `max_acc / sample_freq` from
`wm.last_set_speed`.
"""
function limit_set_speed(wm::AWM, set_speed)
    max_step = wm.set.max_acc / wm.set.sample_freq
    clamp(set_speed, wm.last_set_speed - max_step, wm.last_set_speed + max_step)
end

"""
    update_winch_state!(wm::AbstractWinchModel, set_speed; use_brake=false, speed=nothing,
                        force=nothing)

Update the brake state and the rate limited set speed `wm.last_set_speed` of the winch for the
next time step. Call it once per time step, before integrating, and call `calc_acceleration`
with `update_state=false` during the time step, for example in the residual of a DAE solver.

`calc_acceleration` with `update_state=true` (the default) updates the state on every call, so the
state would depend on the number of calls of the residual function by the solver.

Does nothing if `set_speed` is `nothing`. While the brake is active, `wm.last_set_speed` is kept.
The state of the speed controller of the [`TorqueControlledMachine`](@ref) is still updated on
every call of `calc_acceleration` with a `set_speed`.

The keyword arguments `speed` and `force`, the reel-out speed and the tether force at the start
of the time step, are only used by the [`MatlabMachine`](@ref); the other models ignore them.
"""
function update_winch_state!(wm::AWM, set_speed; use_brake=false, speed=nothing, force=nothing)
    isnothing(set_speed) && return nothing
    if use_brake && update_brake!(wm, set_speed)
        return nothing
    end
    wm.last_set_speed = limit_set_speed(wm, set_speed)
    nothing
end

include("async_generator.jl")
include("winch_controller.jl")
include("torque_controlled_generator.jl")
include("matlab_machine.jl")

end