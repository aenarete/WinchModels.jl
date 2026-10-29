#= MIT License

Copyright (c) 2026 Uwe Fechner

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

#= The model of the AsyncMachine, implemented in Matlab. Matlab is called through the Matlab
engine (package extension WinchModelsMATLABExt, loaded by `using MATLAB`) once per time step,
in update_winch_state!. The Matlab function updates the brake and the rate limited set speed
and returns the acceleration and its partial derivatives at the start of the time step. Within
the time step, calc_acceleration uses the first order expansion, so that the residual of the
DAE solver does not call Matlab. =#

"""
    mutable struct MatlabMachine

Model of a winch with an async generator and a gearbox, implemented in Matlab by the function
`async_machine_step.m`, which must be on the Matlab path. Needs `using MATLAB`.

The parameters of the motor and the brake have the same meaning and default values as those of
the [`AsyncMachine`](@ref). They are sent to Matlab before the first time step; change them
before the first call of [`update_winch_state!`](@ref).
"""
@with_kw mutable struct MatlabMachine <: AbstractWinchModel @deftype Float64
    set::Settings
    "nominal motor voltage"
    u_nom = 400.0/sqrt(3)
    "rated synchronous motor speed [rad/s]"
    omega_sn = 1500 / 60 * 2π
    "rated nominal motor speed [rad/s]"
    omega_mn = 1460 / 60 * 2π
    "rated torque at nominal motor speed [Nm]"
    tau_n = 121
    "Rated maximum torque for synchronous motor speed [Nm]"
    tau_b = 363
    "minimal speed of the winch in m/s. If v_set is lower the brake is activated."
    v_min = 0.2
    "linear acceleration of the brake [m/s²]"
    brake_acc = -25.0
    "if the brake of the winch is activated"
    brake::Bool = true
    "last set speed"
    last_set_speed = 0.0
    "reel-out speed at the start of the time step [m/s], the point of the linearization"
    speed0 = 0.0
    "tether force at the start of the time step [N], the point of the linearization"
    force0 = 0.0
    "acceleration at the start of the time step [m/s²]"
    acc0 = 0.0
    "partial derivative of the acceleration with respect to the reel-out speed [1/s]"
    dacc_dv = 0.0
    "partial derivative of the acceleration with respect to the tether force [m/(s² N)]"
    dacc_dF = 0.0
    "if the parameters have been sent to Matlab"
    initialized::Bool = false
end

function MatlabMachine(set::Settings)
    MatlabMachine(set=set)
end

"""
    matlab_step!(wm::MatlabMachine, set_speed, speed, force, use_brake)

Call the Matlab function `async_machine_step` once and store its results in `wm`. Implemented
in the package extension WinchModelsMATLABExt.
"""
function matlab_step!(wm, set_speed, speed, force, use_brake)
    error("MatlabMachine needs the Matlab engine; run `using MATLAB` first.")
end

"""
    update_winch_state!(wm::MatlabMachine, set_speed; use_brake=false, speed=wm.speed0,
                        force=wm.force0)

Call Matlab once: update the brake state and the rate limited set speed, and calculate the
acceleration and its partial derivatives at the reel-out speed `speed` and the tether force
`force`, which are the values at the start of the time step. Call it once per time step,
before integrating.

If `set_speed` is `nothing`, the brake state and the set speed are kept, as for the
[`AsyncMachine`](@ref), but the linearization is updated.
"""
function update_winch_state!(wm::MatlabMachine, set_speed; use_brake=false, speed=nothing,
                             force=nothing)
    speed = something(speed, wm.speed0)
    force = something(force, wm.force0)
    matlab_step!(wm, set_speed, speed, force, use_brake)
    nothing
end

"""
    calc_acceleration(wm::MatlabMachine, speed, force; set_torque=nothing, set_speed=nothing,
                      use_brake=false, update_state=true)

Acceleration of the winch [m/s²]. With `update_state=true` Matlab is called first, see
[`update_winch_state!`](@ref); with `update_state=false` Matlab is not called and the first order
expansion around the point of the last call is returned, which is what a DAE solver needs.
"""
function calc_acceleration(wm::MatlabMachine, speed, force; set_torque=nothing, set_speed=nothing,
                           use_brake=false, update_state=true)
    if update_state
        update_winch_state!(wm, set_speed; use_brake, speed, force)
    end
    if use_brake && wm.brake
        return wm.brake_acc * speed
    end
    wm.acc0 + wm.dacc_dv * (speed - wm.speed0) + wm.dacc_dF * (force - wm.force0)
end
