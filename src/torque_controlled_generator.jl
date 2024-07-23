#= MIT License

Copyright (c) 2024 Uwe Fechner

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

#= Dynamic winch model. Assumptions:
An asynchronous machine with a gearbox is used. The inertia
of drum and motor are combined into one value (stiff coupling). =#


"""
    mutable struct TorqueControlledMachine

Model of a winch with an torqrue controlled generator and a gearbox.
"""
@with_kw mutable struct TorqueControlledMachine <: AbstractWinchModel @deftype Float64
    set::Settings = se()
    "maximal nominal winch force [N]"
    max_winch_force = 4000
    "radius of the drum [m]"
    drum_radius = 0.1615
    "ratio of the gear box"
    gear_ratio = 6.2
    "inertia of the motor, as seen from the motor [kgm²]"
    inertia_motor = 0.082
    "rated nominal motor speed [rad/s]"
    omega_mn = 1460 / 60 * 2π
    "rated torque at nominal motor speed [Nm]"
    tau_n = 121
    " Inertia of the motor, gearbox and drum, as seen from the motor [kgm²]"
    inertia_total = 0.204
    "coulomb friction [N]"
    f_coulomb = 122.0
    "coefficient for the viscous friction [Ns/m]"
    c_vf = 30.6
end

# calculated the motor reactance X [Ohm]
function calc_reactance(wm::TorqueControlledMachine)
    wm.u_nom^2 / (2 * wm.omega_sn * wm.tau_b) 
end

# coulomb friction torque TAU_STATIC [Nm]
function calc_coulomb_friction(wm::TorqueControlledMachine)
    wm.f_coulomb * wm.drum_radius / wm.gear_ratio
end

# viscous friction torque C_F [Nm]
# omega in rad/s
function calc_viscous_friction(wm::TorqueControlledMachine, omega)
    wm.c_vf * omega * wm.drum_radius^2 / wm.gear_ratio^2     
end

function calc_acceleration(wm::AsyncMachine, speed, force; set_torque=nothing, set_speed=nothing, use_brake = false)
    if use_brake
        if abs(set_speed) < 0.9 * wm.v_min
            wm.brake = true
        elseif abs(set_speed) > 1.1 * wm.v_min
            wm.brake = false
        end
        if wm.brake
            # if the brake is active the acceleration proportional to the speed
            # TODO: check if this is physically correct
            return wm.brake_acc * speed
        end
    end
    # limit the acceleration
    if ! isnothing(set_speed)
        if set_speed > speed + 1  
            set_speed = speed + 1
        elseif set_speed < speed - 1
            set_speed = speed - 1
        end
    end
    omega      = wm.gear_ratio/wm.drum_radius * speed
    τ = calc_coulomb_friction(wm) * smooth_sign(omega) + calc_viscous_friction(wm, omega)
    # calculate tau based on the set_torque

    # calculate tau_total based on the friction
    tau_total = tau + wm.drum_radius / wm.gear_ratio * force * 4000.0 / wm.max_winch_force - τ
    # calculate omega_dot_m based on tau_total and the inertia
    omega_dot_m = tau_total/wm.inertia_total
    wm.drum_radius/wm.gear_ratio * omega_dot_m
end

""" Calculate the tether force as function of the synchronous tether speed and the speed. """
function calc_force(wm::TorqueControlledMachine, set_speed, speed)
    acc = calc_acceleration(wm, set_speed, speed, 0.0)
    (wm.gear_ratio/wm.drum_radius) ^ 2 * wm.inertia_total * acc
end
