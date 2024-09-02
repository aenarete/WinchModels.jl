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
    set::Settings
    "minimal speed of the winch in m/s. If v_set is lower the brake is activated."
    v_min = 0.2
    "linear acceleration of the brake [m/s²]"
    brake_acc = -25.0
    "if the brake of the winch is activated"
    brake::Bool = true;
end

function TorqueControlledMachine(set::Settings)
    TorqueControlledMachine(set=set)
end

# calculated the motor reactance X [Ohm]
function calc_reactance(wm::TorqueControlledMachine)
    wm.u_nom^2 / (2 * wm.omega_sn * wm.tau_b) 
end

# coulomb friction torque TAU_STATIC [Nm]
function calc_coulomb_friction(wm::TorqueControlledMachine)
    wm.set.f_coulomb * wm.set.drum_radius / wm.set.gear_ratio
end

# viscous friction torque C_F [Nm]
# omega in rad/s
function calc_viscous_friction(wm::TorqueControlledMachine, omega)
    wm.set.c_vf * omega * wm.set.drum_radius^2 / wm.set.gear_ratio^2     
end

function calc_acceleration(wm::TorqueControlledMachine, speed, force; set_torque=nothing, set_speed=nothing, use_brake = false)
    if use_brake && ! isnothing(set_speed)
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
    omega      = wm.set.gear_ratio/wm.set.drum_radius * speed
    τ = calc_coulomb_friction(wm) * smooth_sign(omega) + calc_viscous_friction(wm, omega)
    # calculate tau based on the set_torque
    K = 1.0
    tau = set_torque * K
    # calculate tau_total based on the friction
    tau_total = tau + wm.set.drum_radius / wm.set.gear_ratio * force  - τ
    # calculate omega_dot_m based on tau_total and the inertia
    omega_dot_m = tau_total/wm.set.inertia_total
    wm.set.drum_radius/wm.set.gear_ratio * omega_dot_m
end

# """ Calculate the tether force as function of the set_speed and speed. """
# function calc_force(wm::TorqueControlledMachine, set_speed, speed)
#     calc_force(wm::TorqueControlledMachine, speed; set_speed)
# end

# """ Calculate the tether force as function of the set_speed, set_torque and speed. """
# # TODO: fix the calculation of the force
# function calc_force(wm::TorqueControlledMachine, speed; set_speed=nothing, set_torque=nothing)
#     acc = calc_acceleration(wm, set_speed, speed, 0.0)
#     (wm.set.gear_ratio/wm.set.drum_radius) ^ 2 * wm.set.inertia_total * acc
# end
