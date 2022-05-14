#= MIT License

Copyright (c) 2022 Uwe Fechner

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
    abstract type AbstractWinchModel

All winch models must inherit from this type. All methods that are defined on this type.
with all winch models. All exported methods must work on this type. 
"""
abstract type AbstractWinchModel end

const AWM = AbstractWinchModel

"""
    mutable struct AsyncGenerator

Model of a winch with an async generator and a gearbox.
"""
@with_kw mutable struct AsyncMachine <: AbstractWinchModel @deftype Float64
    set::Settings = se()
    "maximal nominal winch force [N]"
    max_winch_force = 4000
    "radius of the drum [m]"
    drum_radius = 0.1615
    "ratio of the gear box"
    gear_ratio = 6.2
    "inertia of the motor, as seen from the motor [kgm²]"
    inertia_motor = 0.082
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
    " Inertia of the motor, gearbox and drum, as seen from the motor [kgm²]"
    inertia_total = 0.204
    "minimal speed of the winch in m/s. If v_set is lower the brake is activated."
    v_min = 0.2
    "linear acceleration of the brake [m/s²]"
    brake_acc = -25.0
    "if the brake of the winch is activated"
    brake::Bool = true;
    "coulomb friction [N]"
    f_coulomb = 122.0
    "coefficient for the viscous friction [Ns/m]"
    c_vf = 30.6
end

# calculated the motor reactance X [Ohm]
function calc_reactance(wm::AsyncGenerator)
    wm.u_nom^2 / (2 * wm.omega_sn * wm.tau_b) 
end

# calculate the motor inductance L [H]
function calc_inductance(wm::AsyncGenerator)
    calc_reactance(wm) / wm.omega_sn
end

# calculate the motor resistance R2 [Ohm]
function calc_resistance(wm::AsyncGenerator)
    (wm.u_nom^2 * (wm.omega_sn - wm.omega_mn))/(2wm.omega_sn^2) * (1 / wm.tau_n + sqrt(1 / wm.tau_n^2 - 1 / wm.tau_b^2))
end

# coulomb friction torque TAU_STATIC [Nm]
function calc_coulomb_friction(wm::AsyncGenerator)
    wm.f_coulomb * wm.drum_radius / wm.gear_ratio
end

# viscous friction torque C_F [Nm]
# omega in rad/s
function calc_viscous_friction(wm::AsyncGenerator, omega)
    wm.c_vf * omega * wm.drum_radius^2 / wm.gear_ratio^2     
end

# differentiable version of the sign function
function smooth_sign(x)
    EPSILON = 6
    x / sqrt(x * x + EPSILON * EPSILON)
end

function calc_acceleration(wm::AsyncGenerator, set_speed, speed, force, use_brake = false)
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
    if set_speed > speed + 1  
        set_speed = speed + 1
    elseif set_speed < speed - 1
        set_speed = speed - 1
    end
    omega      = wm.gear_ratio/wm.drum_radius * speed
    omega_sync = wm.gear_ratio/wm.drum_radius * set_speed
    delta = omega_sync - omega
    R2 = calc_resistance(wm)
    L  = calc_inductance(wm)
    if abs(omega_sync) <= wm.omega_sn
        omega_dot_m = (wm.u_nom^2 * R2 * delta) / (wm.omega_sn^2 * (R2^2 + L^2 * delta^2))
    else
        omega_dot_m = (wm.u_nom^2 * R2 * delta) / (omega_sync^2 * (R2^2 + L^2 * delta^2))
    end
    τ = calc_coulomb_friction(wm) * smooth_sign(omega) + calc_viscous_friction(wm, omega)
    omega_dot_m += wm.drum_radius / wm.gear_ratio * force * 4000.0 / wm.max_winch_force - τ
    omega_dot_m *= 1/wm.inertia_total
    wm.drum_radius/wm.gear_ratio * omega_dot_m
end

""" Calculate the tether force as function of the synchronous tether speed and the speed. """
function calc_force(wm::AsyncGenerator, set_speed, speed)
    acc = calc_acceleration(wm, set_speed, speed, 0.0)
    (wm.gear_ratio/wm.drum_radius) ^ 2 * wm.inertia_total * acc
end
