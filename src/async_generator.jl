#= MIT License

Copyright (c) 2022, 2024 Uwe Fechner

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

#= Dynamic winch model. Two models are supported:
- an asynchronous machine with a gearbox and a brake is used
- a torque controlled machine with a gearbox without brake is used
The inertia of drum and motor are combined into one value (stiff coupling). =#

"""
    mutable struct AsyncMachine

Model of a winch with an async generator and a gearbox.
"""
@with_kw mutable struct AsyncMachine <: AbstractWinchModel @deftype Float64
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
    brake::Bool = true;
    "last set speed"
    last_set_speed = 0.0;
end

function AsyncMachine(set::Settings)
    AsyncMachine(set=set)
end

# calculated the motor reactance X [Ohm]
function calc_reactance(wm::AsyncMachine)
    wm.u_nom^2 / (2 * wm.omega_sn * wm.tau_b) 
end

# calculate the motor inductance L [H]
function calc_inductance(wm::AsyncMachine)
    calc_reactance(wm) / wm.omega_sn
end

# calculate the motor resistance R2 [Ohm]
function calc_resistance(wm::AsyncMachine)
    (wm.u_nom^2 * (wm.omega_sn - wm.omega_mn))/(2wm.omega_sn^2) * (1 / wm.tau_n + sqrt(1 / wm.tau_n^2 - 1 / wm.tau_b^2))
end

# coulomb friction torque TAU_STATIC [Nm]
function calc_coulomb_friction(wm::AsyncMachine)
    wm.set.f_coulomb * wm.set.drum_radius / wm.set.gear_ratio
end

# viscous friction torque C_F [Nm]
# omega in rad/s
function calc_viscous_friction(wm::AsyncMachine, omega)
    wm.set.c_vf * omega * wm.set.drum_radius^2 / wm.set.gear_ratio^2     
end

# differentiable version of the sign function
function smooth_sign(x)
    EPSILON = 6
    x / sqrt(x * x + EPSILON * EPSILON)
end

function calc_acceleration(wm::AsyncMachine, set_speed, speed, force, use_brake = false)
    calc_acceleration(wm::AsyncMachine, speed, force; set_torque=nothing, set_speed, use_brake)
end
function calc_acceleration(wm::AsyncMachine, speed, force; set_torque=nothing, set_speed=nothing, use_brake = false)
    dt = 1/wm.set.sample_freq
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
    # # limit the acceleration
    MAX_ACC = 4.0
    limited_speed = set_speed
    if set_speed > wm.last_set_speed + MAX_ACC*dt  
        limited_speed = wm.last_set_speed + MAX_ACC*dt
    elseif set_speed < wm.last_set_speed - MAX_ACC*dt
        limited_speed = wm.last_set_speed - MAX_ACC*dt
    end
    wm.last_set_speed = limited_speed
    omega      = wm.set.gear_ratio/wm.set.drum_radius * speed
    omega_sync = wm.set.gear_ratio/wm.set.drum_radius * limited_speed
    delta = omega_sync - omega
    R2 = calc_resistance(wm)
    L  = calc_inductance(wm)
    if abs(omega_sync) <= wm.omega_sn
        omega_dot_m = (wm.u_nom^2 * R2 * delta) / (wm.omega_sn^2 * (R2^2 + L^2 * delta^2))
    else
        omega_dot_m = (wm.u_nom^2 * R2 * delta) / (omega_sync^2 * (R2^2 + L^2 * delta^2))
    end
    τ = calc_coulomb_friction(wm) * smooth_sign(omega) + calc_viscous_friction(wm, omega)
    omega_dot_m += wm.set.drum_radius / wm.set.gear_ratio * force * 4000.0 / wm.set.max_force - τ
    omega_dot_m *= 1/wm.set.inertia_total
    wm.set.drum_radius/wm.set.gear_ratio * omega_dot_m
end

""" Calculate the tether force as function of the synchronous tether speed and the speed. """
function calc_force(wm::AsyncMachine, set_speed, speed)
    acc = calc_acceleration(wm, set_speed, speed, 0.0)
    (wm.set.gear_ratio/wm.set.drum_radius) ^ 2 * wm.set.inertia_total * acc
end
