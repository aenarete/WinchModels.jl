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
@with_kw mutable struct AsyncGenerator <: AbstractWinchModel
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
    brake = true;
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

# TAU_STATIC = f_coulomb * drum_radius / gear_ratio    # Coulomb friction torque [Nm]
# C_F = c_vf * drum_radius^2 / gear_ratio^2     # Coefficient for the viscous friction torque [Nms/rad]

function smooth_sign(x)
    EPSILON = 6
    x / sqrt(x * x + EPSILON * EPSILON)
end

# def calcAcceleration(set_speed, speed, force, useBrake = False):
#     if useBrake:
#         global brake
#         if abs(set_speed) < 0.9 * v_min:
#             brake = True
#         if abs(set_speed) > 1.1 * v_min:
#             brake = False
#         if brake:
#             return brake_acc * speed # if the brake is active the acceleration proportional to the speed
#     omega      = gear_ratio/drum_radius * speed
#     omega_sync = gear_ratio/drum_radius * set_speed
#     delta = omega_sync - omega
#     if abs(omega_sync) <= omega_sn:
#         omega_dot_m = (u_nom**2 * R2 * delta) / (omega_sn**2 * (R2**2 + L**2 * delta**2))
#     else:
#         omega_dot_m = (u_nom**2 * R2 * delta) / (omega_sync**2 * (R2**2 + L**2 * delta**2))
#     omega_dot_m += drum_radius / gear_ratio * force * 4000.0 / max_winch_force - C_F * omega - TAU_STATIC * smoth_sign(omega)
#     omega_dot_m *= 1/inertia_total
#     return drum_radius/gear_ratio * omega_dot_m

# def calcForce(set_speed, speed):
#     """ Calculate the thether force as function of the synchronous tether speed and the speed. """
#     acc = calcAcceleration(set_speed, speed, 0.0)
#     return (gear_ratio/drum_radius) ** 2 * inertia_total * acc

# if __name__ == "__main__":
#     if True:
#         print "Inertia, as seen from the generator: ", inertia_total
#         print "C_F", C_F
#         print "TAU_STATIC", TAU_STATIC
#         print "R2, L", R2, L
#         print "v_SN", (omega_sn / (gear_ratio/drum_radius))
#     if False:
#         print 'Acceleration at 4 m/s due to friction: ', calcAcceleration(7.9, 8., 0.0)
#     if False:
#         from pylab import np, plot
#         n = 256
#         X = gear_ratio/drum_radius * np.linspace(-8.0, 8.0, n, endpoint=True)
#         SIGN = []
#         for i in range(n):
#             SIGN.append(smoth_sign(X[i]))
#         plot(X, SIGN, label='smoth_sign')
#     if False:
#         with Timer() as t0:
#             for i in range(10000):
#                 pass
#         with Timer() as t1:
#             for i in range(10000):
#                 calcAcceleration(7.9, 8., 100.0)
#         print "time for calcAcceleration  [Âµs]:   ", (t1.secs - t0.secs)  / 10000 * 1e6
#     if False:
#         from pylab import np, plot, xlim, ylim, legend, grid, gca
#         force = calcForce(4.0*1.025, 4.0)
#         print "Force: ", force
#         n = 256
#         F2, F4, F6, F7, F8 = [], [], [], [], []
#         V = np.linspace(0.0, 9.0, n, endpoint=True)
#         for i in range(n):
#             F2.append(-calcForce(0.15, V[i]))
#             F4.append(-calcForce(1.0, V[i]))
#             F6.append(-calcForce(6.0, V[i]))
# #            F7.append(-calcForce(-7.3, -V[i]))
#             F8.append(-calcForce(8.0, V[i]))
#         plot(V, F2, label=u'$v_s$ = 0.15 m/s')
#         plot(V, F4, label=u'$v_s$ = 1 m/s')
#         plot(V, F6, label=u'$v_s$ = 6 m/s')
# #        plot(V, F7, label=u'$v_s$ = 7.3 m/s')
#         plot(V, F8, label=u'$v_s$ = 8 m/s')
#         ylim(-15000.0, 15000)
#         #xlim(0.0, 9.0)
#         xlim(0.0, 0.4)
#         legend(loc='upper right')
#         gca().set_ylabel(u'Tether force [N]')
#         gca().set_xlabel(u'Reel-out speed [m/s]')
#         grid(True, color='0.25')
#     if False:
#         from pylab import np, plot
#         X = np.linspace(-307., 307., num = 1000)
#         Y = []
#         print "omega_max: ", gear_ratio/drum_radius * 8.
#         for x in X:
#             Y.append(smoth_sign(x))
#         plot(X, Y)
#     if True:
#         print "f_max: ", max_winch_force
