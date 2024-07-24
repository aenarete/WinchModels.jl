using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using ControlPlots, WinchModels

v_ro =     3.0 # m/s
β =       26.0 # degrees
K =      123.0 # N/(m/s)^2
period =  30.0 # seconds

function force(v_wind, v_ro)
    v_eff = v_wind*cosd(β) - v_ro
    if v_eff < 0
        return 0
    end
    K * (v_eff)^2
end

# implementation of a triangle wave generator
function triangle_wave(t, period)
    t = mod(t, period)
    if t < period/2
        return 2*t/period
    else
        return 2*(period-t)/period
    end
end

function calc_force1(time, v_ro; period=30.0)
    v_wind = 12.1 + 1.5*triangle_wave(time, period)
    force(v_wind, v_ro), v_wind
end

function simulate(wm, t_sim=120; f_0=7900, speed_0=2.87, dt=0.005)
    time = 0:dt:t_sim
    F = Float64[]
    ACC = Float64[]
    V_RO = Float64[]
    V_WIND = Float64[]
    f = f_0
    v_ro = speed_0
    set_force = 7663
    n = wm.gear_ratio
    radius = wm.drum_radius
    set_torque = -set_force/n*radius
    for t in time
        # calculate the acceleration
        acc = calc_acceleration(wm, v_ro, f; set_torque, use_brake = false)
        push!(ACC, acc)
        # integrate the acceleration to get the velocity
        v_ro += acc*dt
        push!(V_RO, v_ro)
        # calculate the force using a triangle wind speed
        f, v_wind = calc_force1(t, v_ro)
        push!(V_WIND, v_wind)
        push!(F, f)
    end
    p1=plot(time, F; xlabel="Time [s]", ylabel="Force [N]", fig="force")
    p2=plot(time, V_RO; xlabel="Time [s]", ylabel="v_ro [m/s]", fig="reelout-speed")
    p3=plot(time, ACC; xlabel="Time [s]", ylabel="Acceleration [m/s^2]", fig="acceleration")
    p4=plot(time, V_WIND; xlabel="Time [s]", ylabel="Wind speed [m/s]", fig="wind-speed")
    display(p1); display(p2); display(p3); display(p4)
    nothing
end

wm = TorqueControlledMachine()
simulate(wm)