# Example of using speed control with a step input for the set_speed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using ControlPlots, WinchModels, KiteUtils

set::Settings = deepcopy(load_settings("system.yaml"))
set.sample_freq = 200.0

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

function calc_force(time, v_ro)
    v_wind = 12.1
    force(v_wind, v_ro)
end

function calc_set_speed(time, period=10.0)
    3.0 + 1.5 * (time%period > period/2)
end

function simulate(t_sim=60; f_0=7300, speed_0=3.176, dt=1/set.sample_freq)
    time = 0:dt:t_sim
    F = Float64[]
    ACC = Float64[]
    V_RO = Float64[]
    V_RO_SET = Float64[]
    f = f_0
    v_ro = speed_0
    wm = AsyncMachine(set)
    wm.last_set_speed = v_ro
    for t in time
        # calculate the set_speed
        set_speed = calc_set_speed(t)
        V_RO_SET = push!(V_RO_SET, set_speed)
        # calculate the acceleration
        acc = calc_acceleration(wm::AsyncMachine, v_ro, f; set_speed, use_brake = false)
        push!(ACC, acc)
        # integrate the acceleration to get the velocity
        v_ro += acc*dt
        push!(V_RO, v_ro)
        f = calc_force(t, v_ro)
        push!(F, f)
    end
    p1=plot(time, F; xlabel="Time [s]", ylabel="Force [N]", fig="force")
    p2=plot(time, [V_RO_SET, V_RO]; xlabel="Time [s]", ylabel="Speed [m/s]", labels=["set_speed", "speed"], fig="speed")
    p3=plot(time, ACC; xlabel="Time [s]", ylabel="Acceleration [m/s^2]", fig="acceleration")
    display(p1); display(p2); display(p3)
end

simulate()
