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

function calc_force(time, v_ro; period=30.0)
    v_wind = 12.1 + 1.5*triangle_wave(time, period)
    force(v_wind, v_ro)
end

function simulate(t_sim=120; f_0=7663, dt=0.05)
    time = 0:dt:t_sim
    F = Float64[]
    f = f_0
    wm=AsyncMachine()
    for t in time
        v_ro = 3.0
        # calculate the set_speed using a ramp
        # calculate the acceleration
        # calc_acceleration(wm::AsyncMachine, speed, f; set_speed=nothing, use_brake = false)
        # integrate the acceleration to get the velocity

        f = calc_force(t, v_ro)
        push!(F, f)
    end
    plot(time, F; xlabel="Time [s]", ylabel="Force [N]")
end

simulate()
