using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using ControlPlots

v_ro =     3.0 # m/s
β =       26.0 # degrees
K =      123.0 # N/(m/s)^2
period =  30.0 # seconds

function force(v_wind)
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

time = range(0, 120, length=120)
v_wind = 12.1 .+ 1.5*triangle_wave.(time, period)
f = force.(v_wind)

plot(time, f; xlabel="Time [s]", ylabel="Force")
