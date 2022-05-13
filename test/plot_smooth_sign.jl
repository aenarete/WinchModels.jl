using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using Plots, WinchModels

wm = deepcopy(AsyncGenerator())
n = 256
x = wm.gear_ratio/wm.drum_radius * range(-8.0, 8.0, length=n)
s = smooth_sign.(x)
plot(x,s, legend=false)
savefig("doc/smooth_sign.png")