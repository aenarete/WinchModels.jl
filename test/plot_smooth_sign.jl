using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using MakieControlPlots, WinchModels, KiteUtils

wm = AsyncMachine(se())
n = 256
x = wm.set.gear_ratio/wm.set.drum_radius * range(-8.0, 8.0, length=n)
s = smooth_sign.(x)
plot(x, s; disp=true)
savefig("docs/smooth_sign.png")
