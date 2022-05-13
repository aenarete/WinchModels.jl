# using TestEnv; TestEnv.activate()

using Plots, WinchModels

wm = deepcopy(AsyncGenerator())
n = 256
x = wm.gear_ratio/wm.drum_radius * range(-8.0, 8.0, length=n)
s = smooth_sign.(x)
plot(x,s)