using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using Plots, WinchModels, LaTeXStrings

wm = deepcopy(AsyncGenerator())
wm.inertia_total=4*wm.inertia_motor
n = 256
x = range(0, 9.0, length=n)
f1, f2, f3, f4 = zeros(n), zeros(n), zeros(n), zeros(n)
for i in 1:n
   global f1, f2, f3, f4
   f1[i] = -calc_force(wm, 0.15, x[i])
   f2[i] = -calc_force(wm, 1.0, x[i])
   f3[i] = -calc_force(wm, 6.0, x[i])
   f4[i] = -calc_force(wm, 8.0, x[i])
end
plot(x, f1*0.001, yaxis="Tether force [kN]", xaxis="Reel-out speed [m/s]", label=L"$v_s$ = 0.15 m/s", 
    linewidth=2.0, legend = :bottomright)
plot!(x, f2*0.001, linewidth=2.0, label=L"$v_s$ = 1 m/s")
plot!(x, f3*0.001, linewidth=2.0, label=L"$v_s$ = 6 m/s")
plot!(x, f4*0.001, linewidth=2.0, label=L"$v_s$ = 8 m/s")
savefig("docs/tether_force.png")
