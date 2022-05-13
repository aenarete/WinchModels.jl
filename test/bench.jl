using Pkg
if ! ("BenchmarkTools" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using WinchModels, BenchmarkTools

wm = deepcopy(AsyncGenerator())
@benchmark calc_acceleration(wm, 7.9, 8., 100.0)

# mean time:          17 ns
# mean time Python: 1.05 µs