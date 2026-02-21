using Pkg
if ! ("BenchmarkTools" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using WinchModels, BenchmarkTools, KiteUtils

wm = AsyncMachine(se())
@benchmark calc_acceleration(wm, 7.9, 8., 100.0)

# mean time:          17 ns
# mean time Python: 1.05 µs