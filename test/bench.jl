using WinchModels, BenchmarkTools

wm = deepcopy(AsyncGenerator())
@benchmark calc_acceleration(wm, 7.9, 8., 100.0)

# mean time:         912 ns
# mean time Python: 1.05 µs