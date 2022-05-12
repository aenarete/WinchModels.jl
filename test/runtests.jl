using WinchModels
using Test

cd("..")

@testset "WinchModels.jl" begin
    wm = AsyncGenerator()
    @test calc_reactance(wm)  ≈ 0.4676729273591048
    @test calc_inductance(wm) ≈ 0.002977298325578337
    @test calc_resistance(wm) ≈ 0.07268793534211404
end
