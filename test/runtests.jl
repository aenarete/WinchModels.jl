using WinchModels
using Test

cd("..")
KiteUtils.set_data_path("") 

@testset "WinchModels.jl" begin
    wm = AsyncGenerator()
    @test calc_reactance(wm)  ≈ 0.4676729273591048
    @test calc_inductance(wm) ≈ 0.002977298325578337
    @test calc_resistance(wm) ≈ 0.07268793534211404
    @test calc_coulomb_friction(wm) ≈ 3.1779032258064515
    omega = 1.5
    @test calc_viscous_friction(wm, omega) ≈ 0.03114399778876171
    set_speed = 50.0
    speed = 49.0
    force = 1000.0
    @test calc_acceleration(wm, set_speed, speed, force, false) ≈ -1.7857125353931111
    set_speed = 0.11
    speed = 0.1
    @test calc_acceleration(wm, set_speed, speed, force, true) ≈ -2.5
    set_speed = -0.11
    speed = -0.1
    @test calc_acceleration(wm, set_speed, speed, force, true) ≈ 2.5    
end
