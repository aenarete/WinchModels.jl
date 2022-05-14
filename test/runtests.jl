using WinchModels, KiteUtils
using Test

cd("..")
KiteUtils.set_data_path("") 

 @testset "calc_force" begin
    wm = deepcopy(AsyncMachine())
    wm.inertia_total=4*wm.inertia_motor
    @test wm.inertia_total ≈ 0.328
    @test (-calc_force(wm, 1.0, 0.0))  ≈ -12620.5127746
    @test (-calc_force(wm, 0.15, 9.0)) ≈   3698.78395182
end

@testset "WinchModels.jl" begin
    wm = deepcopy(AsyncMachine())
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
    @test calc_force(wm, set_speed, speed) ≈ -1536.8834794369461
    set_speed = 0.11
    speed = 0.1
    @test calc_acceleration(wm, set_speed, speed, force, true) ≈ -2.5
    set_speed = -0.11
    speed = -0.1
    @test calc_acceleration(wm, set_speed, speed, force, true) ≈ 2.5
    # compare results with Python
    wm.inertia_total=4*wm.inertia_motor
    @test wm.inertia_total ≈ 0.328
    @test calc_viscous_friction(wm, 1.0) ≈ 0.0207626651925
    @test calc_coulomb_friction(wm) ≈ 3.17790322581
    @test calc_resistance(wm) ≈ 0.0726879353421
    @test calc_inductance(wm) ≈ 0.00297729832558
    @test (wm.omega_sn / (wm.gear_ratio/wm.drum_radius)) ≈ 4.09167107705
    @test calc_acceleration(wm, 7.9, 8, 0) ≈ -3.13208622374
    @test calc_force(wm, 4.0*1.025, 4.0) ≈ 4015.21454473
end
