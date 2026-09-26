using WinchModels, KiteUtils
using Test

if basename(pwd()) == "test"; cd(".."); end
KiteUtils.set_data_path("") 
set = se()

 @testset "calc_force" begin
    wm = deepcopy(AsyncMachine(set))
    wm.set.inertia_total=4*0.082
    @test wm.set.inertia_total ≈ 0.328
    @test (-calc_force(wm, 1.0, 0.0))  ≈ -7976.361025382839
    @test (-calc_force(wm, 0.15, 9.0)) ≈   2389.8735064202406
end

@testset "WinchModels.jl" begin
    wm = deepcopy(AsyncMachine(set))
    @test calc_reactance(wm)  ≈ 0.4676729273591048
    @test calc_inductance(wm) ≈ 0.002977298325578337
    @test calc_resistance(wm) ≈ 0.07268793534211404
    @test calc_coulomb_friction(wm) ≈ 3.1779032258064515
    omega = 1.5
    @test calc_viscous_friction(wm, omega) ≈ 0.03114399778876171
    set_speed = 50.0
    speed = 49.0
    force = 1000.0
    @test calc_acceleration(wm, speed, force; set_speed, use_brake = false) ≈ -3.274672680273824
    @test calc_force(wm, set_speed, speed) ≈ -1986.0407795157232
    set_speed = 0.11
    speed = 0.1
    @test calc_acceleration(wm, speed, force; set_speed, use_brake=true) ≈ -2.5
    
    set_speed = -0.11
    speed = -0.1
    @test calc_acceleration(wm, speed, force; set_speed, use_brake=true) ≈ 2.5
    # compare results with Python
    wm.set.inertia_total=4*0.082
    @test wm.set.inertia_total ≈ 0.328
    @test calc_viscous_friction(wm, 1.0) ≈ 0.0207626651925
    @test calc_coulomb_friction(wm) ≈ 3.17790322581
    @test calc_resistance(wm) ≈ 0.0726879353421
    @test calc_inductance(wm) ≈ 0.00297729832558
    @test (wm.omega_sn / (wm.set.gear_ratio/wm.set.drum_radius)) ≈ 4.09167107705
    @test_broken calc_acceleration(wm, 8, 0; set_speed=7.9) ≈ -3.13208622374
    @test_broken calc_acceleration(wm, 7.9, 8, 0) ≈ -3.13208622374
    @test_broken calc_force(wm, 4.0*1.025, 4.0) ≈ 4015.21454473
end

@testset "smooth_sign is odd, zero at zero and bounded by one" begin
    @test smooth_sign(0.0) == 0.0
    @test smooth_sign(-3.0) ≈ -smooth_sign(3.0)
    @test 0.99 < smooth_sign(1000.0) < 1.0
end

@testset "AsyncMachine brake keeps its state inside the hysteresis band" begin
    wm = AsyncMachine(Settings("system.yaml"))
    speed = 0.5
    braked = wm.brake_acc * speed
    @test calc_acceleration(wm, speed, 0.0; set_speed=wm.v_min, use_brake=true) ≈ braked
    calc_acceleration(wm, speed, 0.0; set_speed=2wm.v_min, use_brake=true)
    @test !wm.brake
    @test calc_acceleration(wm, speed, 0.0; set_speed=wm.v_min, use_brake=true) != braked
    @test !wm.brake
end

@testset "AsyncMachine limits the set speed rate to max_acc in both directions" begin
    wm = AsyncMachine(Settings("system.yaml"))
    step = wm.set.max_acc / wm.set.sample_freq
    calc_acceleration(wm, 0.0, 0.0; set_speed=10.0)
    @test wm.last_set_speed ≈ step
    wm.last_set_speed = 0.0
    calc_acceleration(wm, 0.0, 0.0; set_speed=-10.0)
    @test wm.last_set_speed ≈ -step
end

"""
    drive_acceleration(wm, speed, slip)

Acceleration of `wm` at `speed` [m/s] from a set speed `slip` [m/s] above it, minus the
acceleration at zero slip, with the rate limit bypassed.
"""
function drive_acceleration(wm, speed, slip)
    wm.last_set_speed = speed + slip
    with_slip = calc_acceleration(wm, speed, 0.0; set_speed=speed + slip)
    wm.last_set_speed = speed
    with_slip - calc_acceleration(wm, speed, 0.0; set_speed=speed)
end

@testset "AsyncMachine drive torque falls with 1/omega² above synchronous speed" begin
    wm = AsyncMachine(Settings("system.yaml"))
    gear = wm.set.gear_ratio / wm.set.drum_radius
    slip = 0.05
    rated = drive_acceleration(wm, 3.0, slip)
    @test gear * 3.0 < wm.omega_sn < gear * 5.0
    @test rated > 0
    field_weakening = wm.omega_sn^2 / (gear * (5.0 + slip))^2
    @test drive_acceleration(wm, 5.0, slip) / rated ≈ field_weakening
end

@testset "TorqueControlledMachine friction matches AsyncMachine" begin
    tm = TorqueControlledMachine(Settings("system.yaml"))
    am = AsyncMachine(Settings("system.yaml"))
    @test calc_coulomb_friction(tm) ≈ calc_coulomb_friction(am)
    @test calc_viscous_friction(tm, 1.5) ≈ calc_viscous_friction(am, 1.5)
end

@testset "TorqueControlledMachine turns set torque and tether force into acceleration" begin
    tm = TorqueControlledMachine(Settings("system.yaml"))
    ratio = tm.set.drum_radius / tm.set.gear_ratio
    inertia = tm.set.inertia_total
    set_torque = 10.0
    force = 500.0
    @test calc_acceleration(tm, 0.0, 0.0; set_torque) ≈ ratio * set_torque / inertia
    @test calc_acceleration(tm, 0.0, force; set_torque) -
          calc_acceleration(tm, 0.0, 0.0; set_torque) ≈ ratio^2 * force / inertia
    @test calc_acceleration(tm, 1.0, 0.0; set_torque=0.0) < 0
end

@testset "TorqueControlledMachine brake keeps its state inside the hysteresis band" begin
    tm = TorqueControlledMachine(Settings("system.yaml"))
    speed = 0.5
    braked = tm.brake_acc * speed
    @test calc_acceleration(tm, speed, 0.0; set_speed=0.0, use_brake=true) ≈ braked
    @test calc_acceleration(tm, speed, 0.0; set_speed=tm.v_min, use_brake=true) ≈ braked
    calc_acceleration(tm, speed, 0.0; set_speed=2tm.v_min, use_brake=true)
    @test !tm.brake
    calc_acceleration(tm, speed, 0.0; set_speed=tm.v_min, use_brake=true)
    @test !tm.brake
end

@testset "TorqueControlledMachine speed control accelerates towards the set speed" begin
    tm = TorqueControlledMachine(Settings("system.yaml"))
    step = tm.set.max_acc / tm.set.sample_freq
    @test calc_acceleration(tm, 0.0, 0.0; set_speed=10.0) > 0
    @test tm.last_set_speed ≈ step
    tm = TorqueControlledMachine(Settings("system.yaml"))
    @test calc_acceleration(tm, 0.0, 0.0; set_speed=-10.0) < 0
    @test tm.last_set_speed ≈ -step
    tm = TorqueControlledMachine(Settings("system.yaml"))
    @test calc_acceleration(tm, 0.0, 0.0; set_speed=step / 2) > 0
    @test tm.last_set_speed ≈ step / 2
end

@testset "calc_set_torque opposes the tether force when on the set speed" begin
    set = Settings("system.yaml")
    wcs = WinchModels.WinchSpeedController(; dt=1 / set.sample_freq)
    force = 1000.0
    torque = WinchModels.calc_set_torque(set, wcs, 2.0, 2.0, force)
    @test torque ≈ -set.drum_radius / set.gear_ratio * force
end
