# Calls the Matlab implementation of the winch model (async_machine_step.m) through the Matlab
# engine. One engine call per time step: the inputs are written into the command as text, and
# the outputs are read back as one vector. mxcall and the exception check of eval_string would
# need several engine calls, each costing milliseconds.
module WinchModelsMATLABExt

using WinchModels, MATLAB
import WinchModels: matlab_step!

# the MatlabMachine whose parameters Matlab holds; async_machine_step keeps one set of
# parameters only
const ACTIVE = Ref{Any}(nothing)

# shortest representation that round-trips; Matlab also understands NaN, Inf and -Inf
mat_num(x::Bool) = x ? "1" : "0"
mat_num(x::Real) = repr(Float64(x))

function matlab_init!(wm::MatlabMachine, session)
    eval_string(session, "if exist('async_machine_step', 'file') ~= 2, " *
                "error('WinchModels:path', 'async_machine_step.m is not on the Matlab path'); end")
    set = wm.set
    # Float64 for all values: integers would become integer types in Matlab
    p = Dict{String, Float64}(
        "gear_ratio" => set.gear_ratio, "drum_radius" => set.drum_radius,
        "inertia_total" => set.inertia_total, "max_force" => set.max_force,
        "f_coulomb" => set.f_coulomb, "c_vf" => set.c_vf, "max_acc" => set.max_acc,
        "sample_freq" => set.sample_freq,
        "u_nom" => wm.u_nom, "omega_sn" => wm.omega_sn, "omega_mn" => wm.omega_mn,
        "tau_n" => wm.tau_n, "tau_b" => wm.tau_b, "v_min" => wm.v_min,
        "brake_acc" => wm.brake_acc)
    put_variable(session, :wm_p, mxarray(p))
    eval_string(session, "async_machine_step(wm_p); clear wm_p;")
    wm.initialized = true
    ACTIVE[] = wm
    nothing
end

function matlab_step!(wm::MatlabMachine, set_speed, speed, force, use_brake)
    session = get_default_msession()
    if !wm.initialized || ACTIVE[] !== wm
        matlab_init!(wm, session)
    end
    x = (speed, force, something(set_speed, NaN), use_brake, wm.brake, wm.last_set_speed)
    cmd = "try, wm_y = async_machine_step([" * join(mat_num.(x), " ") * "]); " *
          "catch wm_e, wm_y = []; wm_err = wm_e.message; end"
    check = MATLAB.has_exception_check_enabled(session)
    MATLAB.disable_exception_check!(session)
    try
        eval_string(session, cmd)
    finally
        check && MATLAB.enable_exception_check!(session)
    end
    y = vec(jarray(get_mvariable(session, :wm_y)))
    if length(y) != 5
        msg = jstring(get_mvariable(session, :wm_err))
        error("async_machine_step failed: $msg")
    end
    wm.speed0  = speed
    wm.force0  = force
    wm.acc0    = y[1]
    wm.dacc_dv = y[2]
    wm.dacc_dF = y[3]
    wm.brake   = y[4] != 0
    wm.last_set_speed = y[5]
    nothing
end

end
