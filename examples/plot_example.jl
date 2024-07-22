using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using ControlPlots

v_wind = 3.0 # m/s
β =     26.0 # degrees
K =    971.0 # 
