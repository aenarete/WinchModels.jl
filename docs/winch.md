## Winch model

First, we calculate the acceleration of the tether at the ground station. It can be calculated as

$$a_\mathrm{t,o} = \frac{1}{I_w} \frac{r}{n} (\tau_\mathrm{g} + \tau_\mathrm{d} - \tau_\mathrm{f})$$

where $I_w$ is the winch inertia as seen from the generator, $r$ the drum radius, $n$ the gearbox ratio, $\tau_g$ the generator torque, $\tau_d$ the torque exerted by the drum on the generator and $\tau_f$ the friction torque.

The torque exerted by the drum depends on the tether force $F$ as follows:

$$ \tau_\mathrm{d} = \frac{r}{n}~F$$

The friction is modelled as the combination of a viscous friction component with the friction coefficient $c_f$ and the static friction $\tau_s$

$$ \tau_f = c_\mathrm{f}~v_\mathrm{t,o} + \tau_s~\mathrm{sign}(v_\mathrm{t,o})$$

### Torque controlled winch
If the winch uses a generator with Direct Torque Control (DTC) it is possible to calculate $a_{t,o}$ as function of $\tau_g$, $F$ and $v_{t,o}$.

### Test case

We assume an ideal kite that pulls with the force:

$$ F=(v_\mathrm{w}~\mathrm{cos}~\beta)^2 K$$

with $K=328~Ns/m$ and the elevation angle $\beta = 26^o$.

Furthermore we assume the following wind speed:
<p align="center"><img src="https://raw.githubusercontent.com/aenarete/WinchModels.jl/torque/docs/wind-speed.png" width="500" /></p>

This results in the following tether force:
<p align="center"><img src="https://raw.githubusercontent.com/aenarete/WinchModels.jl/torque/docs/force.png" width="500" /></p>

and the following reel-out speed:
<p align="center"><img src="https://raw.githubusercontent.com/aenarete/WinchModels.jl/torque/docs/reelout-speed.png" width="500" /></p>

To execute the test script, run:
```julia
include("examples/torque_control.jl")
```

The settings of the winch model are defined in `settings.yaml` as follows:
```yaml
winch:
    winch_model: "AsyncMachine" # or TorqueControlledMachine
    max_force: 4000        # maximal (nominal) tether force; short overload allowed [N]
    v_ro_max:  8.0         # maximal reel-out speed                          [m/s]
    v_ro_min: -8.0         # minimal reel-out speed (=max reel-in speed)     [m/s]
    drum_radius: 0.1615    # radius of the drum                              [m]
    gear_ratio: 6.2        # gear ratio of the winch                         [-]   
    inertia_total: 0.204   # total inertia, as seen from the motor/generator [kgm²]
    f_coulomb: 122.0       # coulomb friction                                [N]
    c_vf: 30.6             # coefficient for the viscous friction            [Ns/m]
```