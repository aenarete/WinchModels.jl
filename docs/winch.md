## Winch model

First, we calculate the acceleration of the tether at the ground station. It can be calculated as
$$ a_{t,o} = \frac{1}{I_w} \frac{r}{n} (\tau_g + \tau_d - \tau_f) $$
where $I_w$ is the winch inertia as seen from the generator, $r$ the drum radius $n$ the gearbox ratio, $\tau_g$ the generator torque, $\tau_d$ the torque exerted by the drum on the generator and $\tau_f$ the friction torque.

The torque exerted by the drum depends on the tether force $F$ as follows:
$$ \tau_d = \frac{r}{n}~F$$

The friction is modelled as the combination of a viscous friction component with the friction coefficient $c_f$ and the static friction $\tau_s$
$$ \tau_f = c_f v_{t,o} + \tau_s sign(v_{t,o})$$

### Torque controlled winch
If the winch uses a generator with Direct Torque Control (DTC) it is possible to calculate $a_{t,o}$ as function of $\tau_g$, $F$ and $v_{t,o}$.