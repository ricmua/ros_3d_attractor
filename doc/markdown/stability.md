<!-- License

Copyright 2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Dynamic stability of the system

The attractor can interfere with the smooth operation of the robot by 
introducing [instability] and/or [oscillatory behavior]. Kinematic and sampling 
parameters must therefore be chosen carefully. See 
[configuration.md](configuration.md) for more information about specifying 
parameter values.

### Kinematic parameters

If the system is not adequately damped, then the attractor will cause 
oscillations. Values of the [damping ratio] near 1 are recommended, to yield a 
critically-damped system. For a system with [stiffness coefficient] $K$ and 
mass $m$, the critical damping coefficient can be chosen according to the 
equation:

$$C = 2 \sqrt{K m}$$

For example, for a Novint Falcon robot with effector mass $m = 0.190$, a 
stiffness value of $K = 2000$ could be paired with damping of $C = 39$.

### Sampling parameters

The system can be considered to be a [discretized] [linear attractor] or 
[linear control system], and the choice of parameters for the system is 
important for the time-varying dynamics. In particular, the choice of sampling 
rate must be high enough to ensure [stability of digital control]. The 
recommended update rate is 2000 Hz. See the 
[Force Dimension package documentation] for more information.

Importantly, the sampling interval for this package is not the only factor that 
must be considered. The position and velocity state input must also arrive at a 
rate high enough to ensure stable control. The nodes sending this input must 
publish at a high rate (e.g., 2000 Hz).

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[stability of digital control]: https://en.wikipedia.org/wiki/Digital_control#Stability

[linear attractor]: https://en.wikipedia.org/wiki/Attractor#Linear_equation_or_system


[damping ratio]: https://en.wikipedia.org/wiki/Damping#Damping_ratio_definition

[instability]: https://en.wikipedia.org/wiki/Instability#Instability_in_control_systems

[damping]: https://en.wikipedia.org/wiki/Damping

[oscillatory behavior]: https://en.wikipedia.org/wiki/Damping#Oscillation_cases

[linear control system]: https://en.wikipedia.org/wiki/Linear_control

[discretized]: https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models

[Force Dimension package documentation]: https://github.com/ricmua/ros_force_dimension/blob/main/doc/markdown/parameters.md#sampling-interval

[stiffness coefficient]: https://en.wikipedia.org/wiki/Spring_(device)#Physics

