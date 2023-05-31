<!-- License

Copyright 2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Model system

The model system implements a [force field] that functions like an [attractor], 
and is modeled as a [spring+damper] system. 

### Region of attraction

At each sample time point, the attraction force is based principally on the 
distance from the 3D position of the robotic end effector to some [projection] 
of that position on to a point, line, or plane embedded in 3D Euclidean space. 
The details of the projection are configured via basis, weight, and offset 
parameters. Together, these define what is here referred to as the 
_region of attraction_.

The [basis] is a 3 by 3 matrix that defines the _subspace of attraction_. The 
basis influences the region of attraction as follows:

* For free movement, the basis will be the identity matrix, or any 
  [full rank](https://en.wikipedia.org/wiki/Rank_(linear_algebra)) 3x3 matrix.
* For attraction to a plane, the basis must be rank 2.
* For attraction to a line, the basis should be rank 1.
* For attraction to a point, the basis should be the zero matrix (and the 
  offset should be set to match the target point).

The offset is a [location parameter] that can be used to shift the region of 
attraction away from the origin.

The weights <!-- are [scale parameter]s that --> can be used to control the 
relative strength of the attraction force along each dimension.

Let $\textbf{B} \in \mathcal{R}^{3 \times 3}$ represent a matrix with columns 
consisting of the basis set, 
$\boldsymbol{\mu} \in \mathcal{R}^{3}$ represent the offset, and 
$\textbf{w} \in \mathcal{R}^{3}$ represent the vector of weights. If 
$\textbf{W}$ is the matrix with diagonals equal to the weights $\textbf{w}$, 
and zeroes off the diagonal, then a weighted (as in [weighted least squares]) 
projection matrix is then defined by: 

$$\textbf{P}_a = \textbf{B} \left(\textbf{B}^T \textbf{W} \textbf{B} \right)^{-1} \textbf{B}^T \textbf{W}$$

and the projection of the robotic endpoint effector position $\textbf{p}$ onto 
the subspace of attraction is: 

$$\textbf{p}_a = \textbf{P}_a \left( \textbf{p} - \boldsymbol{\mu} \right) + \boldsymbol{\mu}$$

In the source code, the [projection matrix calculation] and the 
[projection calculation] are implemented in the `compute_attractor_force` 
method of the [`ros_3d_attractor.node`](../../ros_3d_attractor/node.py) module.

#### Multivariate Gaussian formulation

The attractor can also be considered in terms of [Gaussian inference]. In these 
terms, the basis can be represented by the eigenvectors of a covariance matrix, 
the weights can be represented by the eigenvalues, and the offset can be 
represented by the mean. In this scenario, the attractor acts like a prior 
probability distribution for the position of the robot endpoint.

### System equations

The force applied to the robotic end effector is calculated using the standard 
equations for a [spring+damper] system. The [stiffness] of the system is 
determined by a parameter [K], the _stiffness coefficient_. The [damping] is 
determined by a parameter [C], the _damping coefficient_. See 
[configuration.md](configuration.md) for more information about these 
parameters. 

The system can be considered to be a [linear attractor], and choice of 
parameters is important for dynamic [stability](stability.md).

The force equation is a sum of two terms. The position-dependent term applies 
an elastic force according to [Hooke's law], pulling the robot toward the 
region of attraction. The [damper] applies a viscous force to avoid 
over-corrections, and to maintain stability. The attractor force equation is: 

$$\textbf{f}_a = K \cdot \left( \textbf{p}_a -\textbf{p} \right) - C \cdot \textbf{v}$$

In the source code, the [force calculation] is implemented in the 
`compute_attractor_force` method of the 
[`ros_3d_attractor.node`](../../ros_3d_attractor/node.py) module.

### Directional viscosity

To remove any viscous component that does not act in the direction orthogonal 
to the subspace of attraction, the applied force is projected onto the normal 
vector. This is necessary because the velocity component was not projected onto 
the subspace prior to use in the system equations. [^implementation_note] This 
operation is a standard [projection] onto a line:
        
```math
\textbf{f} = \frac{\textbf{p}_d \textbf{p}_{d}^T}{\textbf{p}_{d}^T \textbf{p}_d} \textbf{f}_a
```

where $\textbf{p}_d = \textbf{p}_a -\textbf{p}$ is the positional difference 
vector.

[^implementation_note]: This was modeled after the `segment.cpp` 
                        [example from Force Dimension](force_dimension.md).

This operation potentially also improves numerical stability by removing 
unnecessary components.

In the source code, the [final projection calculation] is implemented in the 
`compute_attractor_force` method of the 
[`ros_3d_attractor.node`](../../ros_3d_attractor/node.py) module.


<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[projection]: https://en.wikipedia.org/wiki/Projection_(linear_algebra)

[spring+damper]: https://en.wikipedia.org/wiki/Mass-spring-damper_model

[K]: https://en.wikipedia.org/wiki/Spring_(device)#Physics
[stiffness]: https://en.wikipedia.org/wiki/Stiffness
[damper]: https://en.wikipedia.org/wiki/Dashpot
[damping]: https://en.wikipedia.org/wiki/Damping

[attractor]: https://en.wikipedia.org/wiki/Attractor

[force field]: https://en.wikipedia.org/wiki/Force_field_(physics)

[`compute_effector_force`]: https://github.com/ricmua/ros_nml_transforms/blob/c211c19db66085b9754429231457cb978cd66e89/ros_nml_transforms/attractor_node.py#L76

[linear attractor]: https://en.wikipedia.org/wiki/Attractor#Linear_equation_or_system


[basis]: https://en.wikipedia.org/wiki/Basis_(linear_algebra)

[Gaussian inference]: https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Conditional_distributions

[scale parameter]: https://en.wikipedia.org/wiki/Scale_parameter

[location parameter]: https://en.wikipedia.org/wiki/Location_parameter

[projection calculation]: https://github.com/ricmua/ros_3d_attractor/blob/c07da8a5664ee7bb6fe47a40e1d57f39ed1c8ec5/ros_3d_attractor/node.py#L354

[projection matrix calculation]: https://github.com/ricmua/ros_3d_attractor/blob/c07da8a5664ee7bb6fe47a40e1d57f39ed1c8ec5/ros_3d_attractor/node.py#L345

[force calculation]: https://github.com/ricmua/ros_3d_attractor/blob/c07da8a5664ee7bb6fe47a40e1d57f39ed1c8ec5/ros_3d_attractor/node.py#L363

[final projection calculation]: https://github.com/ricmua/ros_3d_attractor/blob/c07da8a5664ee7bb6fe47a40e1d57f39ed1c8ec5/ros_3d_attractor/node.py#L368

[weighted least squares]: https://en.wikipedia.org/wiki/Weighted_least_squares

[Hooke's law]: https://en.wikipedia.org/wiki/Spring_(device)#Hooke's_law

