<!-- License

Copyright 2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Configuration

The attractor is configured via [ROS2 parameters]. These parameters are 
declared in the [`initialize_parameters`] method of `ros_3d_attractor.node`. A 
[sample configuration file](config/novint_falcon.yaml) is provided for the 
Novint Falcon robot.

### `basis`

This is a matrix that determines the region of attraction that the robot is 
drawn to. It is a subspace basis relative to the robot coordinate system. See 
the [region of attraction section](spring_damper.md#region_of_attraction) of 
the model system documentation.

The parameter is stored as a vector in [row-major order], which is is 
[reshaped] to matrix form (see the [reshaping code]). For example, the matrix 

$$
\begin{bmatrix}
  1 & 2 & 3 \\
  4 & 5 & 6 \\
  7 & 8 & 9 
\end{bmatrix}
$$

would be stored as:

```yaml
/robot/transforms:
  ros__parameters:
    effector:
      basis:
      - 1.00
      - 2.00
      - 3.00
      - 4.00
      - 5.00
      - 6.00
      - 7.00
      - 8.00
      - 9.00
```

The default value is the identity matrix.

See the [system model documentation](spring_damper.md) for more information.

### `offset`

A vector that determines how the region of attraction is offset from the 
origin. The default value is the origin.

See the [system model documentation](spring_damper.md) for more information.

### `weights`

A vector that determines the weights to apply to the different dimensions of 
the basis for the region of attraction. These are comparable to the eigenvalues 
of a covariance matrix. The default value is a vector of ones.

See the [system model documentation](spring_damper.md) for more information.

### `stiffness`

The stiffness coefficient used in determining the elastic force that guides the 
robot endpoint toward the region of attraction. The default value is `2000.0`.

See the [system model documentation](spring_damper.md) for more information.

### `damping`

The damping coefficient used in determinging the viscous force that dampens the 
attraction. The default value is `10.0`.

See the [system model documentation](spring_damper.md) for more information.

### `sample_interval_s`

The interval at which force messages are published by the attractor. The 
default value is `0.0005`, which is equivalent to 2000 Hz.

See the [notes about stability](stability.md) for further information.

### `publish_force`

Attractive force messages are published only if the value of this parameter is 
true.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[attractor]: https://en.wikipedia.org/wiki/Attractor

[force field]: https://en.wikipedia.org/wiki/Force_field_(physics)

[`compute_effector_force`]: https://github.com/ricmua/ros_nml_transforms/blob/c211c19db66085b9754429231457cb978cd66e89/ros_nml_transforms/node.py#L76

[linear attractor]: https://en.wikipedia.org/wiki/Attractor#Linear_equation_or_system


[row-major order]: https://en.wikipedia.org/wiki/Row-_and_column-major_order

[reshaped]: https://numpy.org/doc/stable/reference/generated/numpy.reshape.html

[reshaping code]: https://github.com/ricmua/ros_3d_attractor/blob/c07da8a5664ee7bb6fe47a40e1d57f39ed1c8ec5/ros_3d_attractor/node.py#L348

