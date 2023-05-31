<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Relevant Force Dimension material

This attractor was designed to be used specifically with the 
[Force Dimension ROS2 package]. See the documentation for that package for 
additional information. The [Force Dimension SDK] provides several code 
examples (C++) that are relevant to this package.

### segment.cpp

This command line example illustrates line segment constraints. The 
coefficients of the spring-damper equation are initialized as fixed constants: 

```c++
const double K = 2000.0;
const double C = 20.0;
```

Velocity thresholding is disabled:

```c++
// enable force and button emulation, disable velocity threshold
dhdEnableExpertMode ();
dhdSetVelocityThreshold (0);
```

A line segment is defined in 3D space by selecting two endpoints with the robot 
end effector. At each subsequent update, the program computes the [projection] 
of the end effector position onto this line segment:

```c++
// compute projection of the device position onto segment
project_point_on_segment (p, point[0], point[1], s);
```

An attraction force is then computed by treating the difference between the 
effector endpoint and its orthogonal projection as a [spring+damper] system:

```c++
// compute guidance force, modeled as a spring+damper system that pulls
// the device towards its projection on the constraint segment
for (int i=0; i<3; i++) g[i] = K * (s[i] - p[i]) - C * v[i];
```

It is also useful to project the resultant force onto the same difference 
vector, since the endpoint velocity was not previously constrained:

```c++
// project guidance force onto the vector defined by the device position and 
// its projection; this removes all unwanted force components (e.g. viscosity 
// along the "free" direction)
project_force_on_direction (g, p, s, f);
```

### constraints.cpp

This GUI example applies several different spring-plus-damper constraints to 
the robotic end effector. The stiffness and _viscosity_ parameters are defined 
as follows: 

```c++
// define stiffness and viscosity used to constrain the device position
const double Kp = 2000.0;  // N/m
const double Kv =   10.0;  // N/(m/s)
```

Perhaps the most relevant force applied is the "hold" constraint. The force to 
hold the device in a particular position is implemented as follows:

```c++
cVector3d force = -Kp * (DevicePos - HoldPos) - Kv * DeviceLinVel;
```

The "workspace" constraint is implemented in a similar fashion, but involves a 
projection: 

```c++
// compute surface normal vector
cVector3d n = -cNormalize(DevicePos);

// compute reaction force
cVector3d force = Kp * depth * n;

// compute damping term parallel to normal
cVector3d damping = -Kv * cProject(DeviceLinVel, n);

// add all forces together
deviceForce = deviceForce + (force + damping);
```

### center.cpp

This command line example centers the robotic end effector, on command. The 
source code defines a refresh interval of 100ms, as well as the following 
physical constants:

```c++
#define KP    100.0
#define KVP    10.0
#define MAXF    4.0
```

The centering force is proportional to the position, relative to the origin:

```c++
// compute base centering force
force = - KP * position;
```

A constant force ceiling enforces an upper limit on the applied force:

```c++
// scale force to a pre-defined ceiling
if ((normf = force.norm()) > MAXF) force *= MAXF/normf;
```

Damping force is proportional to the velocity of the end effector:

```c++
// add damping
force  -= KVP * velpos;
```


<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Force Dimension ROS2 package]: https://github.com/ricmua/ros_force_dimension

[Force Dimension SDK]: https://www.forcedimension.com/software/sdk

[projection]: https://en.wikipedia.org/wiki/Projection_(linear_algebra)

[spring+damper]: https://en.wikipedia.org/wiki/Mass-spring-damper_model

