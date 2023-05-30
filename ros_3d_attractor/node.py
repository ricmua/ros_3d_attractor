""" A ROS2 module implementing a node that generates force commands 
    suitable for attracting a robotic effector to a point, line, or plane 
    embedded in a 3D space.

The attractor force field is modeled as a [spring+damper] system that pulls the 
device end effector towards its projection on a specified constraint point, 
line segment, or plane.

[spring+damper]: https://en.wikipedia.org/wiki/Mass-spring-damper_model
        
Examples
--------

Initialize a ROS2 interface.

>>> rclpy.init()

Create a convenence function for spinning ROS once.

>>> spin_once = lambda n: rclpy.spin_once(n, timeout_sec=0.001)

Create an attractor node.

>>> node = Node()
>>> node._sample_timer.cancel()

Set some non-trivial attractor parameters.

>>> from rclpy.parameter import Parameter
>>> basis = [0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 1.0]
>>> result = node.set_parameters([Parameter('basis', value=basis)])
>>> result[0].successful
True

Test the force calculations for the attractor at baseline.

>>> [*node.compute_attractor_force()]
[0.0, 0.0, 0.0]

Test the applied force calculation.

>>> [*node.compute_applied_force()]
[0.0, 0.0, 0.0]

Create a client node for interacting with the attractor node.

>>> client_node = rclpy.node.Node('client')

Initialize a subscription to receive the force output of the attractor node.

>>> force_callback = lambda m: print(f'Force: {(m.x, m.y, m.z)}')
>>> force_subscription \\
...   = client_node.create_subscription(force_message, 'force/output', 
...                                     force_callback, 2)

Initialize publishers for sending messages to the attractor node.

>>> position_publisher \\
...   = client_node.create_publisher(position_message, 'position', 2)
>>> velocity_publisher \\
...   = client_node.create_publisher(velocity_message, 'velocity', 2)
>>> force_publisher \\
...   = client_node.create_publisher(force_message, 'force/input', 2)

Test the full sampling and force publication mechanism, with the baseline 
conditions.

>>> node._sample_callback()
>>> spin_once(node)
>>> spin_once(client_node)
Force: (0.0, 0.0, 0.0)

Test the full sampling and force publication mechanism, after modifying the 
effector position.

>>> position_publisher.publish(position_message(x=1.0))
>>> spin_once(client_node)
>>> spin_once(node)
>>> node._current_position
(1.0, 0.0, 0.0)
>>> node._sample_callback()
>>> spin_once(node)
>>> spin_once(client_node)
Force: (-2000.0, 0.0, 0.0)
>>> node.get_parameter('stiffness').value
2000.0

Test the full sampling and force publication mechanism, after modifying the 
effector velocity.

>>> velocity_publisher.publish(velocity_message(x=0.2, y=0.1))
>>> spin_once(client_node)
>>> spin_once(node)
>>> node._current_velocity
(0.2, 0.1, 0.0)
>>> node._sample_callback()
>>> spin_once(node)
>>> spin_once(client_node)
Force: (-2002.0, 0.0, 0.0)
>>> node.get_parameter('damping').value
10.0


Clean up.

>>> client_node.destroy_node()
>>> node.destroy_node()
>>> rclpy.shutdown()

"""

# Copyright 2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import numpy.
import numpy
from numpy import linalg

# Import ROS2.
import rclpy
import rclpy.qos
import rclpy.node
#import rosidl_runtime_py
import rcl_interfaces.msg

# Local imports.
from .msg import position_message
from .msg import velocity_message
from .msg import force_message


# Declare default quality-of-service settings for ROS2.
DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
""" Default quality-of-service for ROS2 topics. """

# Declare the default transform for output force vectors.
DEFAULT_TRANSFORM = [+1.0, +0.0, +0.0,
                     +0.0, +1.0, +0.0,
                     +0.0, +0.0, +1.0]
""" Default transform for output force vectors, stored as row-major.

See the description of the 'force_transform` ROS2 parameter. 
Defaulting to the identity matrix results in no modification of the force. 
"""

# Declare the default attractor basis.
DEFAULT_BASIS = [+1.0, +0.0, +0.0,
                 +0.0, +1.0, +0.0,
                 +0.0, +0.0, +1.0]
""" Default 3D basis for the attractor, stored row-major.

See the description of the `basis` ROS2 parameter. 
Defaulting to the [standard basis] causes the attractor force to be zero 
(i.e., the robot effector moves freely).

[standard basis]: https://en.wikipedia.org/wiki/Standard_basis
"""


# ROS2 node.
class Node (rclpy.node.Node):
    """ A ROS2 node that generates force commands suitable for attracting a 
        robotic effector to a point, line, or plane embedded in a 3D space.
    """
    
    def __init__(self, *args, node_name='attractor', **kwargs):
        """ Constructor. """
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, **kwargs)
        
        # Initialize local variables.
        self._current_position    = (0.0, 0.0, 0.0)
        self._current_velocity    = (0.0, 0.0, 0.0)
        self._current_force_input = (0.0, 0.0, 0.0)
        
        # Initialize the node.
        self.initialize_parameters()
        self.initialize_publishers()
        self.initialize_subscriptions()
        self.initialize_timer()
        
    def initialize_parameters(self):
        """ Declare ROS2 parameters used by the node, and define the default 
            values.
        """
        
        # Define a force transform. 
        self.declare_parameter('force_transform', DEFAULT_TRANSFORM)
        
        # Define the attractor point, line, or plane. 
        # * For free movement, the basis will be the identity matrix.
        # * For attraction to a plane, one eigenvalue of the basis should be 
        #   less than one. 
        # * For attraction to a line, two eignevalues of the basis should be 
        #   less than one. 
        # * For attraction to a point, all eigenvalues should be less than one, 
        #   and the offset should be set to match the target point.
        self.declare_parameter('basis', DEFAULT_BASIS)
        self.declare_parameter('offset', [0.0, 0.0, 0.0])
        self.declare_parameters('weights', [1.0, 1.0, 1.0])
        
        # Define the parameters of the spring-damper system. Default values are 
        # taken from the `constraints` example provided in the Force Dimension 
        # SDK. There, the units are listed as N/m and N/(m/s), respectively. 
        self.declare_parameter('stiffness', 2000.0)
        self.declare_parameter('damping', 10.0)   
        
        # Define a sample interval in seconds. This determines the frequency 
        # at which the attractor publishes force messages. This frequency must 
        # be high enough for force updates to maintain stability.
        self.declare_parameter('sample_interval_s', 0.0005)
        
        # Add a parameter for enabling and disabling publication of applied 
        # forces.
        self.declare_parameter('publish_force', True)
        
        # Add a "on set" parameter callback.
        self.add_on_set_parameters_callback(self._parameters_callback)
        
    def initialize_publishers(self):
        """ Initialize a ROS2 publisher for force output messages. """
        
        # Initialize a publisher for robot force messages.
        kwargs = dict(topic='force/output',
                      msg_type=force_message,
                      qos_profile=DEFAULT_QOS)
        self._force_publisher = self.create_publisher(**kwargs)
        
    def initialize_subscriptions(self):
        """ Initialize ROS2 subscriptions for position, velocity, and force 
            input messages.
        """
        
        # Initialize a subscription for the effector position.
        kwargs = dict(topic='position',
                      msg_type=position_message,
                      callback=self._position_callback,
                      qos_profile=DEFAULT_QOS)
        self.create_subscription(**kwargs)
        
        # Initialize a subscription for the effector velocity.
        kwargs = dict(topic='velocity',
                      msg_type=velocity_message,
                      callback=self._velocity_callback,
                      qos_profile=DEFAULT_QOS)
        self.create_subscription(**kwargs)
        
        # Initialize a subscription for the input force.
        kwargs = dict(topic='force/input',
                      msg_type=force_message,
                      callback=self._force_callback,
                      qos_profile=DEFAULT_QOS)
        self.create_subscription(**kwargs)
        
    def initialize_timer(self):
        """ Initialize a timer for periodically publishing force messages. """
        
        # Initialize a sample timer.
        sample_interval_s = self.get_parameter('sample_interval_s').value
        self._sample_timer \
          = self.create_timer(timer_period_sec=sample_interval_s,
                              callback=self._sample_callback)
        
    def _parameters_callback(self, parameters):
        
        # Check to see if the sample interval has been updated.
        updated = any([(p.name == 'sample_interval_s') for p in parameters])
        
        # If necessary, re-set the sample timer.
        if updated:
            del self._sample_timer
            self.initialize_timer()
        
        # Result.
        reason = 'Sample interval successfully updated'
        reason = reason if updated else 'No relevant parameter updates'
        result = rcl_interfaces.msg.SetParametersResult(successful=True, 
                                                        reason=reason)
        
        # Return the result.
        return result
        
    def _position_callback(self, message):
        
        # Record the current position.
        self._current_position = (message.x, message.y, message.z)
        
    def _velocity_callback(self, message):
        
        # Record the current velocity.
        self._current_velocity = (message.x, message.y, message.z)
        
    def _force_callback(self, message):
        
        # Record the current force.
        self._current_force = (message.x, message.y, message.z)
        
    def _sample_callback(self):
        
        # Compute attractor force.
        f_a = self.compute_attractor_force()
        
        # Compute applied force.
        f = self.compute_applied_force(f_a)
        
        # Publish the applied force.
        self._publish_force(f)
        
    def compute_attractor_force(self, position=None, 
                                      velocity=None,
                                      basis=None,
                                      offset=None,
                                      weights=None,
                                      stiffness=None,
                                      damping=None):
        """ Compute the force due to the attractor, given the current state of 
            the system.
        """
        
        # Initialize a utility function for retrieving parameters.
        get_parameter = lambda n: self.get_parameter(n).value
        
        # Initialize local variables.
        K = stiffness if stiffness else get_parameter('stiffness')
        C = damping if damping else get_parameter('damping')
        o = offset if offset else get_parameter('offset')
        p = position if position else self._current_position
        v = velocity if velocity else self._current_velocity
        
        # Initialize a utility function for ensuring column vectors.
        to_column = lambda s: numpy.array(s)[:][numpy.newaxis].T
        
        # Convert to column vectors.
        o = to_column(o)
        p = to_column(p)
        v = to_column(v)
        
        # Create a projection matrix from the attractor basis parameter.
        # Weighted projection.
        b   = basis if basis else get_parameter('basis')
        B   = numpy.array(b).reshape((3, 3))
        #P_a = B @ linalg.pinv(B)
        w   = weights if weights else get_parameter('weights')
        Wr  = numpy.diag(numpy.sqrt(w))
        P_a = B @ Wr @ linalg.pinv(Wr @ B) @ (Wr @ Wr.T)
        
        # Compute the projection of the position vector onto the attractor 
        # surface -- that is, the permissable plane in which the effector 
        # should be constrained to move -- after subtracting the offset.
        p_a = P_a @ (p - o) + o
        
        # Compute the difference vector between the position vector and the 
        # offset plane.
        p_d = p_a - p
        
        # Compute the "guidance force" from the spring + damper model.
        # Project the current position onto the attractor surface (i.e., a line 
        # or plane).
        f_g = K * (p_a - p) - C * v
        
        # Project the guidance force onto the difference vector; that is, the 
        # tangent to the permissable plane.
        # This is recommended by the Force Dimension example, and seems 
        # necessary due to the velocity term in the spring + damper model, 
        # which can introduce viscous force components on the permissable 
        # attractor surface (i.e., the "free" plane).
        P_g = p_d @ linalg.pinv(p_d)
        f_g = P_g @ f_g
        
        # The result should be a numpy array with a single dimension.
        f_g = f_g.squeeze()
        
        # Check form of result.
        assert (len(f_g) == 3)
        assert (type(f_g) == numpy.ndarray)
        assert isinstance(f_g[0], float)
        
        # Return the result as a numpy array.
        return f_g
    
    def compute_applied_force(self, *forces):
        """ Sum and transform component forces that act on a robotic effector. 
        """
        
        # Initialize default values.
        forces = forces if forces else [(0.0, 0.0, 0.0)]
        
        # Initialize a utility function for ensuring column vectors.
        to_column = lambda s: numpy.array(s)[:][numpy.newaxis].T
        
        # Sum the component forces.
        f = sum([numpy.array(f) for f in forces])
        f = to_column(f)
        
        # Transform the aggregate force.
        t = numpy.array(self.get_parameter('force_transform').value)
        T = t.reshape((3, 3))
        f = T @ f
        
        # Return the aggregate applied force.
        return f.squeeze()
        
    def _publish_force(self, f):
        
        # Publish the force.
        if self.get_parameter('publish_force').value:
            kwargs = dict(zip(['x', 'y', 'z'], f))
            message = force_message(**kwargs)
            self._force_publisher.publish(message)
    
  

# Entry point.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

