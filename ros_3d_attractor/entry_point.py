""" Entry point for a ROS2, node-based application. """

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import ROS.
import rclpy

# Local imports.
from . import Node


# Main function.
def main(Node=Node, args=None):
    """ Default ROS2 entry point. """
    
    # Initialize ROS.
    rclpy.init()
    
    # Run a node by passing control to ROS.
    try:
        
        # Initialize the node.
        node = Node()
        
        # Spin the node.
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        finally: node.destroy_node()
        
    # Shut ROS down.
    finally: rclpy.shutdown()
    
    # Return.
    return
  

# __main__
if __name__ == '__main__': main()

