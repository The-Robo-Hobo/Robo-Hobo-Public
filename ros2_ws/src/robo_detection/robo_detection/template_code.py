#1/usr/bin/env python3
import rclpy                                            # required for every node
from rclpy.node import Node                             # for node creation using OOP

class Template(Node):                                 # defined class named Template which inherits Node from rclpy
    """A ros2 node as a template file."""

    def __init__(self):                                 # a constructor, required as the first function in every class
        super().__init__("template_node")                 # template_node is the name of the node
        self.get_logger().info("template_node is running")        # way to write a log

        # variables
        timer_period = 1.0  #seconds
        
        self.create_timer(timer_period, self.timer_callback)     # will call timer_callback function infinitely every timer_period seconds


    def timer_callback(self):
        """Method that is periodically called by the timer."""
        self.get_logger().info("Hello")


    # CODE HERE


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but 
    used by ROS2 to configure certain aspects of the Node.
    """
    try: 
        rclpy.init(args=args)                               # initialize ros2 communications
        node = Template()                                   # calls the class Template
        rclpy.spin(node)                                    # loop the node indefinitely
    
    except KeyboardInterrupt:
        pass
    
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
