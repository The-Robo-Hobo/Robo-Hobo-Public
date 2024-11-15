#!/usr/bin/env python3
import rclpy                                            # required for every node
from rclpy.node import Node                             # for node creation using 

from std_msgs.msg import Bool                         # data type used in publisher
from std_msgs.msg import String

import time
import RPi.GPIO as GPIO


class Flash(Node):                                    # defined class which inherits Node from rclpy
    def __init__(self):                                 # a constructor, required as the first function in every class

        super().__init__("flash_node")                # name of the node
        self.get_logger().info("flash_node is running")

        # constants
        TIMER_PERIOD = 1.0
        self.FLASH_PIN = 16     #pin 36
        self.FLASH_TIME = 1

        # variable
        self.thermal_bool = None
        self.image_encode_stat = False

        # subscriber
        self.flash_node_subscriber = self.create_subscription(
            Bool, "thermal_bool", self.activate_flash, 10)
        
        self.subscriber_image_encode_stat = self.create_subscription(
            Bool, 'image_encode_stat', self.check_image_encode_status, 10)
        
        #publisher
        ## stat_pub
        self.publisher_flash_node_stat = self.create_publisher(
            String, 'flash_node_stat', 10)
        self.create_timer(TIMER_PERIOD,self.send_flash_node_status)

        # pin intialization
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False) 
            GPIO.setup(self.FLASH_PIN, GPIO.OUT)
            GPIO.output(self.FLASH_PIN, GPIO.HIGH)
        except Exception as init_exception:
            self.get_logger().error(init_exception)
            self.send_flash_node_status(init_exception)


    def check_image_encode_status(self, msg:Bool):
        self.image_encode_stat = msg.data
        self.get_logger().info(f"RECEIVED image_encode_stat: {msg.data}")


    def activate_flash(self, msg: Bool):
        """
        Flashes LED based on the thermal_bool subscription.
        """
        self.thermal_bool = msg.data

        try: 
            if self.thermal_bool and self.image_encode_stat:
                GPIO.output(self.FLASH_PIN, GPIO.LOW)
                self.get_logger().info("Flash is on!")
                time.sleep(self.FLASH_TIME)
                GPIO.output(self.FLASH_PIN, GPIO.HIGH)
                self.get_logger().info("Flash is off!")
            else:
                GPIO.output(self.FLASH_PIN, GPIO.HIGH)
                self.get_logger().info("Flash is off!")
                
        except Exception as exception:
            self.get_logger().error(exception)
            self.send_flash_node_status(f"error: {exception}")


    def send_flash_node_status(self, stat="ok"):
        msg = String()
        if stat != "ok":
            msg.data = f"error: {stat}"
        else:
            msg.data = "ok"

        self.publisher_flash_node_stat.publish(msg)


def main(args=None):
    try: 
        rclpy.init(args=args)                              # initialize ros2 communications
        node = Flash()                                   # calls the class Thermal
        rclpy.spin(node)                                   # loop the node indefinitely
    except KeyboardInterrupt:
        GPIO.output(node.FLASH_PIN, GPIO.HIGH)
        print("Flash is OFF")
    except Exception() as error:
        GPIO.output(node.FLASH_PIN, GPIO.HIGH)
        print(error)

    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
