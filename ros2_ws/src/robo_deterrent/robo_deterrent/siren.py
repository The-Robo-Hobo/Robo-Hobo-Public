#!/usr/bin/env python3
import rclpy                                            # required for every node
from rclpy.node import Node                             # for node creation using OOP

import time
import RPi.GPIO as GPIO
from std_msgs.msg import String


class Siren(Node):                                      # defined class which inherits Node from rclpy
    """
    Subscribes to a topic that sends a signal for the siren to make noise and deter intruders.
    """

    def __init__(self):                                     # a constructor, required as the first function in every class
        super().__init__("siren_node")                      # siren_node is the name of the node
        self.get_logger().info("siren_node is running")     # way to write a log

        # contants
        TIMER_PERIOD = 1.0
        self.SIREN_PIN = 20     # pin 38
        self.MAKE_NOISE_SECONDS = 30     # period in seconds to make noise
        self.UNKNOWN_FACE_MAX_LIMIT = 1

        # variables
        self.unknown_face_count = 0

        # subscribers
        ## facial_recog
        self.subscriber_facial_recog_name = self.create_subscription(
            String, 'recognized_name', self.facial_recognize_name, 10)

        ## saved_img_recog
        self.subscriber_saved_img_recog_name = self.create_subscription(
            String, 'recognize_evidence', self.saved_img_recognize_name, 10)
        
        # publishers
        ## stat_collector
        self.publisher_siren_node_stat = self.create_publisher(
            String, 'siren_node_stat', 10)
        self.create_timer(TIMER_PERIOD, self.send_siren_node_status)

        # initializations
        try: 
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False) 
            GPIO.setup(self.SIREN_PIN, GPIO.OUT)
            GPIO.output(self.SIREN_PIN, GPIO.HIGH)
        except Exception as init_exception:
            self.get_logger().error(init_exception)
            self.send_siren_node_status(init_exception)

    
    def make_noise(self, period_seconds):
        try:
            GPIO.output(self.SIREN_PIN, GPIO.LOW)
            self.get_logger().info(f"siren is making noise for {period_seconds}")
            time.sleep(period_seconds)
            GPIO.output(self.SIREN_PIN, GPIO.HIGH)
        except Exception as exception:
            self.get_logger().error(exception)
            self.send_siren_node_status(f"error: {exception}")


    def facial_recognize_name(self, msg: String):
        """
        Count the number of instances an unknown individual was detected 
        consecutively before alerting the homeowner.
        """
        self.get_logger().info(f"received: {msg.data}")

        if msg.data == "Unknown":
            self.unknown_face_count += 1
        else:
            self.unknown_face_count = 0

        if self.unknown_face_count == self.UNKNOWN_FACE_MAX_LIMIT:
            self.make_noise(self.MAKE_NOISE_SECONDS)


    def saved_img_recognize_name(self, msg: String):
        """
        Count the number of instances an unknown individual was detected 
        consecutively before alerting the homeowner.
        """
        self.get_logger().info(f"received: {msg.data}")

        if msg.data == "Unknown":
            self.unknown_face_count += 1
        else:
            self.unknown_face_count = 0

        if self.unknown_face_count == self.UNKNOWN_FACE_MAX_LIMIT:
            self.make_noise(self.MAKE_NOISE_SECONDS)
    

    def send_siren_node_status(self, stat="ok"):
        msg = String()
        if stat != "ok":
            msg.data = f"error: {stat}"
        else:
            msg.data = "ok"

        self.publisher_siren_node_stat.publish(msg)


def main(args=None):

    try:
        rclpy.init(args=args)       # initialize ros2 communications
        node = Siren()              # calls the class Electrocute
        rclpy.spin(node)            # loop the node indefinitely
    except KeyboardInterrupt:
        GPIO.output(node.SIREN_PIN, GPIO.HIGH)
        print("Siren is OFF")
    except Exception as error:
        GPIO.output(node.SIREN_PIN, GPIO.HIGH)
        print("Siren is OFF")
        print(error)
    
    rclpy.shutdown()                # shutdown ros2 communications


if __name__ == '__main__':
    main()
