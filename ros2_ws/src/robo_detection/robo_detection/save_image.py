#!/usr/bin/env python3
import rclpy
import sys
import cv2
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from os import makedirs


class SaveImage(Node):
    """
    Convert image message from topic image_raw of v4l2_camera to cv2
    and save them as a PNG file.
    """

    def __init__(self) -> None:

        super().__init__("save_image_node")
        self.get_logger().info("save_image_node is running")

        # variables
        self.bridge = CvBridge()
        self.thermal_trigger = None     # preset variable for storing if 30 deg Celsius is detected
        timer_period = 1.1
        self.file_name = " "
        self.funcs_with_error = []      # for stat collection
        self.errors_in_funcs = []       # for stat collection

        # subscribers
        self.camera_subscription = self.create_subscription(
            Image, "image_raw", self.convert_and_save_image, qos.qos_profile_sensor_data)

        self.thermal_subscription = self.create_subscription(
            Bool, 'thermal_bool', self.capture, 10)
        
        # publishers
        self.email_notif_publisher_ = self.create_publisher(
            String, 'saved_image_filename', 10)
        self.timer_ = self.create_timer(timer_period, self.send_email_notif)

        self.save_image_node_stat_publisher = self.create_publisher(
            String, 'save_image_node_stat', 10)
        self.create_timer(timer_period, self.send_node_status)


    def capture(self, msg: Bool): 
        try: 
            self.thermal_trigger = msg.data
            self.get_logger().info(f"thermal readings: {msg.data}")
        except Exception() as error:
            self.get_logger().error(error)
            self.identify_function_status("capture", error)
    

    def convert_and_save_image(self, data):
        current_datetime = datetime.now().strftime("%Y-%m-%d_%H%M_%S")
        str_current_datetime = str(current_datetime)
        folder_datetime = datetime.now().strftime("%Y-%m-%d")
        str_folder_datetime = str(folder_datetime)

        if self.thermal_trigger is True:
            try: 
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as error:
                self.get_logger().info(error)
                self.identify_function_status("cv_image", error)
            
            try:
                self.file_name = str_current_datetime
                save_image_path = f"/home/hobo/Pictures/evidences/{str_folder_datetime}"

                # cv2.imwrite(f"/home/hobo/Pictures/evidences/{self.file_name}.png", cv_image)
                # self.get_logger().info(f"Saved '{self.file_name}' at /home/hobo/Pictures/evidences/")

                try:
                    makedirs(save_image_path, exist_ok=True)
                    self.get_logger().info(f"created directory: {save_image_path}")
                except Exception as makedir_error:
                    self.get_logger().error(f"makedir error: {makedir_error}")

                cv2.imwrite(f"{save_image_path}/{self.file_name}.png", cv_image)
                self.get_logger().info(f"Saved '{self.file_name}' at '{save_image_path}'")

            except Exception() as exception:
                self.identify_function_status("save_image", exception)


    def send_email_notif(self):
        msg = String()
        if self.thermal_trigger is True:
            try:
                msg.data = self.file_name
            except Exception as error:
                self.get_logger().info(error)
                self.identify_function_status("send_email_notif", error)

        else:
            msg.data = " "
        self.email_notif_publisher_.publish(msg)


    def identify_function_status(self, func: String, error: String):
        """
        Collects functions with errors.
        """
        try:
            self.funcs_with_error.append(func)
            self.errors_in_funcs.append(error)
        except:
            pass

        if len(self.funcs_with_error) > 0:
            return "error"
        else:
            return "ok"

    
    def send_node_status(self):
        msg = String()
        if len(self.funcs_with_error) > 0:
            msg.data = f"errors: {self.errors_in_funcs}"
            self.funcs_with_error.clear()
            self.errors_in_funcs.clear()
        else:
            msg.data = "ok"
        self.save_image_node_stat_publisher.publish(msg)


def main(args=None):
    
    try: 
        rclpy.init(args=args)               # initialize ros2 communications
        node = SaveImage()                  # calls the class OkdoCamera
        rclpy.spin(node)                    # loop the node indefinitely
    except KeyboardInterrupt as kb_interrupt:
        pass
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                        # shutdown ros2 communications


if __name__ == '__main__':
    main(sys.argv)
