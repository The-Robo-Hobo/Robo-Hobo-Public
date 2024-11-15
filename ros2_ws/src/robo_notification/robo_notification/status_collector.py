#!/usr/bin/env python3
import rclpy                    # required for every node
from rclpy.node import Node     # for node creation using OOP

import psutil
import platform
from datetime import datetime
from std_msgs.msg import String
from gpiozero import CPUTemperature


class StatusCollector(Node):    # defined class name which inherits Node from rclpy
    """
    Collects the status of all nodes from all the packages.
    Sends the status to email_notification node.
    """

    def __init__(self):                             # a constructor, required as the first function in every class
        super().__init__("stat_collector_node")     # first_node is the name of the node
        self.get_logger().info("stat_collector_node is running")    # way to write a log

        # constants
        timer_period = 0.1          # seconds for publisher to publish

        # subscribers from robo_detection package
        ## save_image_node
        self.subscriber_save_image_node_stat = self.create_subscription(
            String, 'save_image_node_stat', self.func_save_image_node_stat, 10)
        
        ## evidence_recog_node
        self.subscriber_evidence_recog_node_stat = self.create_subscription(
            String, 'evidence_recog_node_stat', self.func_evidence_recog_node_stat, 10)

        ## thermal_publisher_node
        self.subscriber_thermal_node_stat = self.create_subscription(
            String, 'thermal_node_stat', self.func_thermal_node_stat, 10)
        
        ## facial_recog_node
        self.subscriber_facial_recog_node_stat = self.create_subscription(
            String, 'facial_recog_node_stat', self.func_facial_recog_node_stat, 10)

        ## flash_node
        self.subscriber_flash_node_stat = self.create_subscription(
            String, 'flash_node_stat', self.func_flash_node_stat, 10)

        # subscribers from robo_deterrent package
        ## siren_node
        self.subscriber_siren_node_stat = self.create_subscription(
            String, 'siren_node_stat', self.func_siren_node_stat, 10)

        # subscribers from robo_navigation package
        ## arduino_comms_node
        self.subscriber_arduino_commms_node_stat = self.create_subscription(
            String, 'arduino_comms_node_stat', self.func_arduino_comms_node_stat, 10)

        # subscribers from web_server_node
        self.subscriber_web_server_node_stat = self.create_subscription(
            String, 'web_server_node_stat', self.func_web_server_node_stat, 10)

        # publisher for robo_notification package to send all node statuses to email_notif_node
        self.publisher_node_status = self.create_publisher(
            String, 'node_status', 10)
        self.create_timer(15, self.publish_node_status)

    
    def func_save_image_node_stat(self, msg: String):
        self.msg_save_image_node_stat = msg.data
        self.get_logger().info(f"save_image_node_stat: {msg.data}")


    def func_evidence_recog_node_stat(self, msg: String):
        self.msg_evidence_recog_node_stat = msg.data
        self.get_logger().info(f"evidence_recog_node_stat: {msg.data}")
    

    def func_thermal_node_stat(self, msg: String):
        self.msg_thermal_node_stat = msg.data
        self.get_logger().info(f"thermal_node_stat: {msg.data}")


    def func_facial_recog_node_stat(self, msg: String):
        self.msg_facial_recog_node_stat = msg.data
        self.get_logger().info(f"facial_recog_node_stat: {msg.data}")


    def func_flash_node_stat(self, msg: String):
        self.msg_flash_node_stat = msg.data
        self.get_logger().info(f"flash_node_stat: {msg.data}")
    

    def func_siren_node_stat(self, msg: String):
        self.msg_siren_node_stat = msg.data
        self.get_logger().info(f"siren_node_stat: {msg.data}")
    

    def func_arduino_comms_node_stat(self, msg: String):
        self.msg_arduino_comms_node_stat = msg.data
        self.get_logger().info(f"arduino_comms_node_stat: {msg.data}")

    
    def func_web_server_node_stat(self, msg: String):
        self.msg_web_server_node_stat = msg.data
        self.get_logger().info(f"web_server_node_stat: {msg.data}")
    

    def get_size(self, bytes, suffix="B"):
        """
        Scale bytes to its proper format
        e.g:
            1253656 => '1.20MB'
            1253656678 => '1.17GB'
        """
        factor = 1024
        for unit in ["", "K", "M", "G", "T", "P"]:
            if bytes < factor:
                return f"{bytes:.2f}{unit}{suffix}"
            bytes /= factor

    
    def raspberrypi_status(self):

        # variables system information
        uname = platform.uname()
        svmem = psutil.virtual_memory()
        partitions = psutil.disk_partitions()
        for partition in partitions:
            if partition.mountpoint == '/':
                partition_root = partition
                try:
                    partition_usage = psutil.disk_usage(partition.mountpoint)
                except PermissionError:
                    # this can be catched due to the disk that isn't ready
                    continue
        cpu_temp = CPUTemperature()

        raspi_status_msg = f"""
            Raspberry Pi Status:
            --------------------
            CPU Usage : {psutil.cpu_percent()}%
            Temperature : {cpu_temp.temperature} C
            Memory Usage : {svmem.percent}% used of {self.get_size(svmem.total)}
            Usage of {partition_root.mountpoint} : {partition_usage.percent}% used of {self.get_size(partition_usage.total)}
        """

        return raspi_status_msg


    def node_status_stat_handling(self, msg_node_stat):
        """
        Handles messages received by the subscribers.

        params: 
            msg_node_stat: msg from the subscriber
        returns: 
            node_stat: String, "ok" or "error: {error}"
        """
        try:
            node_stat = msg_node_stat
            if node_stat != "ok":
                raise Exception
        except Exception as node_error:
            node_stat = f"error: {node_error}"

        return node_stat


    def node_status(self):
        try: 
            save_image_node_stat = self.msg_save_image_node_stat
            if save_image_node_stat != "ok":
                raise Exception
        except Exception as save_image_node_error:
            save_image_node_stat = f"error: {save_image_node_error}"

        try: 
            evidence_recog_node_stat = self.msg_evidence_recog_node_stat
            if evidence_recog_node_stat != "ok":
                raise Exception
        except Exception as evidence_recog_node_stat_error:
            evidence_recog_node_stat = f"error: {evidence_recog_node_stat_error}"

        try:
            thermal_node_stat = self.msg_thermal_node_stat
            if thermal_node_stat != "ok":
                raise Exception
        except Exception as thermal_node_error:
            thermal_node_stat = f"error: {thermal_node_error}"

        try:
            facial_recog_node_stat = self.msg_facial_recog_node_stat
            if facial_recog_node_stat != "ok":
                raise Exception
        except Exception as facial_recog_node_error:
            facial_recog_node_stat = f"error: {facial_recog_node_error}"

        try:
            flash_node_stat = self.msg_flash_node_stat
            if flash_node_stat != "ok":
                raise Exception
        except Exception as flash_node_error:
            flash_node_stat = f"error: {flash_node_error}"

        try:
            siren_node_stat = self.msg_siren_node_stat
            if siren_node_stat != "ok":
                raise Exception
        except Exception as siren_node_error:
            siren_node_stat = f"error: {siren_node_error}"

        try:
            arduino_comms_node_stat = self.msg_arduino_comms_node_stat
            if arduino_comms_node_stat != "ok":
                raise Exception
        except Exception as arduino_comms_node_error:
            arduino_comms_node_stat = f"error: {arduino_comms_node_error}"

        try:
            web_server_node_stat = self.msg_web_server_node_stat
            if web_server_node_stat != "ok":
                raise Exception
        except Exception as web_server_node_error:
            web_server_node_stat = f"error: {web_server_node_error}"

        node_status_msg = f"""
            ROS2 Node Status:
            --------------------
            save_image_node         : {save_image_node_stat}
            thermal_publisher_node  : {thermal_node_stat}
            facial_recog_node       : {facial_recog_node_stat}
            evidence_recog_node     : {evidence_recog_node_stat}
            flash_node              : {flash_node_stat}
            siren_node              : {siren_node_stat}
            arduino_comms_node      : {arduino_comms_node_stat}
            web_server_node         : {web_server_node_stat}
        """

        return node_status_msg


    def publish_node_status(self):
        """
        Publish statuses of all nodes.
        """

        msg = String()
        msg.data = f"""
            {self.raspberrypi_status()}
            {self.node_status()}
        """

        self.publisher_node_status.publish(msg)
        self.get_logger().info(f"Published: \n{msg.data}")


def main(args=None):
    
    try: 
        rclpy.init(args=args)       # initialize ros2 communications
        node = StatusCollector()    # calls the class 
        rclpy.spin(node)            # loop the node indefinitely
    except KeyboardInterrupt:
        pass
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                # shutdown ros2 communications


if __name__ == '__main__':
    main()
