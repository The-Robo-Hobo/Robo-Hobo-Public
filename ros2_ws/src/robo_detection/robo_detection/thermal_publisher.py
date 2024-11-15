#!/usr/bin/env python3
import rclpy                                            # required for every node
from rclpy.node import Node                             # for node creation using 

from std_msgs.msg import Bool                         # data type used in publisher
from std_msgs.msg import String

import busio
import board
import adafruit_amg88xx


class Thermal(Node):                                  
    def __init__(self):                               

        super().__init__("thermal_publisher_node")                # camera_node is the name of the nodeZ
        self.get_logger().info("thermal_publisher_node is running")

        # constants
        TIMER_PERIOD = 0.1
        # self.NORMAL_TEMP_MARGIN = 1.8
        # self.HIGH_TEMP_MARGIN = 1.0
        self.bad_pos = (0, 1, 2, 3, 4, 5, 6, 7, 8, 15, 16, 23, 24, 31, 32, 39, 40, 47, 48, 55, 56, 57, 58, 59, 60, 61, 62, 63)
        self.CONSISTENT_HEAT_COUNT_MAX_LIMIT = 3

        # Variable
        self.thermal_bool_var = False
        self.consistent_heat_count = 0

        # Lists for stat collector
        self.funcs_with_error = []
        self.errors_in_funcs = []

        # PUBLICATIONS
        self.publisher_ = self.create_publisher(
            Bool, 'thermal_bool', 10)
        self.create_timer(TIMER_PERIOD, self.thermal)

        self.publisher_thermal_node = self.create_publisher(
            String, 'thermal_node_stat', 10)
        self.create_timer(TIMER_PERIOD, self.send_thermal_node_status)

        self.publisher_thermal_list = self.create_publisher(
            String, 'thermal_orientation', 10)
        self.create_timer(TIMER_PERIOD, self.orientation)

        # initialize variable for amg88xx
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.amg = adafruit_amg88xx.AMG88XX(self.i2c)
            self.get_logger().info("AMG88XX initialized successfully.")
        except Exception as amg88xx_error:
            self.amg88xx_stat = amg88xx_error
            self.identify_function_status("AMG88XX initialization failed: ", amg88xx_error)


    def count_consistent_heat(self, state: bool) -> bool:
        if state is True:
            self.consistent_heat_count += 1
        else:
            self.consistent_heat_count = 0
        
        if self.consistent_heat_count == self.CONSISTENT_HEAT_COUNT_MAX_LIMIT:
            return True
        else:
            return False


    def thermal(self):
        """Send topic data."""
        try:
            msg = Bool()    # sets up boolean interface for data publishing
            msg.data = self.heat_detect()
            msg.data = self.count_consistent_heat(msg.data)
            self.publisher_.publish(msg)

            self.get_logger().info(f"""
    ave_bg_temp:\t{self.ave_room_temp_without_hotspots}
    Max:\t\t{self.max_heat_reading}
    Ave heat block:\t{self.ave_of_highest_heat_block}
    bg-max-diff:\t{ round(self.max_heat_reading - self.ave_room_temp_without_hotspots, 2) }
    NOTE: Negative diff means ave bg temp is higher than ave of hotspots
    Bool return:\t{msg.data}
    orientation string:\t{self.orientation_string}
    detection_list:\t{self.detection_list}
""")

        except Exception as thermal_error:
            self.identify_function_status("thermal()", thermal_error)
            self.get_logger().error(f"thermal_error: {thermal_error}")


    def sensor_input(self):
        """
        This is only used for debugging
        """
        try:
            lst = []
            for row in self.amg.pixels:
                lst.append(",".join(['{0:.1f}'.format(temp) for temp in row]))
            return("\n".join(lst))
        except Exception as sensor_input_error:
            self.identify_function_status("sensor_input()", sensor_input_error)
    
    
    def heat_detect(self):
        # self.get_logger().info("running heat_detect()")
        """
        Reads thermal camera's inputs.
        return bool
        """
        try:
            # self.get_logger().info("running heat_detect() DSA")
            heat_readings = self.amg.pixels
            # self.get_logger().info(f"heat_readings: {heat_readings}")
            self.flat_heat_readings = [x for b in heat_readings for x in b]
            # self.get_logger().info(f"self.flat_heat_readings: {self.flat_heat_readings}")

            """ Modified thermal detect """
            self.max_heat_reading = max(self.flat_heat_readings)
            # self.get_logger().info(f"self.max_heat_reading: {self.max_heat_reading}")

            """ Collect all blocks where center part is a max temp. """
            self.list_of_blocks = []
            for idx in range(64):
                if (idx not in self.bad_pos) and (self.flat_heat_readings[idx] == self.max_heat_reading):
                    self.list_of_blocks.append(self.heat_block(self.flat_heat_readings, idx))

            # self.get_logger().info(f"self.list_of_blocks: {self.list_of_blocks}")

            """ Get ONLY THE HIGHEST sum of block to be used for average room temp. """
            try: 
                highest_heat_block = max(self.list_of_blocks)
            except Exception as highest_heat_block_error:
                # self.get_logger().error(f"self.list_of_blocks return an empty list (its a FEATURE, not a BUG): {highest_heat_block_error}")
                self.ave_room_temp_without_hotspots = round((sum(self.flat_heat_readings)/64),2)
                self.thermal_bool_var = False
                return self.thermal_bool_var
            
            self.ave_room_temp_without_hotspots = round(
                ((sum(self.flat_heat_readings) - highest_heat_block) / 55), 2
            )
            # self.get_logger().info(f"highest_heat_block: {highest_heat_block}")
            # self.get_logger().info(f"self.ave_room_temp_without_hotspots: {self.ave_room_temp_without_hotspots}")

            self.ave_of_highest_heat_block = round(highest_heat_block/9,2)
            # self.get_logger().info(f"self.ave_of_highest_heat_block: {self.ave_of_highest_heat_block}")
            
            """ Stores True or False based on the formula """
            if self.ave_of_highest_heat_block > self.ave_room_temp_without_hotspots:
                self.thermal_bool_var = True
            else:
                self.thermal_bool_var = False

            # self.get_logger().info("heat_detect() DONE. sent: ",self.thermal_bool_var)
            return self.thermal_bool_var

        except Exception as heat_detect_error:
            self.identify_function_status("heat_detect()", heat_detect_error)
            self.get_logger().error(f"heat_detect_error: {heat_detect_error}")


    def heat_block(self, temp_list:list, index:int):
        """ Return sum of heat of block """
        top_portion = sum(temp_list[index-9 : index-6])
        mid_portion = sum(temp_list[index-1 : index+2])
        bot_portion = sum(temp_list[index+7 : index+10])
        return (round((top_portion + mid_portion + bot_portion), 2))


    def orientation(self):
        msg = String()    # sets up boolean interface for data publishing
        msg.data = self.orientation_tag()
        self.publisher_thermal_list.publish(msg)
        self.orientation_string = msg.data


    def orientation_tag(self):
        if self.heat_detect() is True:

            """ Stores index of high temp values """
            idx_list_for_max_heat_reading = []
            for idx in range(len(self.flat_heat_readings)):
                if self.flat_heat_readings[idx] == self.max_heat_reading:
                    idx_list_for_max_heat_reading.append(idx)

            """ Identify location of high temp values """
            self.detection_list = []
            for high_temp_idx in idx_list_for_max_heat_reading:
                self.detection_list.append(self.index_1D_position(high_temp_idx))

            left_count = self.detection_list.count("left")
            front_count = self.detection_list.count("forward")
            right_count = self.detection_list.count("right")
            return "h-" + self.index_tag(left_count, front_count, right_count)
        
        else:
            # self.detection_list = ["empty"]
            return "empty"
        

    def index_tag(self, input1:int, input2:int, input3:int):
        if (input1 < input2 > input3) or (input1 == input2 == input3):
            return "forward"
        elif (input1 > input3) and (input2 == 0):
            return "left"
        elif (input1 < input3) and (input2 == 0):
            return "right"
        else:
            return "empty"
        
    
    def index_1D_position(self, temp_idx: int):
        if (temp_idx % 8) in list(range(0,2)):
            return "left"
        elif (temp_idx % 8) in list(range(2, 6)):
            return "forward"
        elif (temp_idx % 8) in list(range(6, 8)):
            return "right"
        else:
            return "empty"


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

    
    def send_thermal_node_status(self):
        msg = String()
        if len(self.funcs_with_error) > 0:
            msg.data = f"errors: {self.errors_in_funcs}"
            self.funcs_with_error.clear()
            self.errors_in_funcs.clear()
        else:
            msg.data = "ok"
        
        self.publisher_thermal_node.publish(msg)


def main(args=None):
    try: 
        rclpy.init(args=args)                              # initialize ros2 communications
        node = Thermal()                                   # calls the class Thermal
        rclpy.spin(node)                                   # loop the node indefinitely
    except KeyboardInterrupt as kb_interrupt:
        pass
    except Exception() as error:
        print(error)

    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
