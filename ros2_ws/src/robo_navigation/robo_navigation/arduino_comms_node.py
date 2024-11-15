#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import sys
import serial
from serial import SerialException
import time


class ArduinoComms(Node):

    def __init__(self) -> None:
        super().__init__("arduino_comms_node")
        self.get_logger().info("arduino_comms_node is running")

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # (<port number>, <baud rate>, <waiting time before calling an error>)
            if(self.ser.isOpen() == False):
                self.get_logger().info("/dev/ttyACM0 port established!")
                self.ser.open()
            else:
                self.get_logger().info("/dev/ttyACM0 port already established!")
        except serial.SerialException:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)  # (<port number>, <baud rate>, <waiting time before calling an error>)
            if(self.ser.isOpen() == False):
                self.ser.open()
                self.get_logger().info("/dev/ttyACM1 port established!")
            else:
                self.get_logger().info("/dev/ttyACM1 port already established!")
        except IOError as e:
            self.ser.close()
            self.ser.open()
            self.get_logger().info ("port was already open, was closed and opened again!")

        time.sleep(.5) # Wait for connection

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # constants
        TIMER_PERIOD = 0.1

        # facial recog status listener
        self.facial_recog_is_functioning = False

        # variable for reinit_serial_port
        self.number_of_tries = 0

        # variables for battery monitoring
        self.batt_percent_data = '0'

        # VARIABLE HOLDERS FROM SUBSCRIBED TOPIC
        self.thermal_orientation_from_thermal_pub = '   '
        self.thermal_bool_from_thermal_pub = False
        self.funcs_with_error = []      # for stat collection
        self.errors_in_funcs = []       # for stat collection

        # SUBSCRIBER to thermal_publisher_node
        self.thermal_orientation_subscription = self.create_subscription(
            String, 'thermal_orientation', self.thermal_orientation_listener, 10)
        
        self.thermal_bool_subscription = self.create_subscription(
            Bool, 'thermal_bool', self.thermal_bool_listener, 10)
        
        self.facial_recog_node_stat_subscription = self.create_subscription(
            String, 'facial_recog_node_stat', self.facial_recog_node_stat_listener, 10)
        
        # PUBLISHER
        self.publisher_arduino_comms_node_stat = self.create_publisher(
            String, 'arduino_comms_node_stat', 10)
        self.create_timer(TIMER_PERIOD, self.send_arduino_comms_node_status)

        self.publish_battery_level = self.create_publisher(
            String, 'battery_level', 10)
        self.create_timer(15.0, self.battery_level_publisher)

        # repeatedly calls function for commincation with Arduino
        self.create_timer(TIMER_PERIOD, self.communicates_data_to_Arduino)


    def thermal_orientation_listener(self, msg: String):
        """ Listens to 'thermal_orientation' topic and saves value of thermal_orientation. """
        try:
            self.thermal_orientation_from_thermal_pub = msg.data
        except Exception as e:
            self.get_logger().error(f"thermal_orientation_listener error: {e}")


    def thermal_bool_listener(self, msg: Bool):
        """ Listens to 'thermal_orientation' topic and saves value of thermal_bool """
        try: 
            self.thermal_bool_from_thermal_pub = msg.data
        except Exception as e:
            self.get_logger().error(f"thermal_bool_listener error: {e}")


    def go_towards(self, direction):
        """ returns value for each behaviors """
        if direction == "forward":
            return 1
        elif direction == "left":
            return 2
        elif direction == "right":
            return 3
        elif direction == "stop":
            return 4
        elif (direction == "ultrasonic") or (direction == "empty"):
            return 5
        else:
            return 5


    def reset_output_buffer(self, func_name: str):
        """
        This clears the buffer to be sent to the Arduino.

        :parameter:
            func_name (str): function name to easily debug which code script uses the function.
        """
        try:
            self.ser.reset_output_buffer()
        except Exception as ser_reset_output_buffer_error:
            self.get_logger().error(f"{func_name} self.ser.reset_output_buffer error: {ser_reset_output_buffer_error}")
        

    def reinit_serial_port(self):
        """
        This reinitialize serial port. It keeps on looking for a serial port and will only stop if one is found.
        """
        self.reset_output_buffer("reinit_serial_port()")
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # (<port number>, <baud rate>, <waiting time before calling an error>)
            if(self.ser.isOpen() == False):
                self.ser.open()
                self.get_logger().info("/dev/ttyACM0 port established!")
            self.number_of_tries = 0
        except serial.SerialException:
            try:
                self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)  # (<port number>, <baud rate>, <waiting time before calling an error>)
                if(self.ser.isOpen() == False):
                    self.ser.open()
                    self.get_logger().info("/dev/ttyACM1 port established!")
                self.number_of_tries = 0
            except:
                self.number_of_tries += 1
                self.get_logger().error(f"trying reinit_serial_port() again TRY: {self.number_of_tries}")
                if self.number_of_tries == 100:
                    raise IOError
                self.reinit_serial_port()       # keep on looking for a serial port until found
        except IOError as e:
            self.ser.close()
            self.ser.open()
            self.get_logger().info ("port was already open, was closed and opened again!")

        time.sleep(0.5)


    def read_batt_percent(self):
        """ Reads battery based from Arduino's analog pin. """
        self.ser.reset_input_buffer()
        line = self.ser.readline().decode('utf-8').strip() # read buffer
        if line.startswith("Percent: "):
            batt_percent = float(line.split(" ")[1])
            self.batt_percent_data = self.battery_level_indicator(round(batt_percent), 20)


    def communicates_data_to_Arduino(self):
        """
        This sends write data to Arduino using serial communication.
        """
        # self.get_logger().info("function communicating with Arduino")
        if self.ser.in_waiting > 0:
            # self.get_logger().info(f"{self.batt_percent_data}")
            self.read_batt_percent()

        if self.facial_recog_is_functioning == True:
            self.reset_output_buffer("communicates_data_to_Arduino()")

            toSerialVal = self.go_towards(self.thermal_orientation_from_thermal_pub[2:])
            self.get_logger().info(f"""Serial port is open: {self.ser.isOpen()}""")
            try:
                if self.thermal_bool_from_thermal_pub is True:
                    if self.thermal_orientation_from_thermal_pub[2:] == "forward":
                        sensor_listening_to = "Thermal"
                        seconds_to_stop_moving = 2.0
                        serial_value_to_stop_motors = 4

                        self.ser.write(f"{serial_value_to_stop_motors}\n".encode('utf-8'))      # Sends data to Serial Arduino
                        time.sleep(seconds_to_stop_moving)
                    
                    elif self.thermal_orientation_from_thermal_pub[:2] == "h-":
                        sensor_listening_to = "Thermal"
                        self.ser.write(f"{toSerialVal}\n".encode('utf-8'))      # Sends data to Serial Arduino
                        self.reset_output_buffer("communicates_data_to_Arduino() elif Thermal")

                    else:
                        pass
                
                else: 
                    sensor_listening_to = "Ultrasonic"
                    self.ser.write(f"{toSerialVal}\n".encode('utf-8'))
                    self.reset_output_buffer("communicates_data_to_Arduino() else Ultrasonic")
                
                self.get_logger().info(f"""
                    Currently Listening to {sensor_listening_to} Sensors
                    self.ser.write info:\t\t{toSerialVal}""")

            except KeyboardInterrupt:
                self.ser.write("4\n".encode('utf-8'))
                self.get_logger().info(f"KeyboardInterrupt\nser.write info:\t{toSerialVal}")

            except Exception as error:
                self.get_logger().error(f"Error in communicates_data_to_Arduino: {str(error)}")
                self.identify_function_status("communicates_data_to_Arduino() error: ", error)
                self.reinit_serial_port()


    def battery_level_publisher(self):
        try:
            msg = String()
            msg.data = f"{self.batt_percent_data}"
            
            self.publish_battery_level.publish(msg)
            self.get_logger().info(f"Battery level: {msg.data}")
        except Exception as battery_publisher_error:
            self.get_logger().error(f"error in battery_level_publisher: {battery_publisher_error}")
            self.identify_function_status("battery_level_publisher() error: ", battery_publisher_error)


    def battery_level_indicator(self, battery_level, divisor):
        """ Returns fixed levels of battery level. """
        modulated_batt = battery_level // divisor 
        if modulated_batt == 5 or battery_level >= 90: # 90 % and above
            return 100
        elif modulated_batt == 4: # 80 % and above
            return 80
        elif modulated_batt == 3: # 60 % and above
            return 60
        elif modulated_batt == 2: # 40 % and above
            return 40
        elif modulated_batt == 1: # 20 % and above
            return 20
        else: # 20 % and below
            return 0


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


    def send_arduino_comms_node_status(self, stat="ok"):
        msg = String()
        if len(self.funcs_with_error) > 0:
            msg.data = f"errors: {self.errors_in_funcs}"
            self.funcs_with_error.clear()
            self.errors_in_funcs.clear()
        else:
            msg.data = "ok"
        self.publisher_arduino_comms_node_stat.publish(msg)


    def facial_recog_node_stat_listener(self, msg: String):
        if msg.data == "ok":
            self.facial_recog_is_functioning = True
        

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ArduinoComms()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.ser.write('4\n'.encode('utf-8'))
        node.ser.close()
    except Exception() as error:
        print(error)
        node.ser.write('4\n'.encode('utf-8'))
        node.ser.close()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
    