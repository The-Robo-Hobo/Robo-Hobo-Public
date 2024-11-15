#!/usr/bin/env python3
import rclpy                        # required for every node
from rclpy.node import Node         # for node creation using OOP

import pyrebase
import time
from std_msgs.msg import String
from os import makedirs
from datetime import datetime
from timeout import timeout


class WebServer(Node):              # defined class which inherits Node from rclpy
    """
    A ros2 node that communicates with the web server 
    to upload, download, and update data.
    """

    def __init__(self):                                 # a constructor, required as the first function in every class
        super().__init__("web_server_node")             # the name of the node
        self.get_logger().info("web_server_node is running")        # way to write a log

        # constants
        timer_period = 1.0  #seconds
        config = {
            # located in firebase project config, i guess
        }

        server_user_email = "sampleuseremail@example.com"
        server_user_pass = "sample password"        # please secure the credentials
        storage_base_path = "sample_path"

        # initializations
        try:
            firebase = pyrebase.initialize_app(config)      # configuration initialization
            self.auth = firebase.auth()                          # to use firebase authetication
            user = self.auth.sign_in_with_email_and_password(server_user_email, server_user_pass)
            self.id_token = user['idToken']                 # for future authenticated firebase requests
            self.storage = firebase.storage()               # to access firebase storage
            self.db = firebase.database()                   # to perform database operations
        except Exception as firebase_init_error:
            self.get_logger().error(f"init firebase error: {firebase_init_error}")
            self.send_web_server_node_status(f"firebase_init error: {firebase_init_error}")

        # initializations to refresh registered users
        self.user_path = "hobo"
        self.registered_users_path = f"/home/{self.user_path}/Robo-Hobo/ros2_ws/src/robo_notification/robo_notification/files"
        self.registered_users_file = f"{self.registered_users_path}/registered_users.txt"
        try:
            makedirs(self.registered_users_path, exist_ok=True)
        except Exception as init_refresh_error:
            self.get_logger().error(f"init refresh users error: {init_refresh_error}")
        self.write_text_file(self.registered_users_file, "REGISTERED USERS")


        # subscriber
        self.subscriber_saved_image_filename = self.create_subscription(
            String, 'saved_image_filename', self.upload_saved_image, 10)
        
        self.subscriber_stat_collector = self.create_subscription(
            String, 'node_status', self.update_system_status, 10)

        self.subscriber_stat_co = self.create_subscription(
            String, 'battery_level', self.battery_update, 10)
        
        # publishers
        self.publisher_new_user = self.create_publisher(        ## facial_recog update of new user 
            String, 'new_user_data', 10)
        self.create_timer(30, self.publish_new_user_name)

        self.publisher_web_server_node_stat = self.create_publisher(
            String, 'web_server_node_stat', 10)
        self.create_timer(timer_period, self.send_web_server_node_status)


    def write_data(self, database_path, data: dict):
        self.db.child(database_path).push(data)
        self.get_logger().info(f"WROTE on '{database_path}': data = {data}")


    def get_url(self, storage_path):
        url = self.storage.child(storage_path).get_url(None)
        self.get_logger().info(f"URL received: '{url}'")
        return url
        

    def update_data(self, system, data: dict):
        """
        Update the system information of either Raspberry Pi or Robo Nodes.

        :param:
            system (str): the second layer of directory after root
            data (dict): {<key>: <value>}
        
        note:
            system arguments accepted: raspberrypi, robo_nodes
        """
        try:
            path = "sysinfo"
            self.db.child(path).child(system).update(data)
            self.get_logger().info(f"UPDATED data of '{path}/{system}'")
        except Exception as update_error:
            self.get_logger().error(f"update_error: {update_error}")
            self.send_web_server_node_status(f"update_data error: {update_error}")


    def get_data(self, key, value):
        """
        Get user's name to locate the directory to download images from.

        :param: 
            key (str): key in dictionary, aka locator or index
            value (str): value in dictionary, aka data

        note:
            key arguments accepted: birthdate, date_registered, email, first_name, folder_name, gender, last_name, mobile_no, role, username
        """
        try: 
            users = self.db.child("users").get()
            for user in users.each():
                one_user_data = user.val()
                server_key_value = one_user_data[key]

                if value in server_key_value:
                    self.get_logger().info(f"get user data: {key}: {server_key_value}")
                    return server_key_value

        except Exception as get_data_error:
            self.get_logger().error(f"get_data error: {get_data_error}")
            self.send_web_server_node_status(f"get_data error: {get_data_error}")


    @timeout(5, "Connection Timeout")
    def upload_file(self, server_path, local_file_path, id_token=None):
        """
        Upload file to firebase storage for backup and safekeeping.
        """
        try:
            self.get_logger().info(f"func_test: inside the upload_file(); will proceed to communicate using 'self.storage.child({server_path}).put({local_file_path}, {id_token})'")
            # time.sleep(1)     # in case the image is not fully loaded yet
            # self.get_logger().info(f"func_test: slept in 1 second to make sure the image is done saving")
            self.storage.child(server_path).put(local_file_path, id_token)
            self.get_logger().info(f"UPLOADED image: '{local_file_path}' to '{server_path}'")
        
        except Exception as upload_error:
            self.get_logger().error(f"upload error: {upload_error}")
            self.send_web_server_node_status(f"upload_file error: {upload_error}")


    def download_user_images(self, user_type, user_directory, id_token=None):
        """
        Download images from firebase storage to save locally 
        and use these images to teach facial_recog_node the face of homeowners / visitors.

        :param: 
            user_type (str): if homeowner or guest
            user_directory (str): the directory name to download images from; will be based on get_data()
        """
        try:
            server_user_images_path = f"images/{user_type}/{user_directory}"
            local_user_images_path = f"/home/{self.user_path}/Robo-Hobo/ros2_ws/src/robo_detection/robo_detection/images/{user_directory}"
            makedirs(f"{local_user_images_path}/", exist_ok=True)

            image_number = str(1)
            all_images = self.storage.child(server_user_images_path).list_files() 
            for image in all_images:

                if server_user_images_path in image.name:
                    self.storage.download(image.name, f"{local_user_images_path}/{user_directory}_image{image_number}.jpg", id_token)
                    self.get_logger().info(f"downloaded '{image.name}' as '{user_directory}_image{image_number}.jpg'")

                    count = int(image_number)
                    image_number = str(count + 1)
            
            self.get_logger().info(f"Finished downloading {user_directory}'s images to '{local_user_images_path}'")

        except Exception as download_error:
            self.get_logger().error(f"download error: {download_error}")
            self.send_web_server_node_status(f"download_user_images error: {download_error}")


    def upload_image_url(self, file_name, image_storage_path, database_path):
        url = str(self.get_url(image_storage_path))
        data = {
            "file_name": file_name,
            "url": url
        }
        self.write_data(database_path, data)
        

    def upload_saved_image(self, msg: String):
        folder_datetime = datetime.now().strftime("%Y-%m-%d")
        str_folder_datetime = str(folder_datetime)

        try: 
            user = self.auth.refresh(user['refreshToken'])  # refresh the user token before 1 hour expiry
            self.id_token = user['idToken']                 # for future authenticated firebase requests
            self.get_logger().info(f"REFRESHED user token")
        except: 
            pass

        try:
            filename = msg.data
            self.get_logger().info(f"RECEIVED image filename: {filename}")

            if str_folder_datetime in filename:
                self.get_logger().info(f"func_test: Passed the condition 'if str_folder_datetime in filename'; will proceed to self.upload_file()")
                server_path = f"evidences/{str_folder_datetime}/{filename}.png"
                local_file_path = f"/home/{self.user_path}/Pictures/evidences/{str_folder_datetime}/{filename}.png"
                try:
                    self.upload_file(server_path, local_file_path, self.id_token)
                except:
                    self.get_logger().error(f"UPLOAD FAILED: connection timeout")
                
                database_path = f"evidences/{str_folder_datetime}"
                self.upload_image_url(filename, server_path, database_path)

                self.get_logger().info(f"SLEEPING for 15 seconds")
                time.sleep(15)
        
        except Exception as upload_image_error:
            self.get_logger().error(f"upload_saved_image error: {upload_image_error}")
            self.send_web_server_node_status(f"upload_saved_image error: {upload_image_error}")


    def write_text_file(self, file_path, text):
        with open(file_path, "w") as file:
            file.write(f"{text}\n")
            self.get_logger().info(f"write_text: {text}")


    def append_text_file(self, file_path, texts: list):
        with open(file_path, "a") as file:
            for text in texts:
                file.write(f"{text}\n")
            self.get_logger().info(f"append_text: {texts}")


    def read_text_file(self, file_path):
        with open(file_path, "r") as file:
            texts = file.readlines()
            self.get_logger().info(f"read_text: {texts}")
            return texts


    def call_download_user_images(self, new_user: list):
        for user in new_user:
            self.get_logger().info(f"downloading {user}'s images...")

            user_data = user.split(" ")
            user_directory = user_data[0]

            # to download image even if user has two first names with space
            for index in range(len(user_data)):
                if (user_data[index] == "homeowner") or (user_data[index] == "visitor"): 
                    user_role = user_data[index]
                    self.download_user_images(user_role, user_directory, self.id_token)


    def refresh_registered_users(self) -> list:
        """
        For raspi to update the facial_recog_node if there's new registered user.

        :return: 
            (list): directory name of new users
        """
        self.get_logger().info("REFRESHING registered users...")
        try:
            registered_users = self.read_text_file(self.registered_users_file)
            self.get_logger().info(f"REGISTERED users: {registered_users}")
            new_users = []
            
            users = self.db.child("users").get()
            for user in users.each():
                one_user_data = user.val()

                user_directory = one_user_data["folder_name"]
                user_role = one_user_data["role"]
                user_email = one_user_data["email"]
                user_data = f"{user_directory} {user_role} {user_email}"
                
                if user_data+"\n" not in registered_users:
                    new_users.append(user_data)
                    self.get_logger().info(f"{user_data} was added to new_users")
                else:
                    # self.get_logger().info(f"{user_data} is already registered")
                    pass

            # self.get_logger().info(f"new users: {new_users}")
            self.append_text_file(self.registered_users_file, new_users)
            self.call_download_user_images(new_users)

            return new_users
            
        except Exception as refresh_error:
            self.get_logger().error(f"refresh users error: {refresh_error}")
            self.send_web_server_node_status(f"refresh_users error: {refresh_error}")


    def convert_to_dictionary(self, msg: str, keys_new_name: dict) -> dict:
        """
        To clean the msg data received from node_status.

        :param:
            msg (str)
            keys_new_name (dict): contains the {<received_status_key>: <server_key_name>}

        :return: 
            (dictionary): {server_key_name: value}
        """
        data_dict = {}
        
        data = msg.replace(" ", "")
        data_list = data.split("\n")
        data_list2 = []
        for data in data_list:
            if ":" in data:
                data_list2.append(data.split(":"))

        flat_data = [x for b in data_list2 for x in b]
        # self.get_logger().info(f"flat_data: {flat_data}")

        keys = list(keys_new_name.keys())
        for data in flat_data:

            if data in keys:
                key_index = flat_data.index(data)
                value_index = key_index + 1
                
                value = flat_data[value_index].replace("%", "")     # must be integer or float only
                value = value.replace("C", "")
                value = value.split("usedof")[0]

                try:
                    value = float(value)
                except:
                    pass

                data_dict[keys_new_name[data]] = value

        # self.get_logger().info(f"data_dict: {data_dict}")
        return data_dict
        
    
    def battery_update(self, msg: String):
        battery_level = int(msg.data)
        data = {"battery": battery_level}
        self.update_data("raspberrypi", data)


    def update_system_status(self, msg: String):
        try:
            msg = msg.data
            raspi_stat_new_names = {
                "CPUUsage": "cpu_usage",
                "MemoryUsage": "memory_usage",
                "Usageof/": "storage_usage",
                "Temperature": "temperature"
            }
            raspi_stat_new = self.convert_to_dictionary(msg, raspi_stat_new_names)
            self.update_data("raspberrypi", raspi_stat_new)

            nodes_new_name = {
                'save_image_node': 'save_image',
                'thermal_publisher_node': 'thermal',
                'facial_recog_node': 'facial_recognition',
                'flash_node': 'flash',
                'siren_node': 'siren',
                'arduino_comms_node': 'navigation',
                'web_server_node': 'web_server'
            }
            robo_nodes_stat = self.convert_to_dictionary(msg, nodes_new_name)
            self.update_data("robo_nodes", robo_nodes_stat)

        except Exception as update_system_stat_error:
            self.get_logger().error(f"update_system_stat error: {update_system_stat_error}")
            self.send_web_server_node_status(f"update_system error: {update_system_stat_error}")

    
    def clean_user_data(self, user_data: str) -> list:
        """
        :parameter:
            user_data (str): new_user_data topic to be cleaned
        
        :return: 
            (list): containing new_user_data
        """
        try:
            chars_to_be_removed = ["[", "]", "'", ","]
            for char in chars_to_be_removed:
                user_data = user_data.replace(char, "")
            
            return user_data
        
        except Exception as clean_user_error:
            self.get_logger().error(f"clean_user_data error: {clean_user_error}")
            return []


    def publish_new_user_name(self):
        msg = String()
        try:
            msg.data = str(self.refresh_registered_users())
            msg.data = str(self.clean_user_data(msg.data))
        except Exception as publish_new_user_err:
            self.get_logger().error(f"publish_new_user_error: {publish_new_user_err}")

        self.publisher_new_user.publish(msg)
        self.get_logger().info(f"PUBLISHED new users: {msg.data}")


    def send_web_server_node_status(self, stat="ok"):
        msg = String()
        if stat != "ok":
            msg.data = f"error: {stat}"
        else:
            msg.data = "ok"

        self.publisher_web_server_node_stat.publish(msg)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but 
    used by ROS2 to configure certain aspects of the Node.
    """
    try: 
        rclpy.init(args=args)       # initialize ros2 communications
        node = WebServer()          # calls the class
        rclpy.spin(node)            # loop the node indefinitely
    except KeyboardInterrupt:
        pass
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                # shutdown ros2 communications


if __name__ == '__main__':
    main()
