#!/usr/bin/env python3
import rclpy                    # required for every node
from rclpy.node import Node     # for node creation using OOP
from rclpy import qos
import face_recognition
import cv2
import numpy as np
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FacialRecognition(Node):
    """Facial Recognition"""

    def __init__(self):

        super().__init__("facial_recog")
        self.get_logger().info("facial_recog is running")

        # Declare CvBridge object
        self.bridge = CvBridge()
        
        # Constant
        TIME_PERIOD = 0.5
        self.ABSOLUTE_PATH = "/home/hobo/Robo-Hobo/ros2_ws/src/robo_detection/robo_detection/images"

        # Variable
        self.process_this_frame = True
        self.face_from_recognize_face = ' '    # preset variable for return of recognize_face

        # Lists for image encodings and image names
        self.known_face_encodings = []
        self.known_face_names = []
        self.non_face_paths_to_be_ignored = []

        # Lists used per frames
        self.face_locations = []
        self.face_encodings = []

        # Encode all images 
        self.is_done_encoding_images = False
        self.encode_all_images()
        self.create_timer(20.0, self.encode_all_images)

        # SUBSCRIPTION
        self.v4l2_subscription = self.create_subscription(
            Image, "image_raw", self.recognize_face, 
            qos.qos_profile_sensor_data)

        # PUBLICATION
        self.publisher_ = self.create_publisher(
            String, 'recognized_name', 10)
        self.create_timer(TIME_PERIOD, self.facial_recog_publisher)    # call facial_recog_encodings on loop

        self.publisher_image_encode_stat = self.create_publisher(
            Bool, 'image_encode_stat', 10)
        self.create_timer(TIME_PERIOD, self.send_image_encoding_status)

        ## stat_collector
        self.publisher_facial_recog_node_stat = self.create_publisher(
            String, 'facial_recog_node_stat', 10)
        self.create_timer(TIME_PERIOD, self.send_facial_recog_status)


    def encode_all_images(self):
        """
        Places all file locations from images/ into a list. 
        Parent folder name is also appended on known_face_names as identifier of a person.
        """
        self.error_counter_in_image_encoding = 0
        try:
            self.get_logger().info("IMAGE ENCODING RUNNING... ")

            user_names = [f for f in os.listdir(f"{self.ABSOLUTE_PATH}")]

            for user_name in user_names:
                """
                Breaks loop on hidden folders
                if user_name[0] == ".":
                    break
                """

                """ Encode face images if not yet or if new user directory exists """
                if user_name not in self.known_face_names:
                    """ Get ABSOLUTE PATH of user_name """
                    user_name_path = f"{self.ABSOLUTE_PATH}/{user_name}"

                    """ Encoding """
                    file_names = [f for f in os.listdir(user_name_path)]
                    for file_name in file_names:

                        """ Get ABSOLUTE PATH of image """
                        absolute_image_path = f"{user_name_path}/{file_name}"

                        """ Ignore non-face images """
                        if absolute_image_path not in self.non_face_paths_to_be_ignored:
                            self.known_face_names.append(user_name) # APPEND NAMES TO known_face_names

                            try:
                                encoded_image = self.image_encoder(f"{absolute_image_path}")
                                self.known_face_encodings.append(encoded_image) # APPEND ENCODINGS TO known_face_encodings
                                self.get_logger().info(f"encoded face of {user_name} with file path of {absolute_image_path}")

                            except Exception as image_encoder_error:
                                self.known_face_names.pop()
                                self.non_face_paths_to_be_ignored.append(f"{absolute_image_path}")
                                self.get_logger().error(f"face_encoding_error: {image_encoder_error},\nNon-face image: {absolute_image_path}")
            
        except Exception as image_encoding_error:
            self.get_logger().error(image_encoding_error)
            self.send_facial_recog_status(f"error: {image_encoding_error}")
            self.error_counter_in_image_encoding += 1

        finally:
            if self.error_counter_in_image_encoding == 0:
                self.is_done_encoding_images = True
                self.get_logger().info("ALL IMAGES SUCCESSFULLY ENCODED!")
            elif self.error_counter_in_image_encoding > 0:
                self.get_logger().info("IMAGE ENCODING FAILED")


    def image_encoder(self, image_directory: String):
        """Encodes the image file to be appended to the known_face_encodings."""

        image_source = face_recognition.load_image_file(image_directory)
        # image_location = face_recognition.face_locations(image_source, number_of_times_to_upsample=1, model="cnn")
        image_encoding = face_recognition.face_encodings(image_source, known_face_locations=None, num_jitters=3, model="small")[0]
        return image_encoding


    def facial_recog_publisher(self):
        """This will publish the topic data across other nodes"""
        try:
            msg = String()
            # msg.data = self.recognize_every_other_frame()
            msg.data = self.face_from_recognize_face
            self.publisher_.publish(msg)
            self.get_logger().info(f"{msg.data}")
        except Exception as publisher_error:
            self.get_logger().error(f"error in recognize_face publication: {publisher_error}")


    def recognize_face(self, data: Image):
        """This is the main function to be looped."""

        try:
            # Grab a single frame of video
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # frame = video_capture.read() # testing in local computer

            # face_names = []
            if self.process_this_frame:
                # Resize frame of video to 1/4 size for faster face recognition processing
                small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

                # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
                # rgb_small_frame = small_frame[:, :, ::-1]
                rgb_small_frame = np.ascontiguousarray(small_frame[:, :, ::-1])
                
                # Find all the faces and face encodings in the current frame of video
                self.face_locations = face_recognition.face_locations(rgb_small_frame, number_of_times_to_upsample=3, model="hog")
                self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations, num_jitters=3, model="small")

                face_names = []
                for face_encoding in self.face_encodings:
                    
                    # See if the face is a match for the known face(s)
                    matches = face_recognition.compare_faces(self.known_face_encodings,face_encoding, tolerance=0.57)
                    name = "Unknown"

                    # Or instead, use the known face with the smallest distance to the new face
                    face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)

                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                    face_names.append(name)
                
                if len(face_names) == 0:
                    face_names.append("No Face Detected")

                string_data = ",".join(face_names)
                self.face_from_recognize_face = string_data

        except Exception as exception:
            self.send_facial_recog_status(f"error: {exception}")
            self.get_logger().error(exception)
    

    def send_facial_recog_status(self, stat="ok"):
        msg = String()
        if stat != "ok":
            msg.data = f"error: {stat}"
        else:
            msg.data = "ok"

        self.publisher_facial_recog_node_stat.publish(msg)


    def send_image_encoding_status(self):
        """
        publish Boolean if encode_all_images() is done.
        """
        msg = Bool()
        msg.data = self.is_done_encoding_images
        self.publisher_image_encode_stat.publish(msg)


    def show_known_list(self):
        self.get_logger().info(f"""
known_encodings:{self.known_face_encodings}
known_names:{self.known_face_names}""")


def main(args=None):
    try: 
        rclpy.init(args=args)                               # initialize ros2 communications
        node = FacialRecognition()                          # calls the class OkdoCamera
        rclpy.spin(node)                                    # loop the node indefinitely
    except KeyboardInterrupt:
        node.show_known_list()
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
