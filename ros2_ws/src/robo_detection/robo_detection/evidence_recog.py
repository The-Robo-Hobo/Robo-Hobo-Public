#!/usr/bin/env python3
import rclpy                    # required for every node
from rclpy.node import Node     # for node creation using OOP
import face_recognition
import numpy as np
import os
import datetime
from std_msgs.msg import String


class EvidenceRecognition(Node):
    """Evidence Recognition"""

    def __init__(self):

        super().__init__("evidence_recog")
        self.get_logger().info("evidence_recog is running")

        # Declare CvBridge object
        # self.bridge = CvBridge()
        # self.fr_var = FacialRecognition().

        # Constant
        TIME_PERIOD = 0.5
        self.REGISTRANTS_PATH = "/home/hobo/Robo-Hobo/ros2_ws/src/robo_detection/robo_detection/images"
        self.EVIDENCE_PATH = "/home/hobo/Pictures/evidences"

        # Variable
        # self.process_this_frame = True
        self.face_from_recognize_face = ' '    # preset variable for return of recognize_face
        self._current_recent_img = ' '
        self.file_name = ' '

        # Known names and encodings
        self.known_face_encodings = []
        self.known_face_names = []
        # self.create_timer(10.0, self._update_known_list)

        # Ignore paths
        self.non_face_paths_to_be_ignored = []
        self.ignore_non_face_evidence = []

        # Lists used per webcam frames
        self.face_locations_frames = []
        self.face_encodings_frames = []

        # Lists used per evidence frames
        self.face_locations_evidences = []
        self.face_encodings_evidences = []

        # Encode all images 
        self.encode_all_images()
        self.create_timer(10.0, self.encode_all_images)

        # PUBLICATION evidence_recog_node_stat
        ## recognized face from evidences
        self.evidence_name_publisher = self.create_publisher(
            String, 'recognize_evidence', 10)
        self.create_timer(TIME_PERIOD, self.evidence_recog_publisher)

        ## evidence file_name publisher
        self.file_name_publisher = self.create_publisher(
            String, 'recognized_filename', 10)
        self.create_timer(TIME_PERIOD, self.evidence_filename_publisher)

        ## stat collector
        self.publisher_evidence_recog_node_stat = self.create_publisher(
            String, 'evidence_recog_node_stat', 10)
        self.create_timer(TIME_PERIOD, self.send_evidence_recog_status)
        

    def _contents_from(self, source_path: str) -> list:
        """
            Extracts contents of specified path

            ## Parameters:
                - source_path: path of directory to be extracted.
            ## Return:
             List of contents of the specified directory.
        """
        return [item for item in os.listdir(source_path)]
    

    def _extension_of(self, name):
        """
            Identifies extension of a file.

            ## Parameters:
                - file: file name along with its extension.
            ## Return:
             The extension of the specified file name in String format.
             Else, return "folder" for folder names.
        """
        if "." in name:
            index_fnd = name.find(".")
            return name[index_fnd::]
        else:
            return("folder")


    def _most_recent_item_from(self, dir_list: list) -> str:
        """
            Finds the path of the most recent folder or file from the specified directory.

            ## Parameters:
                - dir_list: list of directory with date as file names.
            ## Return:
                A string value of name of the most recent folder/file.
        """
        try:
            extension_of_sample_item = self._extension_of(dir_list[0])

            if extension_of_sample_item == "folder":
                sorted_list = (sorted(dir_list, key=lambda x: 
                                datetime.datetime.strptime(x, '%Y-%m-%d'), 
                                reverse=True))

            elif extension_of_sample_item == ".png":
                sorted_list = (sorted(dir_list, key=lambda x: 
                                datetime.datetime.strptime(x, '%Y-%m-%d_%H%M_%S.png'), 
                                reverse=True))

            elif extension_of_sample_item == ".jpg":
                sorted_list = (sorted(dir_list, key=lambda x: 
                                datetime.datetime.strptime(x, '%Y-%m-%d_%H%M_%S.jpg'), 
                                reverse=True))

            elif extension_of_sample_item == ".jpeg":
                sorted_list = (sorted(dir_list, key=lambda x: 
                                datetime.datetime.strptime(x, '%Y-%m-%d_%H%M_%S.jpeg'), 
                                reverse=True))

            else:
                self.get_logger().warn("file extension is not of image type.")
                raise Exception("File must be of .png, .jpg, or .jpeg format only!")

        except Exception as strptime_error:
            self.get_logger().error(f"_most_recent_item_from error:{strptime_error}")
            self.get_logger().error("MAKE SURE ALL ITEMS HAVE THE SAME FILE EXTENSIONS.")

        return(sorted_list[0])
    

    def _image_encoder(self, image_directory: str):
        """
            Encodes an image file.

            ## Parameters:
                - image_directory: directory of the image file to be encoded.
            ## Return:
                The first item of encoded image from 128-dimensional face encodings
        """
        image_source = face_recognition.load_image_file(image_directory)
        image_encoding = face_recognition.face_encodings(image_source, known_face_locations=None, num_jitters=1, model="small")[0]
        return image_encoding


    def _latest_file_path_from(self, target_path) -> str:
        """
            Finds the most recent file from the most recent folder.

            ## Parameters:
                - target_path: path of the directory in question
            ## Return:
                String of path of the latest file from the latest folder.
        """
        # create list of contents from evidence path
        evidence_contents = self._contents_from(target_path) # list of paths
        recent_folder = self._most_recent_item_from(evidence_contents) # most recent path
        
        recent_folder_contents = self._contents_from(f"{target_path}/{recent_folder}")
        recent_file_from_recent_folder = self._most_recent_item_from(recent_folder_contents)

        # Encode that longest variable name
        return(f"{target_path}/{recent_folder}/{recent_file_from_recent_folder}")


    def encode_all_images(self):
        """
        Places all file locations from images/ into a list. 
        Parent folder name is also appended on known_face_names as identifier of a person.
        """
        self.error_counter_in_image_encoding = 0
        try:
            self.get_logger().info("IMAGE ENCODING RUNNING... ")
            
            for user_name in self._contents_from(self.REGISTRANTS_PATH):

                if user_name not in self.known_face_names:
                    user_name_path = f"{self.REGISTRANTS_PATH}/{user_name}"

                    # Encoding 
                    for file_name in self._contents_from(user_name_path):

                        # ABSOLUTE PATH of image 
                        absolute_image_path = f"{user_name_path}/{file_name}"

                        if absolute_image_path not in self.non_face_paths_to_be_ignored:
                            self.known_face_names.append(user_name) # APPEND NAMES TO known_face_names

                            try:
                                encoded_image = self._image_encoder(absolute_image_path)
                                self.known_face_encodings.append(encoded_image) # APPEND ENCODINGS TO known_face_encodings
                                self.get_logger().info(f"encoded face of {user_name} with file path of {absolute_image_path}")

                            except Exception as image_encoder_error:
                                self.known_face_names.pop()
                                self.non_face_paths_to_be_ignored.append(absolute_image_path)
                                self.get_logger().error(f"face_encoding_error: {image_encoder_error},\nNon-face image: {absolute_image_path}")
            
        except Exception as image_encoding_error:
            self.get_logger().error(image_encoding_error)
            self.send_facial_recog_status(f"error: {image_encoding_error}")
            self.error_counter_in_image_encoding += 1

        finally:
            if self.error_counter_in_image_encoding == 0:
                self.get_logger().info("ALL IMAGES SUCCESSFULLY ENCODED!")
            elif self.error_counter_in_image_encoding > 0:
                self.get_logger().info("IMAGE ENCODING FAILED")


    def evidence_recog_publisher(self):
        """This publishes facial recognition of saved images"""
        try:
            msg = String()
            self.recognize_face_from_evidence()
            msg.data = self.face_from_recognize_face
            self.evidence_name_publisher.publish(msg)
            self.get_logger().info(msg.data)
        except Exception as publisher_error:
            self.get_logger().error(f"error in recognize_face_from_evidence publication: {publisher_error}")
            self.send_evidence_recog_status(f"error: {publisher_error}")


    def evidence_filename_publisher(self):
        """This publishes facial recognition of saved images""" 
        try:
            msg = String()
            msg.data = self.file_name
            self.file_name_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"error in evidence_filename_publisher publication: {e}")


    def recognize_face_from_evidence(self):
        """This returns recognition result of saved images."""
        # self.get_logger().info("Running recognize_face_from_evidence")

        recent_saved_img = self._latest_file_path_from(self.EVIDENCE_PATH)

        if (recent_saved_img != self._current_recent_img) and (recent_saved_img not in self.ignore_non_face_evidence):
            try:
                image = face_recognition.load_image_file(recent_saved_img)
                self.face_locations_evidences = face_recognition.face_locations(image, number_of_times_to_upsample=3, model="hog")
                self.face_encodings_evidences = face_recognition.face_encodings(image, self.face_locations_evidences, num_jitters=2, model="small")

            except Exception as non_face_image_error:
                self.ignore_non_face_evidence.append(recent_saved_img)
                self.get_logger().error(f"recognize_face_from_evidence() error: {non_face_image_error}")

            face_names = []
            for unknown_face_encoding in self.face_encodings_evidences:
                
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings,unknown_face_encoding, tolerance=0.4)
                name = "Unknown"

                # Or instead, use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(self.known_face_encodings, unknown_face_encoding)
                best_match_index = np.argmin(face_distances)

                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
                face_names.append(name)
            

            if len(face_names) == 0:
                face_names.append("No Face Detected")
            else:
                self.file_name = recent_saved_img
            
            string_data = ",".join(face_names)
            self.face_from_recognize_face = string_data

        self._current_recent_img = recent_saved_img


    def send_evidence_recog_status(self, stat="ok"):
        msg = String()
        if stat != "ok":
            msg.data = f"error: {stat}"
        else:
            msg.data = "ok"

        self.publisher_evidence_recog_node_stat.publish(msg)


    def show_known_list(self):
        self.get_logger().info(f"""
known_encodings:{self.known_face_encodings}
known_names:{self.known_face_names}
ignored evidence: {self.ignore_non_face_evidence}""")


def main(args=None):
    try: 
        rclpy.init(args=args)                               # initialize ros2 communications
        node = EvidenceRecognition()                          # calls the class OkdoCamera
        rclpy.spin(node)                                    # loop the node indefinitely
    except KeyboardInterrupt:
        node.show_known_list()
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
