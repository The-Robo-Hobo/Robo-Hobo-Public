#!/usr/bin/env python3
import rclpy                                            # required for every node
from rclpy.node import Node                             # for node creation using OOP

import os
import time
import smtplib 
from email.mime.text import MIMEText 
from email.mime.image import MIMEImage 
from email.mime.application import MIMEApplication 
from email.mime.multipart import MIMEMultipart 
from datetime import datetime
from std_msgs.msg import String


class EmailNotification(Node):                                      # defined class named OkdoCamera which inherits Node from rclpy
    """
    Sends an email with image attached to the homeowner.
    """

    def __init__(self):                                             # a constructor, required as the first function in every class
        super().__init__("email_notif_node")                        # camera_node is the name of the node
        self.get_logger().info("email_notif_node is running")       # way to write a log
        
        # global variables
        self.email_image = []
        self.new_email_address = []

        # subscribers
        self.send_email_subscription_ = self.create_subscription(
            String, "saved_image_filename", self.add_image, 10)
        
        self.subscriber_new_user_data = self.create_subscription(
            String, "new_user_data", self.add_email_address_to_notify, 10)
        
        self.subcriber_evidence_filename = self.create_subscription(
            String, "recognized_filename", self.add_recognized_filename, 10)

        self.subscriber_node_status = self.create_subscription(
            String, "node_status", self.add_node_status, 10)


    def add_image(self, msg: String):
        folder_date = datetime.now().strftime("%Y-%m-%d")
        str_folder_date = str(folder_date)
        image_min = 1
        image_max = 10
        
        if str_folder_date in msg.data:
            self.get_logger().info(f"RECEIVED IMAGE: {msg.data}")
            
            if len(self.email_image) < image_max:
                try:
                    image_path = f"/home/hobo/Pictures/evidences/{str_folder_date}/{msg.data}.png"
                    if image_path not in self.email_image:
                        self.email_image.append(image_path)
                        self.get_logger().info(f"ADDED {msg.data}.png to image list")
                except FileNotFoundError as e:
                    self.get_logger().info(e)
                    pass
                except Exception as e:
                    print(e) 

            elif len(self.email_image) >= image_min:
                self.call_send_email()
        
        elif len(self.email_image) >= image_min:
            self.call_send_email()
        else:
            self.get_logger().info("WAITING for an image")

    
    def add_node_status(self, msg: String):
        self.msg_node_statuses = msg.data
        self.get_logger().info(f"Received status message...")
    

    def add_email_address_to_notify(self, msg: String):
        user_data = msg.data
        user_data = user_data = user_data.split(" ")

        for index in range(len(user_data)):
            if user_data[index] == "homeowner":
                self.new_email_address.append(user_data[index+1])

    
    def add_recognized_filename(self, msg: String):
        """
        Add image with unrecognized face as an attachment to send to email.
        """
        image_path = msg.data

        try:
            if image_path not in self.email_image:
                self.email_image.append(image_path)
                self.get_logger().info(f"ADDED {msg.data} to image list")
        except FileNotFoundError as e:
            self.get_logger().error(e)
            pass
        except Exception as e:
            print(e) 


    def send_email(self):
        # date and time for the receiver to easily determine when the event took place
        current_datetime_readable = datetime.now().strftime("%b %d %I:%M:%S %p")
        str_current_datetime_readable = str(current_datetime_readable)

        # node status
        try:
            node_statuses = self.msg_node_statuses
        except: 
            node_statuses = " "

        # insert smtp data here: smtp_server, smtp_port, smtp_username, smtp_password
        smtp_server = 'smtp.example.com'
        smtp_port = 1234 # sample port
        smtp_username = 'smtpusername@example.com'
        smtp_password = 'sample password'       # please secure the credentials

        email_from = 'emailfrom@example.com'
        email_to = ['emailto@example.com']                           # list of emails to send mail 
        
        # add emails to email_to to be notified
        for email in self.new_email_address:
            email_to.append(email)
            self.get_logger().info(f"added {email} to be notified")
        
        email_subject = f'Intruder Alert! {str_current_datetime_readable}'
        email_body = f'''
        An intruder has been detected! Check the attached photos for reference and take the necessary action.\n
        \n
        {node_statuses}
        '''
        email_image = self.email_image

        # initialize connection to our email server, we use gmail
        smtp = smtplib.SMTP(smtp_server, smtp_port) 
        smtp.ehlo() 
        smtp.starttls() 

        smtp.login(smtp_username, smtp_password) 


        def message(subject="", text="", img=""): 
            msg = MIMEMultipart()               # build message contents 
            msg['Subject'] = subject            # Add Subject 
            msg.attach(MIMEText(text))          # Add text contents 
            
            try:
                if img is not None:                 # Check if we have anything given in the img parameter 
                    if type(img) is not list:       # Check whether we have the lists of images or not! 
                        img = [img]                 # if it isn't a list, make it one 

                    for one_img in img: 
                        try: 
                            img_data = open(one_img, 'rb').read()                   # read the image binary data 
                            msg.attach(MIMEImage(img_data,                          # Attach the image data to MIMEMultipart 
                                                name=os.path.basename(one_img)))    # using MIMEImage, we add the given filename use os.basename 
                            self.get_logger().info(f"ATTACHED {one_img} to mail")
                        except Exception as add_image_error:
                            self.get_logger().error(f"error one_image add image: {add_image_error}")

            except Exception as message_error:
                self.get_logger().error(f"error message(): {message_error}")

            return msg 


        # Call the message function 
        msg = message(email_subject,
                    email_body,
                    email_image)

        # data to the sendmail function
        smtp.sendmail(from_addr=email_from, 
                    to_addrs=email_to, 
                    msg=msg.as_string()) 
        
        self.get_logger().info(f"SENT an email to: {email_to}")

        smtp.quit()         # close the connection 
    

    def call_send_email(self):
        self.get_logger().info(f"No. of Images to be Sent: {len(self.email_image)}")
        
        self.send_email()
        self.email_image.clear()

        self.get_logger().info(f"No of Image in the list: {len(self.email_image)}")
        self.get_logger().info("SLEEPING for 30 seconds")
        time.sleep(30.0)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but 
    used by ROS2 to configure certain aspects of the Node.
    """
    try: 
        rclpy.init(args=args)                               # initialize ros2 communications
        node = EmailNotification()                          # calls the class OkdoCamera
        rclpy.spin(node)                                    # loop the node indefinitely
    except KeyboardInterrupt:
        pass
    except Exception() as error:
        print(error)
        
    rclpy.shutdown()                                    # shutdown ros2 communications


if __name__ == '__main__':
    main()
