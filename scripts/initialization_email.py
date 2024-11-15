#!/usr/bin/env python3
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.application import MIMEApplication
from email.mime.multipart import MIMEMultipart 
from datetime import datetime


def read_text_file():
    file_path = '/home/hobo/Robo-Hobo/scripts/sysinfo.txt'
    with open(file_path, 'r') as file:
        data = file.read()
        return data


def send_mail():
    # constants
    # date and time for the receiver to easily determine when the event took place
    current_datetime_readable = datetime.now().strftime("%b %d %I:%M:%S %p")
    str_current_datetime_readable = str(current_datetime_readable)


    # insert smtp data here: smtp_server, smtp_port, smtp_username, smtp_password
    smtp_server = 'smtp.example.com'
    smtp_port = 1234 # sample port
    smtp_username = 'smtpusername@example.com'
    smtp_password = 'sample password'       # please secure the credentials


    email_from = 'emailfrom@example.com'
    email_to = ['emailto@example.com']
    email_subject = f'Home Defense Rover is Up and Running: {str_current_datetime_readable}'
    email_body = f'''
    Check the status of the device: \n
    {read_text_file()}
    '''
    email_image = " "

    # initialize connection to our email server, we use gmail
    smtp = smtplib.SMTP(smtp_server, smtp_port) 
    smtp.ehlo() 
    smtp.starttls() 

    smtp.login(smtp_username, smtp_password) 


    def message(subject="", text="", img=""):

        msg = MIMEMultipart()               # build message contents 
        msg['Subject'] = subject            # Add Subject 
        msg.attach(MIMEText(text))          # Add text contents 

        return msg


    # Call the message function 
    msg = message(email_subject,
                email_body,
                email_image)

    # data to the sendmail function
    smtp.sendmail(from_addr=email_from, 
                to_addrs=email_to, 
                msg=msg.as_string()) 

    smtp.quit()         # close the connection 


def main():
    send_mail()


if __name__ == '__main__':
    main()
