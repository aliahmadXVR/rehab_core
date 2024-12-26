#!/usr/bin/env python3

import os
import csv
import rospy
import rospkg
import datetime
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String, Float32MultiArray

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class NavigationStatusLogger:
    def __init__(self):
        rospy.init_node('navigation_status_logger', anonymous=False)

        # Set up parameters
        self.log_file = rospy.get_param('~log_file', 'navigation_status.csv')
        # self.log_directory = rospy.get_param('~log_directory', '/home/orin2/rehab_ws/src/Rehabbot-EETeam/rehab_logging/src/')
        self.log_directory = rospy.get_param('~log_directory', f'/home/{username}/rehab_ws/src/Rehabbot-EETeam/rehab_logging/src/')
        self.file_path = os.path.join(self.log_directory, self.log_file)

        # Create CSV file if it doesn't exist
        if not os.path.isfile(self.file_path):
            with open(self.file_path, mode='w') as csv_file:
                fieldnames = ['Timestamp', 'Status']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()

        # Subscribe to move_base action result
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.result_callback)

        # Initialize last status
        self.last_status = None
        self.last_env_info_time = rospy.Time.now()

        # Subscribe to cmd_frm_tablet topic
        rospy.Subscriber('/cmd_frm_tablet', String, self.tablet_callback)
        rospy.Subscriber('/environmental_info', Float32MultiArray, self.env_callback)

    def result_callback(self, data):
        with open(self.file_path, mode='a') as csv_file:
            fieldnames = ['Timestamp', 'Status']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            # Get current timestamp
            current_time = datetime.datetime.now()

            # Check if navigation was successful or not
            if data.status.status == 3:
                status = 'Navigation successful'
            else:
                status = 'Navigation failed'

            # Check if the status has changed since the last write
            if status != self.last_status:
                # Write data to CSV file
                writer.writerow({'Timestamp': current_time, 'Status': status})
                self.last_status = status


    def tablet_callback(self, data):
        with open(self.file_path, mode='a') as csv_file:
            fieldnames = ['Timestamp', 'Status']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            # Get current timestamp
            current_time = datetime.datetime.now()

            # Extract status from received message
            message = data.data
            message_list = message.split(",")
            status = message_list[1]

            # Write data to CSV file
            writer.writerow({'Timestamp': current_time, 'Status': status})

    def env_callback(self, data):
        current_time = rospy.Time.now().to_sec()
        last_env_info_time_sec = self.last_env_info_time.to_sec()
        time_elapsed = current_time - last_env_info_time_sec

        if time_elapsed >= 300:  # 5 minutes have elapsed since last write
            with open(self.file_path, mode='a') as csv_file:
                fieldnames = ['Timestamp', 'Temperature', 'Humidity', 'Pressure', 'Noise', 'Light']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                # Get current timestamp and environmental info
                current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Convert to a human-readable format
                _, _, _, _,press, temp, hum, _, _, _, _ = data.data

                # Round the values to 3 decimal places
                temp = round(temp, 3)
                hum = round(hum, 3)
                press = round(press, 3)

                # Write data to CSV file
                writer.writerow({'Timestamp': current_time, 'Temperature': "Temperature:"+str(temp), 'Humidity': ' Humidity:' +str(hum), 'Pressure': ' Pressure:'+str(press)})

                self.last_env_info_time = rospy.Time.now()


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nav_logger = NavigationStatusLogger()
    nav_logger.run()
