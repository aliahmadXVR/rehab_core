#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import yaml

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

isNavigate = False


class PositionChecker:
    def __init__(self):
        rospy.init_node('position_checker', anonymous=True)

        # file_path = '/home/orin2/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml'
        file_path = f'/home/{username}/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml'
        print("The Yaml File Path: ", file_path)

        # Load the locations from the yaml file
        with open(file_path, 'r') as file:
            self.locations = yaml.safe_load(file)

        # Subscribe to the /amcl_pose topic
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Sucscribe the /move_base/status topic
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

        # Publisher for the /current_position topic
        self.position_pub = rospy.Publisher('/current_position', String, queue_size=10)

        # Store the robot's current position
        self.current_position = None

        # Set a timer to publish the current position every 10 seconds
        rospy.Timer(rospy.Duration(3), self.publish_position)

    def status_callback(self, msg):
        global isNavigate
        # print("\nNavigating?", isNavigate)
        for status in msg.status_list:
            if status.status == 1:  # Check if the goal is active
                isNavigate = True
                return
        isNavigate = False
    
    def pose_callback(self, data):
        # Get the current position of the robot
        self.current_position = data.pose.pose.position

    def publish_position(self, event):
        if self.current_position is None:
            return

        x = self.current_position.x
        y = self.current_position.y

        # Check if the current position matches any of the locations in location.yaml
        current_location = None
        tolerance = 0.5  # Define tolerance for position matching. Increasing tolerance will increase the true area around the true point.
        for location_name, location_data in self.locations.items():
            location_x = location_data['position']['x']
            location_y = location_data['position']['y']
            if abs(location_x - x) < tolerance and abs(location_y - y) < tolerance:
                current_location = location_name
                break

        if isNavigate == True:
            msg = "Robot is Navigating!"
            # continue
        
        elif current_location:
            msg = f"Robot is at {current_location}"
        else:
            msg = "Robot is at some other location"

        # Publish the message
        self.position_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        checker = PositionChecker()
        checker.run()
    except rospy.ROSInterruptException:
        pass

