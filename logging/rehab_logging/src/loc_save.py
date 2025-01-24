#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import os

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)


print("\n Location Saving Node Started!!")

current_location = None

def tablet_callback(data):
    message = data.data
    message_parts = message.split(',')

    if len(message_parts) == 5 and message_parts[1] == 'saveloc':
        location_id = message_parts[2]

        if location_id == '14':  # Location 1
            save_location(1)
        elif location_id == '15':  # Location 2
            save_location(2)
        elif location_id == '16':  # Location 3
            save_location(3)
        elif location_id == '17':  # Location 4
            save_location(4)
        elif location_id == '18':  # Location 5
            save_location(5)
        elif location_id == '19':  # Location 6
            save_location(6)
        elif location_id == '20':  # Location 3
            save_location(7)
        elif location_id == '21':  # Location 4
            save_location(8)
        elif location_id == '22':  # Location 5
            save_location(9)
        elif location_id == '23':  # Location 6
            save_location(10)

def save_location(location_id):
    global current_location

    # Set the file path for the YAML file
    
    # file_path = '/home/orin2/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml'
    file_path = f'/home/{username}/rehab_ws/src/rehab_core/behavior/rehab_behavior/params/location.yaml'
    
    print("The Yaml File Path: ", file_path)

    # Check if the location.yaml file exists
    if not os.path.exists(file_path):
        # Create the YAML file if it doesn't exist
        with open(file_path, 'w') as file:
            yaml.dump({}, file)

    # Update the current_location using the latest coordinates from /amcl_pose
    if current_location is None:
        rospy.logwarn("Current location is not available. Unable to save location.")
        return

    # Get the position and orientation from the current_location
    position = current_location.pose.pose.position
    orientation = current_location.pose.pose.orientation

    # Create a dictionary with the location parameters
    location_params = {
        'position': {
            'x': position.x,
            'y': position.y,
            'z': position.z
        },
        'orientation': {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w
        }
    }

    # Load existing locations from the file
    with open(file_path, 'r') as file:
        existing_locations = yaml.safe_load(file) or {}

    # Update the existing locations with the new values
    existing_locations['location%d' % location_id] = location_params

    # Save the updated locations to the YAML file
    with open(file_path, 'w') as file:
        yaml.dump(existing_locations, file)


def amcl_pose_callback(data):
    global current_location
    current_location = data

if __name__ == '__main__':
    rospy.init_node('location_saver_node', anonymous=True)

    # Subscribe to '/cmd_frm_tablet' topic
    rospy.Subscriber('/cmd_frm_tablet', String, tablet_callback)

    # Subscribe to '/amcl_pose' topic
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    rospy.spin()
