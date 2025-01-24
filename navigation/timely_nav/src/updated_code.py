#!/usr/bin/env python

############### Auto Updating the Parameters #########################
# Import necessary libraries
import rospy
from std_msgs.msg import String
from datetime import datetime
import yaml
import os
import getpass

# Get the current username
<<<<<<< HEAD
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

# Initialize goal times
t1 = t2 = t3 = t4 = t5 = t6 = 0
=======
username = getpass.getuser()
print("USERNAME:", username)

# Initialize total number of locations
total_locations = 10
location_times = [0] * total_locations  # Initialize a list to store goal times
time_flags = [0] * total_locations  # Flags to track if a goal has been published
>>>>>>> b947dcb3a9c044eb442b5f5f6d7845029d34de73

print("\n\nTimely Navigation Node Started!!!!")

# Define function to publish goals
def publish_goal():
    global location_times, time_flags

    # Initialize the node with a unique name
    rospy.init_node('publish_goal_node', anonymous=True)

    # Initialize publisher for "/cmd_frm_tablet" topic
    pub = rospy.Publisher("/cmd_frm_tablet", String, queue_size=10)

    # Load parameters initially
    load_parameters()

    # Loop until ROS node is shutdown
    while not rospy.is_shutdown():
        # Get current time
        current_time = datetime.now().strftime('%H:%M')

        # Reload parameters if 5 minutes have passed
        if (rospy.Time.now() - rospy.Duration(10)).to_sec() > last_reload_time:
            load_parameters()

        # Publish goal messages
        publish_goal_messages(current_time, pub)

        # Wait 1 second before checking current time again
        rospy.sleep(1)

# Function to load parameters from .yaml file
def load_parameters():
    global location_times, last_reload_time

    # Read parameters from .yaml file
    with open(f"/home/{username}/rehab_ws/src/rehab_core/navigation/timely_nav/params/time_config.yaml", 'r') as stream:
        try:
            parameters = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Update goal times dynamically
    for i in range(total_locations):
        location_times[i] = parameters[f'goal_time_{i+1}']  # Get goal time for each location

    # Reset the last reload time
    last_reload_time = rospy.Time.now().to_sec()

    # Display the loaded schedule
    print("\n\nTime Schedule Loaded!!!!")
    for i, time in enumerate(location_times):
        print(f"\n Location {i+1}: {time}")

# Function to publish goal messages
def publish_goal_messages(current_time, pub):
    global location_times, time_flags

    # Get current ROS timestamp
    time_stamp_ros = rospy.Time.now()
    time_stamp = str(time_stamp_ros)

    # Loop through all locations and publish goal if current time matches
    for i in range(total_locations):
        if current_time == location_times[i] and time_flags[i] == 0:
            message = f"{time_stamp},goto,1,loc{i+1},{i+1}"
            pub.publish(message)
            time_flags[i] = 1  # Set flag for current location
            print("Sending the robot to the location:", i+1)
            # Reset all other flags to 0
            for j in range(total_locations):
                if j != i:
                    time_flags[j] = 0

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
