#!/usr/bin/env python

############### Auto Updating the Parameters #########################
# Import necessary libraries
import rospy
from std_msgs.msg import String
from datetime import datetime
import yaml

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

# Initialize goal times
t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0

print("\n\nTimely Navigation Node Started!!!!")

# Define function to publish goals
def publish_goal():
    # Access global variables
    global t1, t2, t3, t4, t5, t6, t7, t8, t9, t10

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

def load_parameters():
    # Access global variables
    global goal_time_1, goal_time_2, goal_time_3, goal_time_4, goal_time_5, goal_time_6, goal_time_7, goal_time_8, goal_time_9, goal_time_10, last_reload_time

    # Read parameters from .yaml file
    
    # with open("/home/orin2/test_ws/src/Rehabbot-EETeam/timely_nav/params/time_config.yaml", 'r') as stream:
    with open(f"/home/{username}/test_ws/src/Rehabbot-EETeam/timely_nav/params/time_config.yaml", 'r') as stream:
        try:
            parameters = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Update global variables
    goal_time_1 = parameters['goal_time_1']
    goal_time_2 = parameters['goal_time_2']
    goal_time_3 = parameters['goal_time_3']
    goal_time_4 = parameters['goal_time_4']
    goal_time_5 = parameters['goal_time_5']
    goal_time_6 = parameters['goal_time_6']
    goal_time_7 = parameters['goal_time_7']
    goal_time_8 = parameters['goal_time_8']
    goal_time_9 = parameters['goal_time_9']
    goal_time_10 = parameters['goal_time_10']

    # Update last reload time
    last_reload_time = rospy.Time.now().to_sec()

    print("\n\nTime Schedule Loaded!!!!")
    print("\n Location 1", goal_time_1)
    print("\n Location 2", goal_time_2)
    print("\n Location 3", goal_time_3)
    print("\n Location 4", goal_time_4)
    print("\n Location 4", goal_time_5)
    print("\n Location 6", goal_time_6)
    print("\n Location 7", goal_time_7)
    print("\n Location 8", goal_time_8)
    print("\n Location 9", goal_time_9)
    print("\n Location 10", goal_time_10)

def publish_goal_messages(current_time, pub):
    # Access global variables
    global t1, t2, t3, t4, t5, t6, t7, t8, t9, t10

    # Get current time as a timestamp
    time_stamp_ros = rospy.Time.now()
    time_stamp = str(time_stamp_ros)
    print(time_stamp)

    # Publish goal messages based on current time
    if current_time == goal_time_1 and t1 == 0:
        message = time_stamp + ",goto,1,loc1,1"
        pub.publish(message)
        t1 = 1
        t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0

    if current_time == goal_time_2 and t2 == 0:
        message = time_stamp + ",goto,1,loc2,2"
        pub.publish(message)
        t2 = 1
        t1 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0

    if current_time == goal_time_3 and t3 == 0:
        message = time_stamp + ",goto,1,loc3,3"
        pub.publish(message)
        t3 = 1
        t1 = t2 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0

    if current_time == goal_time_4 and t4 == 0:
        message = time_stamp + ",goto,1,loc4,4"
        pub.publish(message)
        t4 = 1
        t1 = t2 = t3 = t5 = t6 = t7 = t8 = t9 = t10 = 0

    if current_time == goal_time_5 and t5 == 0:
        message = time_stamp + ",goto,1,loc5,5"
        pub.publish(message)
        t5 = 1
        t1 = t2 = t3 = t4 = t6 = t7 = t8 = t9 = t10 = 0

    if current_time == goal_time_6 and t6 == 0:
        message = time_stamp + ",goto,1,loc6,5"
        pub.publish(message)
        t6 = 1
        t1 = t2 = t3 = t4 = t5 = t7 = t8 = t9 = t10 = 0
        
    if current_time == goal_time_7 and t7 == 0:
        message = time_stamp + ",goto,1,loc7,5"
        pub.publish(message)
        t7 = 1
        t1 = t2 = t3 = t4 = t5 = t6 = t8 = t9 = t10 = 0
        
    if current_time == goal_time_8 and t8 == 0:
        message = time_stamp + ",goto,1,loc8,5"
        pub.publish(message)
        t8 = 1
        t1 = t2 = t3 = t4 = t5 = t6 = t7 = t9 = t10 = 0
        
    if current_time == goal_time_9 and t9 == 0:
        message = time_stamp + ",goto,1,loc9,5"
        pub.publish(message)
        t9 = 1
        t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t10 = 0
    
    if current_time == goal_time_10 and t10 == 0:
        message = time_stamp + ",goto,1,loc10,5"
        pub.publish(message)
        t10 = 1
        t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = 0

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass

