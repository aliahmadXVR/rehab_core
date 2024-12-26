#!/usr/bin/env python3

# Import necessary libraries
import rospy
from std_msgs.msg import String
from datetime import datetime, time

# Initialize goal times
t1 = t2 = t3 = t4 = t5 = 0

# Define function to publish goals
def publish_goal():
    # Access global variables
    global t1, t2, t3, t4, t5

    # Initialize the node with a unique name
    rospy.init_node('publish_goal_node', anonymous=True)

    # Get ROS parameters for goal times
    goal_time_1 = rospy.get_param('goal_time_1')
    goal_time_2 = rospy.get_param('goal_time_2')
    goal_time_3 = rospy.get_param('goal_time_3')
    goal_time_4 = rospy.get_param('goal_time_4')
    goal_time_5 = rospy.get_param('goal_time_5')

    # Initialize publisher for "/cmd_frm_tablet" topic
    pub = rospy.Publisher("/cmd_frm_tablet", String, queue_size=10)

    # Get current time as a timestamp
    time_stamp_ros = rospy.Time.now()
    time_stamp = str(time_stamp_ros)
    #print (time_stamp)

    # Loop until goal time is reached
    while True:
        current_time = datetime.now().strftime('%H:%M')

        # Publish goal message for goal 1 if goal time has been reached and goal 1 hasn't been reached yet
        if current_time == goal_time_1 and t1 == 0:
            message = time_stamp + ",goto,1,loc1,1"
            pub.publish(message)
            t1 = 1
            t2 = t3 = t4 = t5 = 0

        # Publish goal message for goal 2 if goal time has been reached and goal 2 hasn't been reached yet
        if current_time == goal_time_2 and t2 == 0:
            message = time_stamp + ",goto,1,loc2,2"
            pub.publish(message)
            t2 = 1
            t1 = t3 = t4 = t5 = 0

        # Publish goal message for goal 3 if goal time has been reached and goal 3 hasn't been reached yet
        if current_time == goal_time_3 and t3 == 0:
            message = time_stamp + ",goto,1,loc3,3"
            pub.publish(message)
            t3 = 1
            t1 = t2 = t4 = t5 = 0

        # Publish goal message for goal 4 if goal time has been reached and goal 4 hasn't been reached yet
        if current_time == goal_time_4 and t4 == 0:
            message = time_stamp + ",goto,1,loc4,4"
            pub.publish(message)
            t4 = 1
            t1 = t2 = t3 = t5 = 0

        # Publish goal message for goal 5 if goal time has been reached and goal 5 hasn't been reached yet
        if current_time == goal_time_5 and t5 == 0:
            message = time_stamp + ",goto,1,loc5,5"
            pub.publish(message)
            t5 = 1
            t1 = t2 = t3 = t4 = 0

        # Wait 1 second before checking current time again
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
