#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback

# Global variables to store the last messages
last_status_message = None
last_result_message = None
last_feedback_message = None

warn_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)

def goal_status_callback(msg):
    global last_status_message, goal_aborted_published

    for status in msg.status_list:
        if status.status == 4:
            current_message = "Goal was aborted. Possibly due to obstacles."
            
            if current_message != last_status_message:
                rospy.loginfo(current_message)
                last_status_message = current_message
                
            # goal_aborted_published = True

def goal_result_callback(msg):
    global last_result_message

    if msg.status.status == 4:
        current_message = "Goal failed. Potentially due to being out of map bounds."
        ####
        warning_msg = String()
        warning_msg.data = "text,Goal aborted due to obstacles,5"
        print("\nGoal Aborted")
        warn_pub.publish(warning_msg)
        ####
        
        if current_message != last_result_message:
            rospy.loginfo(current_message)
            last_result_message = current_message

def goal_feedback_callback(msg):
    global last_feedback_message

    # Only print the message if it's different from the last one
    if msg.status.text != last_feedback_message:
        rospy.loginfo(f"Feedback: {msg.status.text}")
        last_feedback_message = msg.status.text

def main():
    rospy.init_node('out_of_map_checker')
    
    # Subscribe to the goal status, result, and feedback topics
    rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_result_callback)
    rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, goal_feedback_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
