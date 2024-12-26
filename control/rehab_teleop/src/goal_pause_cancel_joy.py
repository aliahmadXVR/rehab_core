#!/usr/bin/env python

# ####### For publishing the Goal Cancel Message and Pausing the goal for specific time ########
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, String
from actionlib_msgs.msg import GoalID

class JoyController:
    def __init__(self):
        rospy.init_node('joy_controller', anonymous=True)

        # Publishers
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.pause_pub = rospy.Publisher('/pause_goal', Int32, queue_size=10)
        
        # Publisher to send feedback to the tablet
        self.feedback_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)

        # Variables to track button states
        self.last_rb_state = False  # RB button state
        self.last_rt_state = False  # RT button state

        # Subscriber
        rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, joy_msg):
        # Check if RB button (button index 5) is pressed
        rb_pressed = joy_msg.buttons[5] == 1
        if rb_pressed and not self.last_rb_state:
            
            rospy.loginfo(f"RB button pressed. Button value: {joy_msg.buttons[5]}")
            
            # Publish the Int 2 for Emergency Stop ##
            self.pause_pub.publish(Int32(2))
            
            # Cancel the goal on move_base
            rospy.loginfo("Cancelling goal via Joystick")
            self.cancel_pub.publish(GoalID())

            #### Comment the following 4 lines if pause_cancel_goal.py is started automatically through launch #### 
            #### It will avoid doubling of "Emergency Stop" publishing ####
            # Publish feedback message to the tablet
            feedback_msg = String()
            feedback_msg.data = "text,Emergency Stop,5"
            self.feedback_pub.publish(feedback_msg)
            #######################################################################################################
            
        elif not rb_pressed and self.last_rb_state:
            rospy.loginfo(f"RB button released. Button value: {joy_msg.buttons[5]}")
            
        self.last_rb_state = rb_pressed

        # Check if RT button (axis index 5) is pressed (RT axis is -1.0 when pressed)
        rt_pressed = joy_msg.axes[5] == -1.0
        if rt_pressed and not self.last_rt_state:
            rospy.loginfo(f"RT button pressed. Axis value: {joy_msg.axes[5]}")
            # Publish Int32 data 1 on /pause_goal
            self.pause_pub.publish(Int32(1))
                        
        elif not rt_pressed and self.last_rt_state:
            rospy.loginfo(f"RT button released. Axis value: {joy_msg.axes[5]}")
        self.last_rt_state = rt_pressed

    def run(self):
        rospy.loginfo("JoyController node started. Listening to /joy topic...")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = JoyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
