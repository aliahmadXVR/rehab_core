#!/usr/bin/env python

####### For publishing the Goal Cancel Message only once on the button click ########
import rospy
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String

print("\nGoal Cancelleation via Joystick node Started!!!!")

class CancelGoalOnJoy:
    def __init__(self):
        rospy.init_node('cancel_goal_on_joy', anonymous=True)

        # Define the button index for RB (this may vary depending on your joystick model)
        self.rb_button_index = 5  # RB button is typically index 5, adjust if necessary

        # Track the previous state of the RB button
        self.rb_button_prev_state = 0

        # Subscriber to the joystick topic
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Publisher to cancel the current goal
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Publisher to send feedback to the tablet
        self.feedback_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)

    def joy_callback(self, joy_msg):
        # Get the current state of the RB button
        rb_button_current_state = joy_msg.buttons[self.rb_button_index]

        # Check if the RB button has just been pressed (transition from unpressed to pressed)
        if rb_button_current_state == 1 and self.rb_button_prev_state == 0:
            # RB button was just pressed, cancel the current move_base goal
            rospy.loginfo("RB button pressed. Cancelling the current goal...")

            # Cancel the move_base goal
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)

            # Publish feedback message to the tablet
            feedback_msg = String()
            feedback_msg.data = "text,Emergency Stop,5"
            self.feedback_pub.publish(feedback_msg)

        # Update the previous state of the RB button
        self.rb_button_prev_state = rb_button_current_state


if __name__ == '__main__':
    try:
        CancelGoalOnJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#######################################################################################    
    


####### For keep publishing the Goal Cancel Message on the button click & hold ########

# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Joy
# from std_msgs.msg import String
# from actionlib_msgs.msg import GoalID
# from std_msgs.msg import Empty

# class CancelGoalOnJoy:
#     def __init__(self):
#         rospy.init_node('cancel_goal_on_joy', anonymous=True)

#         # Define the button index for RB (this may vary depending on your joystick model)
#         self.rb_button_index = 5  # RB button is typically index 5, adjust if necessary

#         # Subscriber to the joystick topic
#         self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

#         # Publisher to cancel the current goal
#         self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
#         # Publisher to send feedback to the tablet
#         self.goal_msg_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)

#     def joy_callback(self, joy_msg):
#         if joy_msg.buttons[self.rb_button_index] == 1:
#             # RB button is pressed, cancel the current move_base goal
#             rospy.loginfo("\nRB button pressed. Cancelling the current goal...")
            
#             # Cancel the move_base goal
#             cancel_msg = GoalID()
#             self.cancel_pub.publish(cancel_msg)
#             print("\n Gaol Cancelled via Joystick!!!!")
            
#             # Publish the message on tablet. 
#             goal_msg = String()
#             goal_msg.data = "text,Emergency Stop,5"
#             # goal_msg.data = "Goal Cancelled via Joystick"
#             self.goal_msg_pub.publish(goal_msg)


# if __name__ == '__main__':
#     try:
#         CancelGoalOnJoy()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

