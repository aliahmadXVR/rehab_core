#!/usr/bin/env python3

#### This node is written to pause the goal on move_base for a defined time and then resume the gaol####
#### This feature can be used to hault the goal during through speech command during conversational interaction####
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_msgs.msg import Int32, String
from actionlib_msgs.msg import GoalID

class PauseResumeNavigation:
    def __init__(self):
        rospy.init_node('pause_resume_navigation', anonymous=True)
        print("\n Pausing Goal Node Started!!!!")

        # Subscribers and Publishers
        rospy.Subscriber('/pause_goal', Int32, self.pause_callback)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
        # Publisher to send feedback to the tablet
        self.feedback_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)
        
        # Action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # State variables
        self.paused = False
        self.pause_time = 60  # Pause duration in seconds
        self.saved_goal = None

    def goal_callback(self, msg):
        """
        Callback to capture and save the current goal sent to move_base.
        Extracts the goal's target_pose from MoveBaseActionGoal.
        """
        self.saved_goal = MoveBaseGoal()
        self.saved_goal.target_pose = msg.goal.target_pose  # Extract the target_pose
        rospy.loginfo("Goal received and saved.")

    def pause_callback(self, msg):
        if msg.data == 1 and not self.paused and self.saved_goal:
            rospy.loginfo("Pausing goal")
            self.paused = True

            # Publish feedback message to the tablet
            feedback_msg = String()
            feedback_msg.data = "text,Goal Paused,5"
            self.feedback_pub.publish(feedback_msg)

            # Cancel the current goal
            self.cancel_pub.publish(GoalID())

            # Pause for a specific time
            rospy.Timer(rospy.Duration(self.pause_time), self.resume_goal, oneshot=True)
        
        #### For Emergency Stop ####    
        elif msg.data == 2:
            rospy.loginfo("Cancelling goal via Joystick")
            # Cancel all current goals
            self.cancel_pub.publish(GoalID())
            self.saved_goal = None
            self.paused = False
            
            # # Publish feedback message to the tablet
            # feedback_msg = String()
            # feedback_msg.data = "text,Emergency Stop,5"
            # self.feedback_pub.publish(feedback_msg)
        #############################
            
    def resume_goal(self, event):
        if self.saved_goal:
            rospy.loginfo("Resuming goal")
            self.client.send_goal(self.saved_goal)
            self.paused = False

            # Publish feedback message to the tablet
            feedback_msg = String()
            feedback_msg.data = "text,Resuming Goal,5"
            self.feedback_pub.publish(feedback_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PauseResumeNavigation()
        node.run()
    except rospy.ROSInterruptException:
        pass
