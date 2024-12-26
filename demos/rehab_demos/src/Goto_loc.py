#!/usr/bin/env python

############ For Single Location #################
# import rospy
# from std_msgs.msg import Int32
# from geometry_msgs.msg import PoseStamped
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import actionlib

# class GoToLocation:
#     def __init__(self):
#         rospy.init_node('go_to_location_node', anonymous=True)
        
#         # Subscribe to the /test topic
#         rospy.Subscriber('/test', Int32, self.test_callback)
        
#         # Action client for move_base
#         self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         rospy.loginfo("\n\nWaiting for move_base action server...")
#         self.move_base_client.wait_for_server()
#         rospy.loginfo("\n\nMove_base action server found")

#     def test_callback(self, data):
#         # Check if the data received is equal to 1
#         if data.data == 1:
#             rospy.loginfo("\n\nReceived 1 on /test topic. Sending robot to specific location.")
#             self.send_goal_to_move_base()

#     def send_goal_to_move_base(self):
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.pose.position.x = 1.9065395852392248
#         goal.target_pose.pose.position.y = 0.01362221537328496
#         goal.target_pose.pose.orientation.w = 1.0

#         rospy.loginfo("\n\nSending goal to move_base: x={}, y={}".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
#         self.move_base_client.send_goal(goal)
#         self.move_base_client.wait_for_result()

# if __name__ == '__main__':
#     try:
#         GoToLocation()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Interrupted")
##################################################

############ For Multiple Locations with Orientation #################
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class GoToLocation:
    def __init__(self):
        rospy.init_node('go_to_location_node', anonymous=True)
        
        # Subscribe to the /test topic
        rospy.Subscriber('/test', Int32, self.test_callback)
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server found")
        
        # Define the locations with orientation
        self.locations = {
            1: {'position': (3.021913427755831, 2.829701823041944), 'orientation': (0.0, 0.0, 0.9972647996313916, 0.07391156483366117)},  # Define x1, y1 and orientation for location 1
            2: {'position': (8.505291442781076, 6.017916069495288), 'orientation': (0.0, 0.0, 0.842009204904278, 0.5394631580158794)},  # Define x2, y2 and orientation for location 2
            3: {'position': (6.557712416903309, 4.2940385916625265), 'orientation': (0.0, 0.0, 0.23973168831055053, 0.9708391821613779)},  # Define x3, y3 and orientation for location 3
            4: {'position': (5.4772573127168025, 0.46573999651603115), 'orientation': (0.0, 0.0, 0.9999825822877574, 0.005902128523554669)},  # Define x4, y4 and orientation for location 4
            # 5: {'position': (2.6427218034016544, -1.3318543159989995), 'orientation': (0.0, 0.0, 0.5509736441491293, 0.8345226440624776)},  # Define x5, y5 and orientation for location 5
            # 6: {'position': (5.6231338379719515, -0.34994496906474076), 'orientation': (0.0, 0.0, -0.8946991747575416, 0.44666921394716025)}   # Define x6, y6 and orientation for location 6
        }

    def test_callback(self, data):
        # Check if the data received is within the range of defined locations
        if data.data in self.locations:
            rospy.loginfo("Received {} on /test topic. \n\nSending robot to location {}.".format(data.data, data.data))
            self.send_goal_to_move_base(data.data)
        else:
            rospy.loginfo("\nInvalid location received: {}".format(data.data))

    def send_goal_to_move_base(self, location):
        position = self.locations[location]['position']
        orientation = self.locations[location]['orientation']

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation = Quaternion(*orientation)

        rospy.loginfo("Sending goal to move_base: x={}, y={}, orientation={}".format(position[0], position[1], orientation))
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

if __name__ == '__main__':
    try:
        GoToLocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
##############################################################################