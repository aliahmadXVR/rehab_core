#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import numpy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from math import pow, atan2, sqrt
from sensor_msgs.msg import JointState
from rehab_msgs.msg import bumper

undock_distance = 0.3 #travel undock_distance on undock command 
stop_distance = 0.6   #stop before docking station when the distance between tag and base_link is stop_distance
navigation_point =  [1.1, 0, -0.99, 0.063] #its the goal_point in front of docking station convention=x,y,th,w
# navigation_point =  [5.12, -0.91, 0.338, 0.94] #its the goal_point in front of docking station convention=x,y,th,w

target = Pose()
target.position.x = 0
target.position.y = 0

robot_x = robot_y = robot_yaw =  0
marker_pose  = marker_y = marker_yaw =  0
marker_x = 0
marker_id1 = "not_in_frame"
def visual_callback(msg):
    global marker_x, marker_y, marker_yaw, marker_id1
    try:
        marker_x = msg.pose.position.x
        marker_y = msg.pose.position.y
        marker_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        marker_euler = euler_from_quaternion(marker_quaternion)
        marker_yaw = marker_euler[2]+np.pi/2
        marker_id1 = "in_frame"
    except:
        marker_id1 = "not_in_frame"
        print("marker 1 not detected")

def odom_callback(odom):
    global robot_x, robot_y, robot_yaw
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    robot_quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    robot_euler = euler_from_quaternion(robot_quaternion)
    robot_yaw = robot_euler[2]

bumper_id = 0
bumper_state = False
def bumper_callback(msg):
    global bumper_id, bumper_state
    bumper_id = msg.sensor_id
    bumper_state = msg.sensor_state
    #print(bumper_id,bumper_state )


twist = Twist()

navigation_status = "False"
def navigation_callback(status, result):
    global navigation_status
    if status == GoalStatus.SUCCEEDED:
        navigation_status = "True"
        rospy.loginfo("Navigation succeeded!")
    else:
        navigation_status = "False"
        rospy.logwarn("Navigation failed with status: " + str(status))

def navigate(x,y,th,w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = th
    goal.target_pose.pose.orientation.w = w
    rospy.loginfo("server is up")
    client.send_goal(goal,done_cb=navigation_callback)
    client.wait_for_result()


reach_docking_station_status = "False"
def auto_dock_basic():
    if (marker_x > stop_distance):
        if marker_y>=0.01:
            twist.linear.x = 0.06
            twist.angular.z = 0.05
            pub.publish(twist)
        elif marker_y<=-0.01:
            twist.linear.x = 0.06
            twist.angular.z = -0.05
            pub.publish(twist)
        else:
            twist.linear.x = 0.06
            twist.angular.z = 0.0    
            pub.publish(twist)
        rospy.sleep(0.1)

    elif marker_x <= stop_distance:
        reach_docking_station_status = "True"
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
    else:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)



def jointState_callback(data):
    global joint_angles
    joint_angles = data.position

wheel_value_180 = 6.5
def rotate():
    last_wheel_value = joint_angles[0]
    while(abs(joint_angles[0]-last_wheel_value) < wheel_value_180 and not rospy.is_shutdown()):
        twist.angular.z = 0.2
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)


dock_check = False

def last_stop():
    global dock_check, bumper_id
    bumper_id = 0
    if dock_check == False:
        while( not rospy.is_shutdown() ):
            twist.linear.x = -0.08
            twist.angular.z = 0.0
            pub.publish(twist)
            rospy.sleep(0.1)
            #print(bumper_id)
            if (bumper_id == 1 or bumper_id == 5):
                break
    rospy.sleep(0.1)


def dock():
    global reach_docking_station_status
    print("docking called")
    navigate(navigation_point[0], navigation_point[1], navigation_point[2], navigation_point[3]) 
    rospy.sleep(1)
    if marker_id1 == "in_frame" and navigation_status == "True":
        while(marker_x > stop_distance and not rospy.is_shutdown()):
            auto_dock_basic()
            if marker_x <= stop_distance:
                reach_docking_station_status = "True"
            rospy.sleep(0.1)
    rospy.sleep(1)
    if marker_x <= stop_distance and marker_id1 == "in_frame":
        reach_docking_station_status = "True"
    rospy.sleep(0.1)
    if reach_docking_station_status == "True" and marker_id1 == "in_frame":
        rotate()
        last_stop()
        reach_docking_station_status = "False"

    else:
        print("AR tag is not in the frame")

def undock():
    last_x = robot_x
    last_y = robot_y

    while(sqrt(pow((robot_x - last_x), 2) + pow((robot_y - last_y), 2)) < undock_distance and not rospy.is_shutdown()):
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

def docking_callback(data):
    message = data.data
    message_list = message.split(",")
    status = message_list[1]
    #print("Status: ", status)
    if status == "dock":
        marker_id1 = "not_in_frame"
        dock()
    elif status == "undock":
        undock()
        rospy.set_param('/start_navigation', True)

if __name__ == '__main__':
    # Initialize the ROS node with the name 'sound_direction_pan'
    rospy.init_node('dock_undock')
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/aruco_single/pose", PoseStamped, visual_callback)
    odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
    rospy.Subscriber("/cmd_frm_tablet", String, docking_callback)
    rospy.Subscriber("/measured_joint_states", JointState, jointState_callback)
    rospy.Subscriber("/bumper", bumper, bumper_callback)
    rospy.spin()