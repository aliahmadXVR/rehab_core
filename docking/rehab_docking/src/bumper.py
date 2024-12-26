#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from rehab_msgs.msg import bumper
from rehab_msgs.msg import WheelsCmdStamped


twist = Twist()
wheels_cmd_msg = WheelsCmdStamped()
bumper_id = 0
bumper_state = False

left_bumper = 1
left_bumper_state = False

right_bumper = 5
right_bumper_state = False

dock_check = False

def last_stop():
    global left_bumper, left_bumper_state, right_bumper, right_bumper_state, dock_check
    if bumper_id == 0:
        twist.linear.x = -0.1
        twist.angular.z = 0.0
        print("first_state")
        pub.publish(twist)

    elif(bumper_id == left_bumper and bumper_id == right_bumper and dock_check == False):
        print("both pressed")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        dock_check = True


    #left back bumper
    elif(bumper_id == left_bumper and dock_check == False):
        bumper_state = left_bumper_state
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        print("left_state")
        while( bumper_id != right_bumper and not rospy.is_shutdown() ):
            wheels_cmd_msg.wheels_cmd.angular_velocities.joint = [0.0, -0.6]
            single_pub.publish(wheels_cmd_msg)
            rospy.sleep(0.1)
        wheels_cmd_msg.wheels_cmd.angular_velocities.joint = [0.0, 0.0]
        single_pub.publish(wheels_cmd_msg)
        dock_check = True
    
    #right back bumper
    elif(bumper_id == right_bumper and dock_check == False):
        bumper_state = right_bumper_state
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        print("right_state")
        while( bumper_id != left_bumper and not rospy.is_shutdown() ):
            wheels_cmd_msg.wheels_cmd.angular_velocities.joint = [-0.6, 0.0]
            single_pub.publish(wheels_cmd_msg)
            rospy.sleep(0.1)
        wheels_cmd_msg.wheels_cmd.angular_velocities.joint = [0.0, 0.0]
        single_pub.publish(wheels_cmd_msg)
        dock_check = True

    rospy.sleep(0.1)




def bumper_callback(msg):
    global bumper_id, bumper_state
    bumper_id = msg.sensor_id
    bumper_state = msg.sensor_state
    print(bumper_id,bumper_state )


# Initialize the ROS node with the name 'sound_direction_pan'
rospy.init_node('bumper_test')
pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
rospy.Subscriber("/bumper", bumper, bumper_callback)
single_pub = rospy.Publisher('/wheel_cmd_velocities', WheelsCmdStamped, queue_size=10)

while not rospy.is_shutdown():
     last_stop()
     rospy.sleep(0.1)