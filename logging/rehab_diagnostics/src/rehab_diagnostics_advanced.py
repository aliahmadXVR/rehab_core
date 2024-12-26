#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from nav_msgs.msg import Odometry
from rehab_msgs.msg import bumper
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist

twist =Twist()

class DiagnosticNode:
    def __init__(self):
        rospy.init_node('diagnostic_node')
        
        self.lidar_status = "Lidar not detected"
        self.camera_status = "Camera not detected"
        self.depth_status = "Depth camera not detected"
        self.odometry_status = "Odometry not detected"
        self.bumper_status = "Bumper not detected"
        
        self.lidar_received = False
        self.camera_received = False
        self.depth_received = False
        self.odometry_received = False
        self.bumper_received = False
        self.rotation = 0
        
        self.diag_trig = False
        
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.depth_callback)
        rospy.Subscriber("/bumper", bumper, self.bumper_callback)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.odom_callback)
        rospy.Subscriber("cmd_frm_tablet", String, self.diag_trig_callback)
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher("/feedback_to_tablet", String, queue_size=10)
        
        self.run_diagnostics()
    
    def scan_callback(self, msg):
        self.lidar_status = "Lidar is working"
        self.lidar_received = True
    
    def image_callback(self, msg):
        self.camera_status = "Camera is working"
        self.camera_received = True
    
    def depth_callback(self, msg):
        self.depth_status = "Depth camera is working"
        self.depth_received = True
    
    def odom_callback(self, msg):
        # self.odometry_status = "Odometry is working"
        # self.odometry_received = True
        self.rotation = msg.pose.pose.orientation.z
        
    def odom_status(self):
        previous_z = round(self.rotation,2)
        for i in range(20):
            twist.angular.z = 0.2
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        if round(self.rotation,2)!=previous_z:
            self.odometry_status = "Odometry and Cmd_vel are working"
            self.odometry_received = True
        for i in range(20):
            twist.angular.z = -0.2
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
            twist.angular.z = 0
            self.cmd_pub.publish(twist)

    def bumper_callback(self, msg):
        self.bumper_status = "Bumper is working"
        self.bumper_received = True
    
    def diag_trig_callback(self, msg):
        string_values = msg.data.split(',')
        if string_values[1] == "diagnostics":
            self.diag_trig = True
            print("diagnostics started")

    def speak(self, msg):
        screen_msg = "hello," + msg + ",5"
        self.pub.publish(screen_msg)
    
    def run_diagnostics(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.diag_trig:
                self.speak(self.lidar_status)
                rospy.loginfo(self.lidar_status)
                rospy.sleep(5)

                self.speak(self.camera_status)
                rospy.loginfo(self.camera_status)
                rospy.sleep(5)

                self.speak(self.depth_status)
                rospy.loginfo(self.depth_status)
                rospy.sleep(5)

                self.odom_status()
                self.speak(self.odometry_status)
                rospy.loginfo(self.odometry_status)
                rospy.sleep(5)

                if not self.bumper_received:
                    self.speak("press bumper with in 5 sec")
                    rospy.loginfo("press bumper with in 5 sec")
                    rospy.sleep(5)
                self.speak(self.bumper_status)
                rospy.loginfo(self.bumper_status)
                rospy.sleep(5)

                self.speak("Testing The Head & Neck")
                rospy.loginfo("Testing The Head & Neck")
                rospy.set_param('neck_cmd', 7)


                self.diag_trig = False
                self.reset_status()
                print("finished")
                    
            rate.sleep()
    
    def reset_status(self):
        self.lidar_status = "Lidar not detected"
        self.camera_status = "Camera not detected"
        self.depth_status = "Depth camera not detected"
        self.odometry_status = "Odometry not detected"
        self.bumper_status = "Bumper not detected"
        self.cmd_status = "cmd not working"
        
        self.lidar_received = False
        self.camera_received = False
        self.depth_received = False
        self.odometry_received = False
        self.bumper_received = False
        self.rotation = 0

if __name__ == '__main__':
    try:
        DiagnosticNode()
    except rospy.ROSInterruptException:
        pass
