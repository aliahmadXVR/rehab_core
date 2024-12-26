#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

class AutoNav:
    def __init__(self):
        rospy.init_node('autonav_node')
        self.process = None
        rospy.Subscriber("cmd_frm_tablet", String, self.start_mapping_callback)
        rospy.loginfo("Automatic Navigation node has been initialized.")

    def start_mapping_callback(self, msg):
        string_values = msg.data.split(',')
        if string_values[1] == "hello" and self.process is None:
            rospy.loginfo("Starting Navigation node...")
            self.process = subprocess.Popen(["roslaunch", "rehab_navigation", "rehab_navigation_camera.launch"])
            # self.process = subprocess.Popen(["roslaunch", "rehab_navigation", "rehab_navigation_camera_autopose.launch"]) ##Uncomment if you want to auto localize the robot after boot
        elif string_values[1] == "hello" and self.process is not None:
            # stop gmapping node
            rospy.loginfo("Stopping Autonav node...")
            self.process.terminate()
            self.process.wait()
            self.process = None

if __name__ == '__main__':
    autonav_node = AutoNav()
    rospy.spin()
