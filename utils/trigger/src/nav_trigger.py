#!/usr/bin/env python3

############# Triggering the Navigation #########

import rospy
from std_msgs.msg import String
from subprocess import Popen
import os
import time

class GenericNode:
    def __init__(self):
        rospy.init_node('nav_trigger_node', anonymous=True)
        print("\nNavigation TRIGGER NODE RUNNING!!")
        print("\nListening to the rostopic '/module_status' .... \n")

        self.control_sub = rospy.Subscriber('/module_status', String, self.control_callback)

        self.navigation_process = None
        self.camera_process = None

    def control_callback(self, msg):
        # Expected format: "NavMode, 1" or "NavMode, 0"
        data = msg.data.split(',')
        if len(data) == 2 and data[0].strip() == "NavMode":
            try:
                command = int(data[1].strip())
                if command == 1:
                    self.start_camera()
                    time.sleep(3)
                    self.start_navigation()
                elif command == 0:
                    self.stop_navigation()
                    self.stop_camera()
                    
            except ValueError:
                rospy.logwarn("Invalid command value in /module_status message for Navigation")
                
    def start_camera(self):
        if self.camera_process is None:
            rospy.loginfo("\nStarting Navigation...")
            self.camera_process = Popen(["roslaunch", "realsense2_camera", "rs_rgbd.launch"])

    def stop_camera(self):
        if self.camera_process is not None:
            rospy.loginfo("\nStopping Navigation...")
            self.camera_process.terminate()
            self.camera_process.wait()
            self.camera_process = None
            print("\nListening to the rostopic '/module_status' .... \n")

    def start_navigation(self):
        if self.navigation_process is None:
            rospy.loginfo("\nStarting Navigation...")
            self.navigation_process = Popen(["roslaunch", "rehab_navigation", "rehab_navigation_camera.launch"])
            # self.navigation_process = Popen(["roslaunch", "rehab_navigation", "rehab_navigation_camera_autopose.launch"]) ## Uncomment for autopose localization

    def stop_navigation(self):
        if self.navigation_process is not None:
            rospy.loginfo("\nStopping Navigation...")
            self.navigation_process.terminate()
            self.navigation_process.wait()
            self.navigation_process = None
            # print("\nListening to the rostopic '/module_status' .... \n")

if __name__ == '__main__':
    try:
        node = GenericNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
