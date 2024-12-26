#!/usr/bin/env python3

############# Using Single ROSTOPIC #########

import rospy
from std_msgs.msg import Int16
from subprocess import Popen, call
import os

# Get the current username
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class GenericNode:
    def __init__(self):
        rospy.init_node('trigger_node', anonymous=True)
        print("\nTRIGGER NODE RUNNING!!")
        print("\nListening to the rostopic '/control' .... \n")

        self.control_sub = rospy.Subscriber('/control', Int16, self.control_callback)

        # self.display_process = None
        self.azure_kinect_process = None
        self.main_ros_process = None

    def control_callback(self, msg):
        # 0: Stop, 1: Start
        if msg.data == 1:
            
            self.start_azure_kinect()
            self.start_main_ros()
        elif msg.data == 0:
            self.stop_azure_kinect()
            self.stop_main_ros()
            
    def start_azure_kinect(self):
        if self.azure_kinect_process is None:
            rospy.loginfo("\nStarting Azure Kinect ROS Driver...")
            self.azure_kinect_process = Popen(["roslaunch", "azure_kinect_ros_driver", "kinect_rgbd.launch"])

    def stop_azure_kinect(self):
        if self.azure_kinect_process is not None:
            rospy.loginfo("\nStopping Azure Kinect ROS Driver...")
            self.azure_kinect_process.terminate()
            self.azure_kinect_process.wait()
            self.azure_kinect_process = None

    def start_main_ros(self):
        if self.main_ros_process is None:
<<<<<<< HEAD
            rospy.loginfo("\nStarting main_ros.py...")
            main_ros_path = f"/home/{username}/trt_pose_gait/main_ros.py" #Replace with the actual path of ros_master_node.py
            
            # rospy.loginfo("\nStarting ros_master_node.py...")
            # main_ros_path = f"/home/{username}/trt_pose_gait/ros_master_node.py" #Replace with the actual path of ros_master_node.py
            
=======
            # rospy.loginfo("\nStarting main_ros.py...")
            # main_ros_path = "/home/orin2/gait_analysis/main_ros.py"  # Replace with the actual path of main_ros.py
            rospy.loginfo("\nStarting ros_master_node.py...")
            # main_ros_path = "/home/orin2/master_node/ros_master_node.py" #Replace with the actual path of ros_master_node.py
            main_ros_path = f"/home/{username}/master_node/ros_master_node.py" #Replace with the actual path of ros_master_node.py
>>>>>>> b947dcb3a9c044eb442b5f5f6d7845029d34de73
            self.main_ros_process = Popen(["python3", main_ros_path])

    def stop_main_ros(self):
        if self.main_ros_process is not None:
            rospy.loginfo("\n\nStopping main_ros.py...")
            os.system("pkill -f main_ros.py")  # Adjust this based on how you start main_ros.py
            
            # rospy.loginfo("\nStopping ros_master_node.py...")
            # os.system("pkill -f ros_master_node.py")  # Adjust this based on how you start main_ros.py
            
            self.main_ros_process = None
            print("\nListening to the rostopic '/control'.... \n")

if __name__ == '__main__':
    try:
        node = GenericNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
