#!/usr/bin/env python3

############# Using Single ROSTOPIC #########

import rospy
from std_msgs.msg import Int16
from subprocess import Popen, call
import os
import time

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
        self.main_goto_process = None
        self.main_risky_process = None
        self.main_emotion_process = None

    def control_callback(self, msg):
        # 0: Stop, 1: Start
        if msg.data == 1:
            self.start_azure_kinect()
            time.sleep(5)
            self.start_goto()
            # self.start_main_tracker()
        
        elif msg.data == 11:
            self.start_risky()
            
        elif msg.data == 12:
            self.stop_risky()
            self.start_emotion()
               
        elif msg.data == 0:
            self.stop_azure_kinect()
            self.stop_goto()
            self.stop_emotion()
            self.stop_risky()
            # self.stop_main_tracker()
            
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

    def start_goto(self):
        if self.main_goto_process is None:
            rospy.loginfo("\nStarting Goto Location...")
            # goto_path = "/home/orin2/Gait_tracker/Goto_loc.py" #Replace with the actual path of ros_master_node.py
            goto_path = f"/home/{username}/Gait_tracker/Goto_loc.py" #Replace with the actual path of ros_master_node.py
            self.main_goto_process = Popen(["python3", goto_path])

    def stop_goto(self):
        if self.main_goto_process is not None:
            # rospy.loginfo("\n\nStopping main_ros.py...")
            # os.system("pkill -f main_ros.py")  # Adjust this based on how you start main_ros.py
            rospy.loginfo("\nStopping Goto Location...")
            self.main_goto_process.terminate()
            self.main_goto_process.wait()
            self.main_goto_process = None
            
            # os.system("pkill -f Goto_loc.py")  # Adjust this based on how you start main_ros.py
            # self.main_goto_process = None
            print("\nListening to the rostopic '/control'.... \n")
            
    def start_risky(self):
        if self.main_risky_process is None:
            rospy.loginfo("\nStarting Risky Behavior....")
            # goto_path = "/home/orin2/risky_behaviors/main.py" #Replace with the actual path of ros_master_node.py
            goto_path = f"/home/{username}/risky_behaviors/main.py" #Replace with the actual path of ros_master_node.py
            self.main_risky_process = Popen(["python3", goto_path, "video"])

    def stop_risky(self):
        if self.main_risky_process is not None:
            # rospy.loginfo("\n\nStopping main_ros.py...")
            # os.system("pkill -f main.py")  # Adjust this based on how you start main_ros.py
            rospy.loginfo("\nStopping Risky Behavior...")
            self.main_risky_process.terminate()
            # self.main_risky_process.wait()
            self.main_risky_process = None
            
            # os.system("pkill -f main.py")  # Adjust this based on how you start main_ros.py
            # self.main_risky_process = None
            # print("\nListening to the rostopic '/control'.... \n")
    
    def start_emotion(self):
        if self.main_emotion_process is None:
            rospy.loginfo("\nStarting Emotion Detection !!!!...")
            # emotion_path = "/home/orin2/Emotion/emotion_ros.py" #Replace with the actual path of ros_master_node.py
            emotion_path = f"/home/{username}/Emotion/emotion_ros.py" #Replace with the actual path of ros_master_node.py
            self.main_emotion_process = Popen(["python3", emotion_path])

    def stop_emotion(self):
        if self.main_emotion_process is not None:
            # rospy.loginfo("\n\nStopping main_ros.py...")
            # os.system("pkill -f main_ros.py")  # Adjust this based on how you start main_ros.py
            rospy.loginfo("\nStopping Emotion Detection !!!!....")
            self.main_emotion_process.terminate()
            self.main_emotion_process.wait()
            self.main_risky_process = None
            
            # os.system("pkill -f emotion_ros.py")  # Adjust this based on how you start main_ros.py
            # self.main_emotion_process = None
            # print("\nListening to the rostopic '/control'.... \n")
    
if __name__ == '__main__':
    try:
        node = GenericNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
