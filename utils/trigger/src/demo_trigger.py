#!/usr/bin/env python3

########## For string data ################
import rospy
from std_msgs.msg import String
from subprocess import Popen
import os

# Get the current username
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class GenericNode:
    def __init__(self):
        rospy.init_node('trigger_node', anonymous=True)
        print("\nTRIGGER NODE RUNNING!!")
        print("\nListening to the rostopic '/cmd_frm_tablet' .... \n")

        self.cmd_sub = rospy.Subscriber('/cmd_frm_tablet', String, self.cmd_callback)

        self.azure_kinect_process = None
        self.main_ros_process = None
        self.emotion_detection_process = None

    ## Start & Stop Commands functions
    def start_command(self, data_str, demo_type):
        parts = data_str.split(',')
        return len(parts) == 5 and parts[1] == demo_type

    def stop_command(self, data_str):
        parts = data_str.split(',')
        return len(parts) == 5 and parts[1] == 'exitdemo'
    
    #Callback function
    def cmd_callback(self, msg):
        data_str = msg.data
        
        if self.start_command(data_str, 'demo2'):
            rospy.loginfo("\nReceived command to start risky behavior demo: %s", data_str)
            # self.start_azure_kinect()
            self.start_main_ros()

        elif self.start_command(data_str, 'demo1'):
            rospy.loginfo("\nReceived command to start emotion detection demo: %s", data_str)
            # self.start_azure_kinect()
            self.start_emotion_detection()

        elif self.stop_command(data_str):
            rospy.loginfo("\nReceived stop command: %s", data_str)
            # self.stop_azure_kinect()
            self.stop_main_ros()
            self.stop_emotion_detection()

    # ## Start & Stop Azure Kinect ROS Driver
    # def start_azure_kinect(self):
    #     if self.azure_kinect_process is None:
    #         rospy.loginfo("\nStarting Azure Kinect ROS Driver...")
    #         self.azure_kinect_process = Popen(["roslaunch", "azure_kinect_ros_driver", "kinect_rgbd.launch"])
            
    # def stop_azure_kinect(self):
    #     if self.azure_kinect_process is not None:
    #         rospy.loginfo("\nStopping Azure Kinect ROS Driver...")
    #         self.azure_kinect_process.terminate()
    #         self.azure_kinect_process.wait()
    #         self.azure_kinect_process = None
    
    ## Start & Stop risky behavior model (main.py)
    def start_main_ros(self):
        if self.main_ros_process is None:
<<<<<<< HEAD
            rospy.loginfo("\nStarting ros_master_node.py...")
            # main_ros_path = "/home/orin2/risky_behaviors/main.py"  # Replace with the actual path of main.py
            main_ros_path = f"/home/{username}/risky_behaviors/main.py"  # Replace with the actual path of main.py
            
=======
            rospy.loginfo("\nStarting ros_master_node.py for risky behaviors...")
            main_ros_path = f"/home/{username}/risky_behaviors/main.py"  # Path to risky behavior model
>>>>>>> b947dcb3a9c044eb442b5f5f6d7845029d34de73
            self.main_ros_process = Popen(["python3", main_ros_path])
            
    def stop_main_ros(self):
        if self.main_ros_process is not None:
            rospy.loginfo("\nStopping ros_master_node.py for risky behaviors...")
            os.system("pkill -f ros_master_node.py")  # Stop the risky behavior model
            self.main_ros_process = None

    ## Start & Stop emotion detection model
    def start_emotion_detection(self):
        if self.emotion_detection_process is None:
            rospy.loginfo("\nStarting emotion detection model...")
            emotion_detection_path = f"/home/{username}/Emotion/emotion_ros.py"  # Path to emotion detection model
            self.emotion_detection_process = Popen(["python3", emotion_detection_path])

    def stop_emotion_detection(self):
        if self.emotion_detection_process is not None:
            rospy.loginfo("\nStopping emotion detection model...")
            os.system("pkill -f emotion_detection.py")  # Stop the emotion detection model
            self.emotion_detection_process = None
            print("\nListening to the rostopic '/cmd_frm_tablet'.... \n")

if __name__ == '__main__':
    try:
        node = GenericNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
