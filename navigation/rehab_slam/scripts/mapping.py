#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

class GMappingNode:
    def __init__(self):
        rospy.init_node('gmapping_node')
        self.process = None
        rospy.Subscriber("cmd_frm_tablet", String, self.start_mapping_callback)
        rospy.loginfo("GMapping node has been initialized.")

    def start_mapping_callback(self, msg):
        string_values = msg.data.split(',')
        if string_values[1] == "startmap" and self.process is None:
            # start gmapping node
            rospy.loginfo("Starting GMapping node...")
            self.process = subprocess.Popen(["roslaunch", "rehab_slam", "rehab_gmapping.launch"])
        elif string_values[1] == "endmap" and self.process is not None:
            # stop gmapping node
            rospy.loginfo("Stopping GMapping node...")
            # save map
            rospy.loginfo("Saving map...")
            subprocess.call(["rosrun", "map_server", "map_saver", "-f", "/home/orin2/test_ws/src/Rehabbot-EETeam/rehab_navigation/maps/online_map"])
            self.process.terminate()
            self.process.wait()
            self.process = None


if __name__ == '__main__':
    gmapping_node = GMappingNode()
    rospy.spin()
