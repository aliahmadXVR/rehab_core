#!/usr/bin/env python3

############# Triggering the Time Navigation #########

import rospy
from std_msgs.msg import String
from subprocess import Popen
import os
import time

class TimelyNavNode:
    def __init__(self):
        rospy.init_node('timelynav_trigger_node', anonymous=True)
        print("\nTIMELY Navigation TRIGGER NODE RUNNING!!")
        print("\nListening to the rostopic '/cmd_frm_tablet' .... \n")

        self.control_sub = rospy.Subscriber('/cmd_frm_tablet', String, self.control_callback)

        self.timelynav_process = None

    def control_callback(self, msg):
        # Expected format: "routine, 1" or "routine, 0"
        data = msg.data.split(',')
        if len(data) == 2 and data[0].strip() == "routine":
            try:
                command = int(data[1].strip())
                if command == 1:
                    self.start_timelynav()
                elif command == 0:
                    self.stop_timelynav()
                 
            except ValueError:
                rospy.logwarn("Invalid command value in /module_status message for Navigation")
                
    def start_timelynav(self):
        if self.timelynav_process is None:
            rospy.loginfo("\nStarting Time Navigation...")
            self.timelynav_process = Popen(["roslaunch", "timely_nav", "timely_nav.launch"])

    def stop_timelynav(self):
        if self.timelynav_process is not None:
            rospy.loginfo("\nStopping Time Navigation...")
            self.timelynav_process.terminate()
            self.timelynav_process.wait()
            self.timelynav_process = None
            print("\nListening to the rostopic '/cmd_frm_tablet' .... \n")



if __name__ == '__main__':
    try:
        node = TimelyNavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
