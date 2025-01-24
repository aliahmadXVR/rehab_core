#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from subprocess import Popen, PIPE, call

# Get the current username
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class DemoTriggerNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('demo_trigger_node', anonymous=True)
        rospy.loginfo("Demo Trigger Node is Running!")

        # Subscribe to the /demo_control topic
        self.control_sub = rospy.Subscriber('/demo_control', Int32, self.control_callback)

        # To track the process running the .sh script
        self.demo_process = None

    # Callback function to handle the data from /demo_control topic
    def control_callback(self, msg):
        demo_control_value = msg.data
        rospy.loginfo("Received data on /demo_control: %d", demo_control_value)

        # If data is 1, start the .sh script
        if demo_control_value == 1:
            self.start_demo_script()
            
        # If data is 0, stop the .sh script
        elif demo_control_value == 0:
            self.stop_demo_script()

    # Function to start the .sh script
    def start_demo_script(self):
        if self.demo_process is None:
            rospy.loginfo("Starting demo.sh script...")
            # Replace with the actual path to your .sh script
            # script_path = f"/home/{username}/demo_models.sh"
            script_path = f"/home/{username}/rehab_ws/src/rehab_core/utils/trigger/scripts/demo_models.sh"
            self.demo_process = Popen([script_path])
        else:
            rospy.logwarn("demo.sh script is already running.")
            
   # Function to stop the .sh script (terminate tmux session)
    def stop_demo_script(self):
        rospy.loginfo("Stopping tmux session for demo.sh script...")
        # Kill the tmux session (adjust session name if different)
        session_name = "adl_session"
        result = call(["tmux", "kill-session", "-t", session_name])
        if result == 0:
            rospy.loginfo("tmux session %s stopped successfully.", session_name)
        else:
            rospy.logwarn("Failed to stop tmux session %s. It may not be running.", session_name)

        # Clear the demo_process variable
        self.demo_process = None
        
        

if __name__ == '__main__':
    try:
        # Create the node
        node = DemoTriggerNode()
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
