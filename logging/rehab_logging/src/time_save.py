#!/usr/bin/env python3
################# To Update Schedule Immediately ##################
import rospy
from std_msgs.msg import String
import yaml
import os

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class RoutineListener:
    def __init__(self):
        self.goal_times = {
            'goal_time_1': "00:00",
            'goal_time_2': "00:00",
            'goal_time_3': "00:00",
            'goal_time_4': "00:00",
            'goal_time_5': "00:00",
            'goal_time_6': "00:00",
            'goal_time_7': "00:00",
            'goal_time_8': "00:00",
            'goal_time_9': "00:00",
            'goal_time_10': "00:00"
        }
        # self.yaml_file_path = '/home/orin2/test_ws/src/Rehabbot-EETeam/timely_nav/params/time_config.yaml'
        self.yaml_file_path = f'/home/{username}/rehab_ws/src/rehab_core/navigation/timely_nav/params/time_config.yaml'
        # Load existing data if the file exists
        if os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'r') as yaml_file:
                self.goal_times = yaml.safe_load(yaml_file) or self.goal_times

    def callback(self, data):
        routine_data = data.data.split(',')
        location = routine_data[0]
        hours = routine_data[1]
        minutes = routine_data[2]
        goal_time = f"{hours}:{minutes}"

        # Determine which goal_time to update based on the location
        if location == "loc1":
            self.goal_times['goal_time_1'] = goal_time
        elif location == "loc2":
            self.goal_times['goal_time_2'] = goal_time
        elif location == "loc3":
            self.goal_times['goal_time_3'] = goal_time
        elif location == "loc4":
            self.goal_times['goal_time_4'] = goal_time
        elif location == "loc5":
            self.goal_times['goal_time_5'] = goal_time
        elif location == "loc6":
            self.goal_times['goal_time_6'] = goal_time
        elif location == "loc7":
            self.goal_times['goal_time_7'] = goal_time
        elif location == "loc8":
            self.goal_times['goal_time_8'] = goal_time
        elif location == "loc9":
            self.goal_times['goal_time_9'] = goal_time
        elif location == "loc10":
            self.goal_times['goal_time_10'] = goal_time
            
        else:
            rospy.logwarn(f"Unknown location: {location}")
            return

        # Save the updated goal times to the YAML file
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(self.goal_times, yaml_file, default_flow_style=False)

        rospy.loginfo(f"Updated {location} to {goal_time} and saved to {self.yaml_file_path}")

    def listener(self):
        rospy.init_node('routine_listener', anonymous=True)
        rospy.Subscriber("/routine_frm_tablet", String, self.callback)
        rospy.spin()

if __name__ == '__main__':
    listener = RoutineListener()
    listener.listener()
