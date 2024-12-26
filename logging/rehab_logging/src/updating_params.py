#!/usr/bin/env python3
#!/usr/bin/env python

import rospy
import yaml
import os
import time

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

class ParamUpdaterNode:
    def __init__(self):
        rospy.init_node('param_updater_node', anonymous=True)

        # Path to your YAML file
        # self.yaml_file_path = rospy.get_param("~yaml_file_path", "/home/orin2/test_ws/src/Rehabbot-EETeam/timely_nav/params/time_config.yaml")
        self.yaml_file_path = rospy.get_param("~yaml_file_path", f"/home/{username}/test_ws/src/Rehabbot-EETeam/timely_nav/params/time_config.yaml")

        # Initialize last_modified_time
        self.last_modified_time = 0
        
        # Load initial parameters from YAML file
        self.load_parameters_from_yaml()

        # Monitor the YAML file for changes
        self.monitor_yaml_file()

    def load_parameters_from_yaml(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                params = yaml.safe_load(file)
                # Update ROS parameters from the YAML file
                for param_name, param_value in params.items():
                    rospy.set_param(param_name, param_value)
        except IOError:
            rospy.logerr("Failed to read YAML file at {}".format(self.yaml_file_path))
    
        # Refresh parameters after updating
        rospy.get_param('goal_time_1')  # This will fetch all parameters again, forcing a refresh
        rospy.get_param('goal_time_2')
        rospy.get_param('goal_time_3')
        rospy.get_param('goal_time_4')
        rospy.get_param('goal_time_5')
        rospy.get_param('goal_time_6')
        rospy.get_param('goal_time_7')
        rospy.get_param('goal_time_8')
        rospy.get_param('goal_time_9')
        rospy.get_param('goal_time_10')
        print("\nAll Paramters Updated!!!!")

        
    def monitor_yaml_file(self):
        # Continuously monitor the YAML file for changes
        while not rospy.is_shutdown():
            if os.path.exists(self.yaml_file_path):
                # Check modification time of the YAML file
                modification_time = os.path.getmtime(self.yaml_file_path)
                if modification_time > self.last_modified_time:
                    rospy.loginfo("YAML file modified. Updating parameters...")
                    # Reload parameters from the YAML file
                    self.load_parameters_from_yaml()
                    self.last_modified_time = modification_time
            else:
                rospy.logerr("YAML file not found at {}".format(self.yaml_file_path))

            # Sleep for a short interval before checking again
            time.sleep(1)

if __name__ == '__main__':
    try:
        ParamUpdaterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
