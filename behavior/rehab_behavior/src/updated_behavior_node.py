#!/usr/bin/env python

############ For Multiple Locations with Orientation #################
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import datetime
from datetime import datetime, timedelta
import time
import yaml

# Get the current username
import os
import getpass
username = getpass.getuser()
print("USERNAME:", username)

#### To Publish the location Text
loc_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)

# desired_time = datetime.now().replace(hour=12, minute=15, second=0, microsecond=0)
# print("\n\nDesired Time = ",desired_time)
print("\n\nRehab Behavior Node Started!!!!")
class GoToLocation:
    def __init__(self):
        rospy.init_node('go_to_location_node', anonymous=True)
        
        # Subscribe to the /cmd_frm_tablet topic
        rospy.Subscriber('/cmd_frm_tablet', String, self.cmd_from_tablet_callback)
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server found")
        
        ##########################################################################     
        with open(f'/home/{username}/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml', 'r') as file:
            data = yaml.safe_load(file)

        total_locations = 11 ## For 10 locations (For I locations write i+1)
        # Initialize lists to store the extracted values
        x_values = []
        y_values = []
        w_values = []
        z_values = []
        
        # Initialize a dictionary to store the extracted values, keyed by location number
        location_data = {}
                
        # Loop through locations 1 to 10
        for i in range(1, total_locations):
            location = data.get(f'location{i}')  # Use get() to avoid KeyError
            if location is None:
                print(f'location{i} not found, skipping.')
                location_data[i] = None  # Leave a blank entry for the missing location
                continue  # Skip this iteration if location is not found
            
            orientation = location['orientation']
            position = location['position']
            
             # Store the values in the dictionary with location index
            location_data[i] = {
                'position': (position['x'], position['y']),
                'orientation': (0.0, 0.0, orientation['z'], orientation['w'])
            }
            
            # Append values to the respective lists
            x_values.append(position['x'])
            y_values.append(position['y'])
            w_values.append(orientation['w'])
            z_values.append(orientation['z'])

        # Now you can access the values as x_values[0] for location1, x_values[1] for location2, and so on
        print("\nX Values:", x_values)
        print("\nY Values:", y_values)
        print("\nZ Values:", z_values)
        print("\nW Values:", w_values)
    
        ##########################################################################
        self.locations = {}
        # Loop through the locations and assign position and orisentation dynamically    
            
        for i in range(1, total_locations):
            if location_data[i] is None:
                print(f'\nLocation {i} is missing or was deleted.\n')
                # Optional: Leave a blank space or assign some placeholder if needed
                self.locations[i] = {'position': None, 'orientation': None}
            else:
                self.locations[i] = location_data[i]
                print(f"\nLocation",i,":", self.locations[i])  # {'position': (x1, y1), 'orientation': (0.0, 0.0, z1, w1)}
        
        # Example of accessing location data
        print(self.locations)  
        ######################################################

    def cmd_from_tablet_callback(self, data):
        cmd_str = data.data
        rospy.loginfo("\nReceived command from tablet: {}".format(cmd_str))
        # Split the received string by comma to extract information
        parts = cmd_str.split(',')
        if len(parts) >= 4 and parts[1] == 'goto':
            location_str = parts[3]
            print("\nLocation no = ",location_str)
            # Find the location index from the string
            for location, info in self.locations.items():
                # if str(location) == location_str:
                if str(location) == location_str.strip('loc'): 
                    
                    #### Check if the Location has data None ####
                    if self.locations[location]['position'] is None:
                        rospy.loginfo(f"Location {location} not tagged.")
                        print("Location!!: ", location)
                        loc_text = String()
                        loc_text.data = "text,Location Not Tagged,5"
                        loc_pub.publish(loc_text.data)  # Publish feedback message
                        return
                    
                    #### If not None, then send to the desired Location ####
                    else:
                        loc_text = String()
                        loc_text.data = "text,Command Recieved,5"
                        loc_pub.publish(loc_text)
                        time.sleep(5)
                        
                        loc_text.data = "text,Navigating,5"
                        loc_pub.publish(loc_text)
                        
                        rospy.loginfo("Sending robot to location {} based on command from tablet.".format(location_str))
                        self.send_goal_to_move_base(location)  
                    break
                    ######  
            else:
                rospy.loginfo("Location {} not found in the predefined locations.".format(location_str))
    
    ############## Sending the loc to Move base ############################
    def send_goal_to_move_base(self, location):
        position = self.locations[location]['position']
        orientation = self.locations[location]['orientation']    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation = Quaternion(*orientation)

        rospy.loginfo("Sending goal to move_base: x={}, y={}, orientation={}".format(position[0], position[1], orientation))
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
    ########################################################################
       
if __name__ == '__main__':
    try:
        GoToLocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
