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
        # Load the YAML file
        # with open('/home/orin2/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml', 'r') as file:
        with open(f'/home/{username}/test_ws/src/Rehabbot-EETeam/rehab_behavior/params/location.yaml', 'r') as file:
            data = yaml.safe_load(file)

        # Extracting parameters from location1
        location1 = data['location1']
        orientation = location1['orientation']
        position = location1['position']
        # Extracting x, y, w, and z parameters
        x1 = position['x']
        y1 = position['y']
        w1 = orientation['w']
        z1 = orientation['z']

         # Extracting parameters from location2
        location2 = data['location2']
        orientation = location2['orientation']
        position = location2['position']
        # Extracting x, y, w, and z parameters
        x2 = position['x']
        y2 = position['y']
        w2 = orientation['w']
        z2 = orientation['z']
        
         # Extracting parameters from location3
        location3 = data['location3']
        orientation = location3['orientation']
        position = location3['position']
        # Extracting x, y, w, and z parameters
        x3 = position['x']
        y3 = position['y']
        w3 = orientation['w']
        z3 = orientation['z']
        
         # Extracting parameters from location4
        location4 = data['location4']
        orientation = location4['orientation']
        position = location4['position']
        # Extracting x, y, w, and z parameters
        x4 = position['x']
        y4 = position['y']
        w4 = orientation['w']
        z4 = orientation['z']
        
         # Extracting parameters from location5
        location5 = data['location5']
        orientation = location5['orientation']
        position = location5['position']
        # Extracting x, y, w, and z parameters
        x5 = position['x']
        y5 = position['y']
        w5 = orientation['w']
        z5 = orientation['z']
        
         # Extracting parameters from location6
        location6 = data['location6']
        orientation = location6['orientation']
        position = location6['position']
        # Extracting x, y, w, and z parameters
        x6 = position['x']
        y6 = position['y']
        w6 = orientation['w']
        z6 = orientation['z']
        ##########################################################################
        
        # Define the locations with orientation
        self.locations = {
                 
            # 1: {'position': (4.863711619076721, 2.8375814821913203), 'orientation': (0.0, 0.0, 0.9063072895452509, 0.4226193286128082)},  # Define x2, y2 and orientation for location 2
            # 2: {'position': (11.473990367207294, 2.223867809811212), 'orientation': (0.0, 0.0, 0.14493354743610948, 0.9894413913050055)},  # Define x3, y3 and orientation for location 3
            # 3: {'position': (15.840418593434737, 2.693405150020472), 'orientation': (0.0, 0.0, 0.9998049555140824, 0.01974970707336338)},  # Define x3, y3 and orientation for location 3
            # 4: {'position': (2.5343358820095943, 1.4465983575765236), 'orientation': (0.0, 0.0, -0.3443310478982513, 0.9388482994889494)},  # Define x4, y4 and orientation for location 4
            # 5: {'position': (0.5665131604442297, -0.047279239677015354), 'orientation': (0.0, 0.0, 0.1394373346915623, 0.9902308971619261)},   # Define x6, y6 and orientation for location 6            
            # 6: {'position': (8.477294587434654, 2.692162373425871), 'orientation': (0.0, 0.0, 0.9963935732162642, 0.08485191366919828)}   # Define x6, y6 and orientation for location 6
            
            1: {'position': (x1, y1), 'orientation': (0.0, 0.0, z1, w1)},  
            2: {'position': (x2, y2), 'orientation': (0.0, 0.0, z2, w2)}, 
            3: {'position': (x3, y3), 'orientation': (0.0, 0.0, z3, w3)},  
            4: {'position': (x4, y4), 'orientation': (0.0, 0.0, z4, w4)},
            5: {'position': (x5, y5), 'orientation': (0.0, 0.0, z5, w5)},  
            6: {'position': (x6, y6), 'orientation': (0.0, 0.0, z6, w6)},
            
        ##### Uncomment to following lines for time scheduling in this same node ############
        # # #Define the schedule
        # self.schedule = {
        #     "15:06": 1,
        #     "15:05": 2,
        #     # "01:00": 3
        #     # Add more entries for other times and locations as needed
       
        }

        # # Timer for scheduled navigation
        # self.timer = rospy.Timer(rospy.Duration(60), self.scheduled_navigation_callback)  # Check every minute
        ####################################################################################
        
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
                    
                    loc_text = String()
                    
                    loc_text.data = "text,Command Recieved!,5"
                    loc_pub.publish(loc_text)
                    time.sleep(3)
                    
                    loc_text.data = "text,Navigating!,5"
                    loc_pub.publish(loc_text)
                    
                    rospy.loginfo("Sending robot to location {} based on command from tablet.".format(location_str))
                    self.send_goal_to_move_base(location)
                    break
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

### Uncomment the following function if time scheduling is done in this same node #####
################ Schedule based Navigation ###########            
    # def scheduled_navigation_callback(self, event):
    #     # current_time = datetime.now()
    #     # current_time_str = current_time.strftime("%H:%M")
    #     current_time = datetime.now().strftime("%H:%M")
    #     print("\nThe Current Time is: ", current_time)
    #     if current_time in self.schedule:
    #         location = self.schedule[current_time]
    #         rospy.loginfo("Scheduled time reached: {}. Sending robot to location {}.".format(current_time, location))
    #         self.send_goal_to_move_base(location)
########################################################################################
       
if __name__ == '__main__':
    try:
        GoToLocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
