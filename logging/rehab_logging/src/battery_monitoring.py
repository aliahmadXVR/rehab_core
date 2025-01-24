#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32MultiArray
import time

battery_display_pub = rospy.Publisher('/battery_display', String, queue_size=10)
feedback_to_tablet_pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)
dock_pub = rospy.Publisher('/cmd_frm_tablet', String, queue_size=10)

class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor_node')
        print("Battery Monitoring Node Started!!!!")
        self.battery_info_sub = rospy.Subscriber('/battery_info', Float32MultiArray, self.battery_info_callback)
        
        self.last_warning_time = time.time()
        
        self.dock_message_published = False  # Flag to track dock message state
        
        battery_percentage = 100.0  # Default value
        self.rate = rospy.Rate(1)  # 1 Hz
        
        # self.run()

    def battery_info_callback(self, msg):
        if len(msg.data) > 0:
            battery_percentage = msg.data[3]  ## Take the value from index 3
            
            battery_percentage = int(battery_percentage) ## To remove the decimal
            
            print("\nThe Current Battery is: ", battery_percentage)
            rospy.loginfo(f"Received battery percentage: {battery_percentage}")
            
    # def run(self):
        # while not rospy.is_shutdown():
            battery_display_msg = String()        
            # battery_display_msg.data = f"{battery_percentage}%"
            battery_display_msg.data = f"{battery_percentage}"
            print("\nBattery Percentage: ", battery_percentage)
            
            battery_display_pub.publish(battery_display_msg)
            
            rospy.loginfo(f"Published to /battery_display: {battery_display_msg.data}")
            
        if battery_percentage < 30.0:
            current_time = time.time()
            if current_time - self.last_warning_time >= 180: 
                warning_msg = String()
                warning_msg.data = "text,Battery is less than 30%,5"
                print("\nBattery Low!")
                feedback_to_tablet_pub.publish(warning_msg)
                rospy.loginfo(f"Published warning to /feedback_to_tablet: {warning_msg.data}")
                self.last_warning_time = current_time 
             
        ####################### For publishing Auto-Docking Message #####################################
        # Comment the following lines if you want to turn off the auto-docking based on battery percentage
        if battery_percentage < 20.0:
            if not self.dock_message_published:  # Check if the dock message has already been published
                # time.sleep(3)
                dock_msg = String()
                dock_msg.data = "time stamp,dock,12,0,0,5"
                dock_pub.publish(dock_msg)
                rospy.loginfo(f"Published auto-docking message to /cmd_frm_tablet: {dock_msg.data}")
                self.dock_message_published = True  # Set the flag to true after publishing   
                
                time.sleep(2) 
                # print("\nBattery Critically Low! Going to dock")
                warn_msg = String()
                warn_msg.data = "text,Battery critically low! Auto-docking,5"
                feedback_to_tablet_pub.publish(warn_msg)
                
        elif battery_percentage > 20.0:
                # Reset the flag once the battery is above 10%
                self.dock_message_published = False
        ###################################################################################################
        
        if battery_percentage > 85.0:
            current_time = time.time()
            if current_time - self.last_warning_time >= 180: 
                warning_msg = String()
                warning_msg.data = "text,Battery Full! unplug the charger,5s"
                print("\nBattery Full!")
                feedback_to_tablet_pub.publish(warning_msg)
                rospy.loginfo(f"Published warning to /feedback_to_tablet: {warning_msg.data}")
                self.last_warning_time = current_time
        ##################
        
        # self.rate.sleep()

if __name__ == '__main__':
    try:
        BatteryMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
