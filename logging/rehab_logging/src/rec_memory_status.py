#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String

def check_disk_space():
    # Initialize the ROS node
    rospy.init_node('disk_space_monitor_os_node', anonymous=True)
    
    # Create a publisher for the string topic
    disk_string_pub = rospy.Publisher('memory_status', String, queue_size=10)
    
    # Set the rate to publish every second
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Get disk usage statistics for the home directory (or any other directory you wish to monitor)
        disk_info = os.statvfs('/home')  # Change '/home' if needed
        
        # Calculate total disk space in GB
        total_disk_gb = (disk_info.f_blocks * disk_info.f_frsize) / (1024 ** 3)
        
        # Calculate available disk space in GB
        available_disk_gb = (disk_info.f_bavail * disk_info.f_frsize) / (1024 ** 3)
        
        # Format the string with both total and available disk space, separated by a comma
        disk_status_str = f"{total_disk_gb:.2f},{available_disk_gb:.2f}"
        
        # Log the disk space information
        rospy.loginfo(f"Total Disk Space: {total_disk_gb:.2f} GB, Available Disk Space: {available_disk_gb:.2f} GB")
        
        # Publish the disk space information as a string
        disk_string_pub.publish(disk_status_str)
        
        # Sleep for the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        check_disk_space()
    except rospy.ROSInterruptException:
        pass
