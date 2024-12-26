#!/usr/bin/env python3

import rospy

# Initialize the ROS Node
rospy.init_node('led_modes')

# Set the initial value of "is_charging" parameter to False (if not already set)
rospy.set_param('/is_charging', False)
rospy.set_param('/is_navigation', False)
# Define the LED control function
def control_led():
    rate = rospy.Rate(1)  # 1 Hz frequency (1 second interval)
    
    while not rospy.is_shutdown():
        # Read the "is_charging" parameter
        is_charging = rospy.get_param('/is_charging')
        is_navigation = rospy.get_param('/is_navigation')
        
        # Check the value of "is_charging" and control the LED accordingly
        if is_charging:
            rospy.set_param('/led_1_set', "red")  # Turn on the LED
            rate.sleep()
            rospy.set_param('/led_1_set', "off")  # Turn off the LED
        elif is_navigation:
            rospy.set_param('/led_1_set', "green")  # Turn on the LED
            rate.sleep()
            rospy.set_param('/led_1_set', "off")  # Turn off the LED

        rate.sleep()

if __name__ == '__main__':
    try:
        control_led()
    except rospy.ROSInterruptException:
        pass
