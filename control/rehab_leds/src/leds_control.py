#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

# Function to set the corresponding bits of a binary number based on the values in the led lists
def set_bits(num, led_1, led_2, led_3):
    for i in range(3):
        num = set_bit(num, i, led_1[2-i])  # Sets bits 0, 1, 2
    for i in range(3,6):
        num = set_bit(num, i, led_2[2-(i-3)])  # Sets bits 3, 4, 5
    for i in range(6,9):
        num = set_bit(num, i, led_3[2-(i-6)])  # Sets bits 6, 7, 8
    return num

# Function to set a specific bit of a binary number to a value
def set_bit(num, bit, value):
    mask = 1 << bit
    num &= ~mask
    if value:
        num |= mask
    return num

# Function to determine the state of the LEDs based on the input parameters
def led_operation(led_1_set,led_2_set,led_3_set):
    led_1 = [1,1,1]  # Off by default
    led_2 = [1,1,1]  # Off by default
    led_3 = [1,1,1]  # Off by default

    if (led_1_set == "off"):
        led_1 = [1,1,1]
    if (led_1_set == "red"):
        led_1 = [0,1,1]
    if (led_1_set == "green"):
        led_1 = [1,0,1]

    if (led_2_set == "off"):
        led_2 = [1,1,1]
    if (led_2_set == "red"):
        led_2 = [0,1,1]
    if (led_2_set == "green"):
        led_2 = [1,0,1]
        
    if (led_3_set == "off"):
        led_3 = [1,1,1]
    if (led_3_set == "red"):
        led_3 = [0,1,1]
    if (led_3_set == "green"):
        led_3 = [1,0,1]

    return led_1, led_2, led_3

def main():
    rospy.init_node('rehab_leds')  # Initialize the ROS node
    pub = rospy.Publisher("rehab_leds", UInt16, queue_size=10)  # Create a publisher for the "rehab_leds" topic
    rate = rospy.Rate(10)  # Set the rate to 10 Hz
    num = 511  # Initialize the binary number to be published


    while not rospy.is_shutdown():
        led_1_set = rospy.get_param('led_1_set', "Of") # Get the current state of LED 1
        led_2_set = rospy.get_param('led_2_set', "Of") # Get the current state of LED 2
        led_3_set = rospy.get_param('led_3_set', "Of") # Get the current state of LED 3
        led_operation(led_1_set,led_2_set,led_3_set)  # Determine the state of each LED
        led_1, led_2, led_3 = led_operation(led_1_set,led_2_set,led_3_set) # Assign the state of each LED to a variable
        num = set_bits(num, led_1, led_2, led_3)  # Set the corresponding bits of the binary number based on LED states
        pub.publish(num)  # Publish the binary number
        #rospy.loginfo("Modified number: {:016b}".format(num))  # Log the binary number in the format of 16 bits
        #print(led_1_set,led_2_set,led_3_set) # Print the state of each LED
        rate.sleep()  # Sleep for a period of time defined by the rate variable


if __name__ == '__main__':
    main()
