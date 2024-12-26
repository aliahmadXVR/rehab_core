#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
import time

class small_demo:
    def __init__(self):
        rospy.init_node('head_neck_screen_demo')
        rospy.Subscriber("cmd_frm_tablet", String, self.demo_callback)
        rospy.Subscriber('/battery_info', Float32MultiArray, self.battery_info_callback)
        rospy.Subscriber('/environmental_info', Float32MultiArray, self.environmental_info_callback)
        rospy.Subscriber('/column_button', Bool, self.button_callback)
        self.pub=rospy.Publisher("/feedback_to_tablet", String, queue_size=10)
        self.battery_percentage = 0
        self.temperature = 0
        self.button_state = False
        self.button_pressed_times = []
        self.button_pressed_count = 0

    def environmental_info_callback(self, msg):
        self.temperature = msg.data[5]

    # Callback function to handle incoming messages
    def button_callback(self, msg):
        self.button_state = msg.data
        if msg.data:  # If the button is pressed (message is True)
            current_time = time.time()
            self.button_pressed_times.append(current_time)
            # Remove timestamps older than 3 seconds
            self.button_pressed_times = [t for t in self.button_pressed_times if current_time - t <= 3][-4:]
            #print(self.button_pressed_times)
            
            self.button_pressed_count = len(self.button_pressed_times)
            
            if self.button_pressed_count == 1:
                rospy.set_param('/led_1_set', "green")
                rospy.set_param('/led_2_set', "off")

            elif self.button_pressed_count == 2:
                rospy.set_param('/led_1_set', "off")
                rospy.set_param('/led_2_set', "red")
            elif self.button_pressed_count == 3:
                rospy.set_param('/led_1_set', "off")
                rospy.set_param('/led_2_set', "off")
                rospy.set_param('/neck_cmd', "2")
                screen_msg = "hello,"+"Hi How are you,10"
                self.pub.publish(screen_msg)
                rospy.sleep(8)
                screen_msg = "hello,"+"Battery is "+str(int(self.battery_percentage))+"%"+ "   Temperature is "+str(int(self.temperature))+ "°C"+",10"
                self.pub.publish(screen_msg)
                rospy.sleep(1)

    def battery_info_callback(self, msg):
        self.battery_percentage = msg.data[3]

    def demo_callback(self, msg):
        global pub
        string_values = msg.data.split(',')
        if string_values[1] == "goto":
            if string_values[4]== "5":
                #print("got text")
                screen_msg = "hello,Hello to you,10"
                rospy.set_param('/neck_cmd', "2")
                self.pub.publish(screen_msg)
                rospy.sleep(5)
                rospy.set_param('/neck_cmd', "6")
                rospy.sleep(10)
                rospy.set_param('/neck_cmd', "2")
                rospy.sleep(1)
                screen_msg = "hello,"+"Battery = "+str(int(self.battery_percentage))+"%"+ "   Temperature = "+str(int(self.temperature))+ "°C"+",10"
                self.pub.publish(screen_msg)
                rospy.sleep(1)

            if string_values[4]== "6":
                rospy.set_param('/neck_cmd', "1")
                rospy.sleep(1)

        if string_values[1] == "turnaround":
            screen_msg = "image,1821126_edit.jpg,5"
            self.pub.publish(screen_msg)
            rospy.sleep(1)


if __name__ == '__main__':
    demo = small_demo()
    rospy.spin()
