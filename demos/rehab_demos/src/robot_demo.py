#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
import time

#Trigger this demo when you get data on /cmd_frm_tablet
# time stamp,demo4,24,0,0

class small_demo:
    def __init__(self):
        rospy.init_node('robot_demo')
        rospy.Subscriber('/battery_info', Float32MultiArray, self.battery_info_callback)
        rospy.Subscriber('/environmental_info', Float32MultiArray, self.environmental_info_callback)
        rospy.Subscriber('/column_button', Bool, self.button_callback)
        rospy.Subscriber("cmd_frm_tablet", String, self.demo_trig_callback)
        self.pub=rospy.Publisher("/feedback_to_tablet", String, queue_size=10)
        self.battery_percentage = 0
        self.temperature = 25
        self.button_state = False
        self.demo_flag = False
        self.demo_trig = False

    def demo_trig_callback(self, msg):
        string_values = msg.data.split(',')
        if string_values[1] == "demo4":
            self.demo_trig = True
            print("demo msg recieved")


    def environmental_info_callback(self, msg):
        self.temperature = msg.data[5]

    # Callback function to handle incoming messages
    def button_callback(self, msg):
        self.button_state = msg.data

    def speak(self,msg):
        screen_msg = msg
        self.pub.publish("hello,"+ screen_msg+",5")
        #rospy.sleep(14) # screen time + 9

    def demo(self):
        if self.demo_trig == True and self.demo_flag == False:
            self.demo_flag = True
            self.demo_trig = False
            print("demo started")

            self.speak("Welcome to the Technical Demo")
            rospy.sleep(5)

            self.speak("I have Moveable Head & Neck for Interractions")
            rospy.sleep(5)
            rospy.set_param('/neck_cmd', "7")
            rospy.sleep(6)

            self.speak("Similarly I have indication LEDs")
            rospy.sleep(5)
            self.speak("These can be used to indicate something")
            rospy.sleep(5)

            rospy.set_param('/led_2_set', "green")
            rospy.set_param('/led_1_set', "green")
            rospy.sleep(3)
            rospy.set_param('/led_1_set', "off")
            rospy.set_param('/led_2_set', "off")
            rospy.sleep(3)
            rospy.set_param('/led_1_set', "red")
            rospy.set_param('/led_2_set', "red")
            rospy.sleep(3)      

            self.speak("Moreover I have battery Monitoring")
            rospy.sleep(5)
            self.speak("which can trigger automatic docking")
            rospy.sleep(5)

            screen_msg = "Currently Battery is "+str(int(self.battery_percentage)) + "%" + ",10"

            self.speak(screen_msg)
            rospy.sleep(5) ##

            self.speak("I can also monitor temperature & gases")
            rospy.sleep(5)
            screen_msg = "Currently the ambient temperature is "+str(int(self.temperature))+ "°C"+",10"
            self.speak(screen_msg)
            rospy.sleep(5)

            self.speak("Coming towards some more capabilities")
            rospy.sleep(5)

            self.speak("I can pull some image or video from cloud storage")
            rospy.sleep(5)

            self.speak("For example person A want to see family picture when agitated ")
            rospy.sleep(5)

            # screen_msg = "image,1821126_edit.jpg,5"
            screen_msg = "image,persona_a.jpg,5"
            self.pub.publish(screen_msg)
            rospy.sleep(5)


            self.speak("Similarly person B calms down after watching landscapes ")
            rospy.sleep(5)

            screen_msg = "image,persona_b.jpg,5"
            self.pub.publish(screen_msg)
            rospy.sleep(5)

            # screen_msg = "video,exercise.mp4,45"
            # self.pub.publish(screen_msg)
            # rospy.sleep(60)

            self.speak("Thats all for the technical demo for now")
            rospy.sleep(5)
            self.speak("Some more features are under development")
            rospy.sleep(5)

            self.speak("Thank you!")
            rospy.sleep(2)

            self.demo_flag = False # ready for another demo
            print("ready for new demo")
            print("------------------")

    def battery_info_callback(self, msg):
        self.battery_percentage = msg.data[3]


demo_node = small_demo()
while not rospy.is_shutdown():
    demo_node.demo()
    rospy.sleep(0.1)
