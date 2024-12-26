#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher_node():
    rospy.init_node('feedback_publisher', anonymous=True)
    pub = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        message = "text, hello patient, 5"
        rospy.loginfo("Publishing: %s", message)
        pub.publish(message)
        rate.sleep()
        a = input("now enter : ")

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
