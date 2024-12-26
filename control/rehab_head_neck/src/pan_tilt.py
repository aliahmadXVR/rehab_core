#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

pan_msg = Int32MultiArray()
tilt_msg = Int32MultiArray()

def main():
    pan_pub = rospy.Publisher('pan_angles', Int32MultiArray, queue_size=1000)
    tilt_pub = rospy.Publisher('tilt_angles', Int32MultiArray, queue_size=1000)
    rospy.init_node('pan_tilt_int', anonymous=True)
    print("center:1 up:2 right:3 left:4 nod:5 no:6 both:7")
    print("----")
    while not rospy.is_shutdown():
        #a = input("enter a number= ")
        a = int(rospy.get_param("neck_cmd"))

        if a == 1: #center
            pan_msg.data = rospy.get_param("pan_center")
            tilt_msg.data = rospy.get_param("tilt_center")
            pan_pub.publish(pan_msg)
            tilt_pub.publish(tilt_msg)
            rospy.sleep(1)
            rospy.set_param('neck_cmd', 0)

        if a == 2: #look up
            pan_msg.data = rospy.get_param("pan_above")
            tilt_msg.data = rospy.get_param("tilt_above")
            pan_pub.publish(pan_msg)
            tilt_pub.publish(tilt_msg)
            rospy.sleep(1)
            rospy.set_param('neck_cmd', 0)

        if a == 3: #right
            pan_msg.data = rospy.get_param("pan_right")
            tilt_msg.data = rospy.get_param("tilt_right")
            pan_pub.publish(pan_msg)
            tilt_pub.publish(tilt_msg)
            rospy.sleep(1)
            rospy.set_param('neck_cmd', 0)

        if a == 4: #left
            pan_msg.data = rospy.get_param("pan_left")
            tilt_msg.data = rospy.get_param("tilt_left")
            pan_pub.publish(pan_msg)
            tilt_pub.publish(tilt_msg)
            rospy.sleep(1)
            rospy.set_param('neck_cmd', 0)

        if a == 5: #nod up down
            for i in range(len(rospy.get_param("pan_nod"))):
                pan_msg.data = [rospy.get_param("pan_nod")[i]]
                tilt_msg.data = [rospy.get_param("tilt_nod")[i]]
                #print(rospy.get_param("tilt_nod")[i])
                pan_pub.publish(pan_msg)
                tilt_pub.publish(tilt_msg)
                rospy.sleep(1.2)
            rospy.set_param('neck_cmd', 0)

        if a == 6: #No left right
            for i in range(len(rospy.get_param("pan_no"))):
                pan_msg.data = [rospy.get_param("pan_no")[i]]
                tilt_msg.data = [rospy.get_param("tilt_no")[i]]
                pan_pub.publish(pan_msg)
                tilt_pub.publish(tilt_msg)
                rospy.sleep(1)
            rospy.set_param('neck_cmd', 0)

        if a == 7: #both pan and tilt
            for i in range(len(rospy.get_param("pan_both"))):
                pan_msg.data = [rospy.get_param("pan_both")[i]]
                tilt_msg.data = [rospy.get_param("tilt_both")[i]]
                pan_pub.publish(pan_msg)
                tilt_pub.publish(tilt_msg)
                rospy.sleep(1.5)
            rospy.set_param('neck_cmd', 0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass