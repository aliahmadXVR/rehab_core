#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

print("\nRobot Visualization on Map Node Started!")
class RobotLocationVisualizer:
    def __init__(self):
        rospy.init_node('robot_location_visualizer', anonymous=True)

        # Subscribe to the map and pose topics
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        self.image_pub = rospy.Publisher('/robot_location_image', Image, queue_size=10)
        print("\nThe resultant image is being published on the rostopic /robot_location_image")
        
        self.map_data = None
        self.pose = None
        
        self.bridge = CvBridge()
        self.map_info = None

        rospy.Timer(rospy.Duration(0.5), self.update_image)  # Update image every 0.1 seconds

    def map_callback(self, data):
        # Convert OccupancyGrid data to a numpy array
        map_data = np.array(data.data).reshape((data.info.height, data.info.width))
        map_data = np.flipud(map_data)  # Flip the map to align with the robot's coordinate frame

        # Normalize map data to 0-255 grayscale image
        map_img = np.uint8((map_data + 1) * 127.5)
        self.map_data = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
        self.map_info = data.info

    def pose_callback(self, data):
        # Extract the robot's position
        self.pose = data.pose.pose

    def update_image(self, event):
        if self.map_data is not None:
            # Create a copy of the map to draw on
            map_with_robot = self.map_data.copy()

            if self.pose and self.map_info:
                self.draw_robot_marker(map_with_robot)

            # Convert the OpenCV image to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(map_with_robot, encoding="bgr8")
            self.image_pub.publish(image_msg)

    def draw_robot_marker(self, image):
        # Calculate robot's position on the map
        x = int((self.pose.position.x - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((self.pose.position.y - self.map_info.origin.position.y) / self.map_info.resolution)
        # Adjust y-coordinate to account for the inverted axis
        y = image.shape[0] - y

        # Draw the robot's position
        cv2.circle(image, (x, y), radius=15, color=(0, 255, 255), thickness=-1)

if __name__ == '__main__':
    try:
        visualizer = RobotLocationVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
