###### Estimating the Robot Localization based on AMCL particles #######

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# Callback function to process incoming particle data
def particlecloud_callback(msg):
    particles = []
    weights = []
    
    # Extract positions and orientations from the PoseArray message
    for pose in msg.poses:
        x = pose.position.x
        y = pose.position.y
        # Assuming theta can be derived from the quaternion
        q = pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        weight = 1.0 / len(msg.poses)  # Assuming equal weights for simplicity
        particles.append([x, y, theta])
        weights.append(weight)
    
    particles = np.array(particles)
    weights = np.array(weights)
    
    # Compute weighted mean
    mean_position = np.average(particles, axis=0, weights=weights)

    # Compute covariance matrix
    cov_matrix = np.cov(particles.T, aweights=weights)

    # Extract variances
    var_x = cov_matrix[0, 0]
    var_y = cov_matrix[1, 1]
    var_theta = cov_matrix[2, 2]

    # Define thresholds for variances
    threshold_x = 0.1
    threshold_y = 0.1
    threshold_theta = 0.05

    # Determine if the robot is localized
    localized = var_x < threshold_x and var_y < threshold_y and var_theta < threshold_theta

    rospy.loginfo(f"Mean Position: {mean_position}")
    rospy.loginfo(f"Covariance Matrix: \n{cov_matrix}")
    rospy.loginfo(f"Localized: {localized}")

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def main():
    rospy.init_node('particle_cloud_listener', anonymous=True)
    rospy.Subscriber("/particlecloud", PoseArray, particlecloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
