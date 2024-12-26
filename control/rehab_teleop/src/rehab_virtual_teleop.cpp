#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
geometry_msgs::Twist msg;
void virtualCallback(const geometry_msgs::Twist::ConstPtr& virtual_msg)
{
  if(virtual_msg->linear.x>=0){
    msg.linear.x = virtual_msg->linear.x;
  }
  msg.angular.z = virtual_msg->angular.z;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rehab_virtual_teleop");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
  ros::Subscriber virtual_joy_sub = n.subscribe("/virtual_joy_vel", 1000, virtualCallback);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    vel_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}