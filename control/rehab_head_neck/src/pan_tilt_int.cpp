/**
 * @file pan_int.cpp
 * @author Ali Ahmad ali.ahmad@xavor.com)
 * @brief 
 * This file is used to create a two way communication with PAN/TILT
 * mechanism.
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include <sstream>


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pan_tilt_int");
  ros::NodeHandle n;

  ros::Publisher pan_pub = n.advertise<std_msgs::Int32MultiArray>("pan_angles", 1000);
  ros::Publisher tilt_pub = n.advertise<std_msgs::Int32MultiArray>("tilt_angles", 1000);


  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    
    std_msgs::Int32MultiArray pan_msg;
    std_msgs::Int32MultiArray tilt_msg;

    // pan_msg.data = {45,-45,0};
    // tilt_msg.data = {30,0,30,0};
 

       

    int value;;
    std::cout<<"Enter Value: "<<std::endl;
    std::cin>> value;

    if (value == 1)   //look up
    {
      pan_msg.data = {90};
      tilt_msg.data = {30};
      pan_pub.publish(pan_msg);
      tilt_pub.publish(tilt_msg);
      ros::spinOnce();
    }
    else if( value == 2)  //look down
    {
         pan_msg.data = {0};
      tilt_msg.data = {0};
      pan_pub.publish(pan_msg);
      tilt_pub.publish(tilt_msg);
      ros::spinOnce();
    }

    
    loop_rate.sleep();
  }


  return 0;
}