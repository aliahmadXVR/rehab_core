
#ifndef _REHAB_BUMPER2CLOUD_HPP_
#define _REHAB_BUMPER2CLOUD_HPP_
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <rehab_msgs/SensorState.h>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace rehab_bumper2cloud
{

class Bumper2PcNodelet : public nodelet::Nodelet
{
public:
  Bumper2PcNodelet()
    : P_INF_X(+100*sin(0.34906585)),
      P_INF_Y(+100*cos(0.34906585)),
      N_INF_Y(-100*cos(0.34906585)),
      ZERO(0), prev_bumper(0), prev_cliff(0) { }
  ~Bumper2PcNodelet() { }

  void onInit();

private:
  const float P_INF_X; 
  const float P_INF_Y;  
  const float N_INF_Y; 
  const float ZERO;

  uint8_t prev_bumper;
  uint8_t prev_cliff;

  float pc_radius_;
  float pc_height_;
  float p_side_x_;
  float p_side_y_;
  float n_side_y_;

  ros::Publisher  pointcloud_pub_;
  ros::Publisher  status_pub_;
  ros::Subscriber core_sensor_sub_;

  sensor_msgs::PointCloud2 pointcloud_;
  std_msgs::String msg_;
  void coreSensorCB(const rehab_msgs::SensorState::ConstPtr& msg);
};

} 
#endif 
