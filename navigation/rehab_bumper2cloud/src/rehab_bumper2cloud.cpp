/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "rehab_bumper2cloud/rehab_bumper2cloud.hpp"

namespace rehab_bumper2cloud
{

void Bumper2PcNodelet::coreSensorCB(const rehab_msgs::SensorState::ConstPtr& msg)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
   {
      return;
  }

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper && ! prev_cliff)
  {
      // msg_.data = "inside 2nd if";
      return;
  }
    

  prev_bumper = msg->bumper;
  prev_cliff  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 4, CENTER = 2 and RIGHT = 1
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
if ((msg->bumper & rehab_msgs::SensorState::BUMPER_LEFT) ||  (msg->cliff  & rehab_msgs::SensorState::CLIFF_LEFT))
{
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
    // ROS_INFO("Center bumper");
    msg_.data ="left bumper";
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if ((msg->bumper & rehab_msgs::SensorState::BUMPER_CENTRE) )
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
    // ROS_INFO("Center bumper");
    msg_.data ="center bumper";
  }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if ((msg->bumper & rehab_msgs::SensorState::BUMPER_RIGHT ) ||  (msg->cliff  & rehab_msgs::SensorState::CLIFF_RIGHT))
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
    // ROS_INFO("Center bumper");
    msg_.data ="rigth bumper";
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  status_pub_.publish(msg_);
  pointcloud_.header.stamp = msg->header.stamp;
  pointcloud_pub_.publish(pointcloud_);
}

void Bumper2PcNodelet::onInit()
{
  ros::NodeHandle nh = this->getPrivateNodeHandle();

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  std::string base_link_frame;
  double r, h, angle;
  nh.param("pointcloud_radius", r, 0.25); pc_radius_ = r;
  nh.param("pointcloud_height", h, 0.02); pc_height_ = h;
  nh.param("side_point_angle", angle, 0.015); 
  nh.param<std::string>("base_link_frame", base_link_frame, "base_link");

  // Lateral points x/y coordinates; we need to store float values to memcopy later
  p_side_x_ = 0.2+ pc_radius_*sin(angle); // angle degrees from vertical
  p_side_y_ = -0.1+ pc_radius_*cos(angle); // angle degrees from vertical
  n_side_y_ = 0.1- pc_radius_*cos(angle); // angle degrees from vertical
  // n_side_y_ = - pc_radius_*cos(angle);

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

  pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("pointcloud", 10);
  
  status_pub_  = nh.advertise <std_msgs::String> ("status", 10);

  core_sensor_sub_ = nh.subscribe("/sensors_state", 10, &Bumper2PcNodelet::coreSensorCB, this);

  ROS_INFO("Bumper/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
}

} // namespace kobuki_bumper2pc


PLUGINLIB_EXPORT_CLASS(rehab_bumper2cloud::Bumper2PcNodelet, nodelet::Nodelet);
