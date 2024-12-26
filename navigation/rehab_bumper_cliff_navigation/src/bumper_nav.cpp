
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h> // move_base
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <rehab_msgs/SensorState.h>


geometry_msgs::Pose2D current_pose;
geometry_msgs::PoseStamped previous_pose;
float getCurrentOdomOnce(){
  ROS_INFO("Returning current position");
  return current_pose.x;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
}

int goalId = 0;
void bumperCallBack(const rehab_msgs::SensorState::ConstPtr &ptr){
  if(ptr->bumper != 0){
    goalId = 1;
  }
}

void previousCallBack(const geometry_msgs::PoseStamped::ConstPtr &ptr){
  previous_pose.header = ptr->header;
  previous_pose.pose = ptr->pose;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "bumper_nav");
  ros::NodeHandle n;
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel",1);
  ros::Publisher previous_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  ros::Subscriber goal_sub = n.subscribe<rehab_msgs::SensorState>("/sensors_state",1000, bumperCallBack);
  ros::Subscriber previous_goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base/current_goal",1, previousCallBack);
  ros::Subscriber sub_odometry = n.subscribe<nav_msgs::Odometry>("mobile_base_controller/odom", 1, odomCallback);
  geometry_msgs::Twist move;

  ros::Rate r(100);
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  while(ros::ok()){
    if(goalId == 1){
      ac.cancelAllGoals();
      ROS_INFO("Cancelling goal");
      goalId = 0;
      int count = 0;
      float distanceM = getCurrentOdomOnce();
      while(1){
        // if(getCurrentOdomOnce()<= (distanceM-1)){
        if(count <230){
          move.linear.x = -0.08; //Change this value to control the backward motion of robot
          movement_pub.publish(move);
          count++;
          // ROS_INFO("current distance = %0.2f,   goal distance = %0.2f", current_pose.x,distanceM-1);
          r.sleep();
          // ros::spinOnce();
        }
        else{
          move.linear.x = 0;
          movement_pub.publish(move);
          ROS_INFO("Stopping");
          r.sleep();
          break;
        }
      }
previous_goal_pub.publish(previous_pose);
    }     
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}


