
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>

#include <cstdlib>
int goalId = 1;
// void idCallBack(const std_msgs::Int16::ConstPtr &ptr){
//   goalId = ptr->data;
// }

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "map_navigation_startup");
  ros::NodeHandle n;
  srand(time(0));


  // ros::Subscriber goal_sub = n.subscribe<std_msgs::Int16>("goal_id",1000, idCallBack);
  ros::Rate r(10);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  while(ros::ok()){
    goalId = 1+ (rand() % 3);
    if(goalId ==1){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  -1.00229763985;
        goal.target_pose.pose.position.y =  -0.190472722054;
        goal.target_pose.pose.orientation.z =  0.9067858209;
        goal.target_pose.pose.orientation.w = 0.421591597419;

        ROS_INFO("Moving to point A");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached at point A :)");
          ros::spinOnce();
        }
    }
    else if(goalId ==2){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  3.61923360825;
        goal.target_pose.pose.position.y = 4.19984102249;
        goal.target_pose.pose.orientation.z = 0.549715098274;
        goal.target_pose.pose.orientation.w = 0.835352207592;

        ROS_INFO("Moving to point B");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached at point B :)");
          ros::spinOnce();
        }
    }
    else if(goalId ==3){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  -1.60645675659;
        goal.target_pose.pose.position.y =  3.57930850983;
        goal.target_pose.pose.orientation.z =  1;
        goal.target_pose.pose.orientation.w = 0;

        ROS_INFO("Moving to point C");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached at point C :)");
          ros::spinOnce();
        }
    }
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}


