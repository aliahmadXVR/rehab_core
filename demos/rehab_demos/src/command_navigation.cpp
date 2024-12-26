
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
bool person_point = false;
int goalId =0;
void cameraCallBack(const geometry_msgs::PointStamped::ConstPtr &ptr){
    if (ptr->point.x != -9999 || ptr->point.y != -9999 || ptr->point.z != -9999){


      person_point = true;  
    }
}
void idCallBack(const std_msgs::String::ConstPtr &ptr){
   if (ptr->data == "bedroom"){
       goalId = 1;
   }else if (ptr->data == "kitchen"){
       goalId = 2;
   }else if(ptr->data == "lobby"){
       goalId = 3;
   }else if(ptr->data == "tvroom"){
       goalId = 4;
   }
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "command_navigation");
  ros::NodeHandle n;
  int count = 0;
  ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/person_loc",1000, cameraCallBack);
  ros::Subscriber goal_sub = n.subscribe<std_msgs::String>("/goal_command",1000, idCallBack);
  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);

  ros::Rate r(10);
  geometry_msgs::Twist msg;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  while(ros::ok()){
    if(goalId ==1){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  -6.49307060242;
        goal.target_pose.pose.position.y =  3.0324485302;
        goal.target_pose.pose.orientation.z =  0.927666739724;
        goal.target_pose.pose.orientation.w = 0.37340918576;

        ROS_INFO("Moving to bedroom");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

          ROS_INFO("I have reached in bedroom :)");
          while(!person_point && count<220){
            count++;
            msg.angular.z = 0.3;
            pub_vel.publish(msg);
            r.sleep();
            ros::spinOnce();
          }
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          person_point = false;
          count = 0;
        }
        goalId = 0;
    }
    else if(goalId==2){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  0.429913520813;
        goal.target_pose.pose.position.y =  2.6376285553;
        goal.target_pose.pose.orientation.z =  0.676470628333;
        goal.target_pose.pose.orientation.w = 0.736469611731;

        ROS_INFO("Moving to kitchen");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached in kitchen :)");
          while(!person_point && count<220){
            count++;
            msg.angular.z = 0.3;
            pub_vel.publish(msg);
            r.sleep();
            ros::spinOnce();
          }
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          person_point = false;
          count = 0;
        }
        goalId = 0;
    }
    else if(goalId ==3){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  -3.88095235825;
        goal.target_pose.pose.position.y =  0.418211221695;
        goal.target_pose.pose.orientation.z =  -0.617896597104;
        goal.target_pose.pose.orientation.w = 0.786259368966;

        ROS_INFO("Moving to lobby");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached in lobby :)");
          while(!person_point && count<220){
            count++;
            msg.angular.z = 0.3;
            pub_vel.publish(msg);
            r.sleep();
            ros::spinOnce();
          }
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          person_point = false;
          count = 0;
        }
        goalId = 0;
    }
        else if(goalId ==4){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  -6.44437217712;
        goal.target_pose.pose.position.y =  -1.17962515354;
        goal.target_pose.pose.orientation.z =  0.855862761088;
        goal.target_pose.pose.orientation.w = 0.517202991274;

        ROS_INFO("Moving to tvroom");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("I have reached in tvroom :)");
          while(!person_point && count<220){
            count++;
            msg.angular.z = 0.3;
            pub_vel.publish(msg);
            r.sleep();
            ros::spinOnce();
          }
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          msg.angular.z = 0.0;
          pub_vel.publish(msg);
          person_point = false;
          count = 0;
        }
        goalId = 0;
    }else{

    }
    ros::spinOnce();

  }
  return 0;
}


