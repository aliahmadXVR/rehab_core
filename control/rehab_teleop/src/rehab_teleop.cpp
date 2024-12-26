#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopRehab
{
public:
  TeleopRehab();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_=1; 
  double a_scale_=1;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopRehab::TeleopRehab():
  linear_(4),
  angular_(3)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRehab::joyCallback, this);

}

void TeleopRehab::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
if (joy->buttons[4])
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}
else if (joy->axes[2]==-1)
{
geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rehab_teleop");
  TeleopRehab teleop;

  ros::spin();
}
