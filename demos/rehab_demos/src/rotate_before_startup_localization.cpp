#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "rotate_before_startup_localization");
    ros::NodeHandle n;
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
    int count = 0;
    ros::Rate r(10);
    geometry_msgs::Twist msg;
    while(count<220){
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
    count = 0;

    return 0;
}