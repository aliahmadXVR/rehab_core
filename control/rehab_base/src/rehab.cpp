#include <ros/ros.h>
#include <rehab_base/rehab_hw_interface.h>
#include <controller_manager/controller_manager.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rehab_hw_interface");
    ros::NodeHandle nh;
    rehab_base::RehabHWInterface rehab(nh);

    controller_manager::ControllerManager cm(&rehab);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); 
    rate.sleep();

    while (ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        rehab.read(time, period);
        cm.update(time, period);
        rehab.write(time, period);
        rate.sleep();
    }
    return 0;
}
