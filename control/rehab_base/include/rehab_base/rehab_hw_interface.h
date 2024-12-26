#ifndef REHAB_HW_INTERFACE_H
#define REHAB_HW_INTERFACE_H


// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <rehab_msgs/EncodersStamped.h>
#include <rehab_msgs/WheelsCmdStamped.h>
#include <rehab_msgs/AngularVelocitiesStamped.h>
#include <sensor_msgs/JointState.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>



namespace rehab_base
{
    const unsigned int NUM_JOINTS = 2;

    struct JointState
    {
        float angular_position_;
        float angular_velocity_;
    };
    class RehabHWInterface : public hardware_interface::RobotHW
    {
    public:
        RehabHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

        virtual ~RehabHWInterface() {}
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        virtual void read(const ros::Time& time, const ros::Duration& period) override;
        virtual void write(const ros::Time& time, const ros::Duration& period);
        bool isReceivingMeasuredJointStates(const ros::Duration &timeout=ros::Duration(1));

    protected:
        virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);
        void measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states);
        double ticksToAngle(const int &ticks) const;
        double normalizeAngle(double &angle) const;
        double linearToAngular(const double &distance) const;
        double angularToLinear(const double &angle) const;
        
        std::string name_;
        ros::NodeHandle nh_;
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        std::vector<std::string> joint_names_;
        std::size_t num_joints_;
        urdf::Model *urdf_model_;

        double wheel_radius_;
        double wheel_diameter_;
        double max_velocity_;
        double encoder_resolution_;
        bool debug_;
        double joint_velocity_commands_[NUM_JOINTS];
        double joint_positions_[NUM_JOINTS];
        double joint_velocities_[NUM_JOINTS];
        double joint_efforts_[NUM_JOINTS];

        ros::ServiceServer srv_start_;
        ros::ServiceServer srv_stop_;

        ros::Publisher pub_left_motor_value_;
        ros::Publisher pub_right_motor_value_;

        ros::Publisher pub_wheel_cmd_velocities_;

        ros::Publisher pub_reset_encoders_;
        ros::Subscriber sub_encoder_ticks_;
        ros::Subscriber sub_measured_joint_states_;
        int encoder_ticks_[NUM_JOINTS];
        JointState measured_joint_states_[NUM_JOINTS];
    };  
} 

#endif 
