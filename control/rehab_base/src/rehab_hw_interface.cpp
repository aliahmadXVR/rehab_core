#include <rehab_base/rehab_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace rehab_base
{
    RehabHWInterface::RehabHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        if (urdf_model == NULL){
            loadURDF(nh, "robot_description");
            ROS_INFO("Found robot description on parameter server!");
        }            
        else{
            ROS_INFO("Found robot description on parameter server!");
            urdf_model_ = urdf_model;
        }

        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);

        wheel_diameter_ = 2.0 * wheel_radius_;
        max_velocity_ = linearToAngular(max_velocity_);

        ROS_INFO("Max velocity = %f", max_velocity_);
        ROS_INFO_STREAM("mobile_base_controller/wheel_radius: " << wheel_radius_);
        ROS_INFO_STREAM("mobile_base_controller/linear/x/max_velocity: " << max_velocity_);
        pub_wheel_cmd_velocities_ = nh_.advertise<rehab_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);
        sub_measured_joint_states_ = nh_.subscribe("measured_joint_states", 10, &RehabHWInterface::measuredJointStatesCallback, this);
        init(nh_, nh_);
        isReceivingMeasuredJointStates(ros::Duration(10));
    }

 
    bool RehabHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing Rehab Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; 

            joint_velocity_commands_[i] = 0.0;

            encoder_ticks_[i] = 0.0;
            measured_joint_states_[i].angular_position_ = 0.0;
            measured_joint_states_[i].angular_velocity_ = 0.0;
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing Rehab Hardware Interface");

        return true;
    }

    void RehabHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            joint_positions_[i] = measured_joint_states_[i].angular_position_;
            joint_velocities_[i] = measured_joint_states_[i].angular_velocity_;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }
    }

    void RehabHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        rehab_msgs::WheelsCmdStamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_velocity_commands_[i]);
        }

        pub_wheel_cmd_velocities_.publish(wheel_cmd_msg);
    }

    bool RehabHWInterface::isReceivingMeasuredJointStates(const ros::Duration &timeout)
    {
        ROS_INFO("Get number of measured joint states publishers");

        ros::Time start = ros::Time::now();
        int num_publishers = sub_measured_joint_states_.getNumPublishers();
        ROS_INFO("Waiting for measured joint states being published...");
        while ((num_publishers == 0) && (ros::Time::now() < start + timeout))
        {
            ros::Duration(0.1).sleep();
            num_publishers = sub_measured_joint_states_.getNumPublishers();
        }
        if (num_publishers == 0)
        {
            ROS_ERROR("No measured joint states publishers. Timeout reached.");
        }
        else
        {
            ROS_INFO_STREAM("Number of measured joint states publishers: " << num_publishers);
        }
        return (num_publishers > 0);
    }

    void RehabHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void RehabHWInterface::measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states)
    {
        /// Update current encoder ticks in encoders array
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            measured_joint_states_[i].angular_position_ = msg_joint_states->position[i];
            measured_joint_states_[i].angular_velocity_ = msg_joint_states->velocity[i];
        }
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }


    double RehabHWInterface::ticksToAngle(const int &ticks) const
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0*M_PI / encoder_resolution_);
        ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	    return angle;
    }

    double RehabHWInterface::normalizeAngle(double &angle) const
    {
        angle = fmod(angle, 2.0*M_PI);

        if (angle < 0)
            angle += 2.0*M_PI;
        return angle;
    }


    double RehabHWInterface::linearToAngular(const double &distance) const
    {
        return distance / wheel_diameter_ * 2.0;
    }

    double RehabHWInterface::angularToLinear(const double &angle) const
    {
        return angle * wheel_diameter_ / 2.0;
    }

};
