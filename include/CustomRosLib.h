#ifndef CUSTOM_ROS_LIB_H
#define CUSTOM_ROS_LIB_H

#include <cmath>
#include<iostream>
#include<fstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <trac_ik/trac_ik.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/SwitchController.h>
#include "SharedVariable.h"
#include <control_msgs/JointJog.h>
#include <controller_manager_msgs/ListControllers.h>

using namespace std;

class CustomRosLib
{
    public:
        CustomRosLib(ros::NodeHandle &nh);
        CustomRosLib(std::shared_ptr<SharedVariable> ptr);
        virtual ~CustomRosLib();

        bool CartesianPositionControl(geometry_msgs::Pose target_position, double duration, double delay);
        void JointPositionControl(std::vector<double> target_joint_position, double duration);
        std_msgs::Float64MultiArray ManipulatorIK(geometry_msgs::Pose target_pose);
        geometry_msgs::Pose ListenToTransform(string base_frame, string target_frame);
        tf::StampedTransform ListenToTransform_TF(string base_frame, string target_frame);
        void UpdateJointSeed(Eigen::VectorXd joint_states);
        bool ActivateController(string controller_name);
        bool DeactivateController(string controller_name);
        bool SwitchController(string stop_controller_name, string start_controller_name);
        void PublishJointVelocity(Eigen::VectorXd joint_velocity);
        void CheckBox(Eigen::Vector3d center, Eigen::Vector3d range);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pos_tra_controller_;

        // ros::Subscriber desired_pose_sub;
        std_msgs::Float64MultiArray home_angles_;

        TRAC_IK::TRAC_IK ik_solver_;
        KDL::JntArray joint_seed_;
        KDL::Frame desired_eef_pose_;
        std_msgs::Float64MultiArray last_computed_angle_;

        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        vector<string> joints_name_;
        ros::Publisher jog_vel_pub_;
};

#endif
