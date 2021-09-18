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
#include "geometry_msgs/WrenchStamped.h"

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
        void BroadcastTransform(string parent_frame, string child_frame, geometry_msgs::PoseStamped transformation);
        void BroadcastTransform(string parent_frame, string child_frame, Eigen::Affine3d transformation);
        void WrenchRvizDisplay(Eigen::VectorXd vel, string frame_name, double sacle);
        Eigen::Affine3d ListenToTransform_Eigen(string base_frame, string target_frame);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pos_tra_controller_;
        ros::Publisher wrench_display_pub_;

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

class WrenchRvizDisplay
{
    public:
        WrenchRvizDisplay(string topic_name)
        {
            wrench_display_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
        }
        virtual ~WrenchRvizDisplay(){}
        void Display(Eigen::VectorXd vel, string frame_name, double sacle)
        {
            geometry_msgs::WrenchStamped wrench;
            wrench.header.frame_id = frame_name;
            wrench.header.stamp = ros::Time::now();
            vel = vel * sacle;
            wrench.wrench.force.x = vel(0);
            wrench.wrench.force.y = vel(1);
            wrench.wrench.force.z = vel(2);
            wrench.wrench.torque.x = vel(3);
            wrench.wrench.torque.y = vel(4);
            wrench.wrench.torque.z = vel(5);
            wrench_display_pub_.publish(wrench);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher wrench_display_pub_;
};

#endif
