#ifndef STATES_HUB_H
#define STATES_HUB_H
// #include <stdlib.h>
// #include <stdio.h>
#include <thread>
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "SharedVariable.h"

class StatesHub
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber force_torque_sensor_sub_;
    ros::Subscriber joint_states_sub_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    robot_state::JointModelGroup* joint_model_group_;

    std::shared_ptr<SharedVariable> shared_variable_ptr_;
    // boost::property_tree::ptree config_tree_;

    
public:
    StatesHub(std::shared_ptr<SharedVariable> ptr);
    ~StatesHub();

    void force_torque_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr msg);

};

#endif

