#ifndef SHARED_VARIABLE_H
#define SHARED_VARIABLE_H
#include <thread>
#include <Eigen/Eigen>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

struct SharedVariable
{
    Eigen::VectorXd wrench;
    Eigen::VectorXd filtered_wrench;
    Eigen::VectorXd joint_states;
    Eigen::VectorXd joint_velocity;
    std::vector<std::string> joint_names;
    Eigen::MatrixXd jacobian;
    Eigen::Affine3d end_effector_state;

    pthread_rwlock_t shared_variables_rwlock;
    boost::property_tree::ptree config_tree;

};

#endif