#include "StatesHub.h"

StatesHub::StatesHub(std::shared_ptr<SharedVariable> ptr)
:robot_model_loader_("robot_description")
{
    shared_variable_ptr_ = ptr;
    // config_tree_ = shared_variable_ptr_->config_tree;
    force_torque_sensor_sub_ = nh_.subscribe("/ft_sensor/ft_compensated", 1, &StatesHub::force_torque_sensor_callback, this);
    joint_states_sub_ = nh_.subscribe("/joint_states", 1, &StatesHub::joint_states_callback, this);
    // kinematic model
    kinematic_model_ = robot_model_loader_.getModel();
    robot_state::RobotStatePtr k_s(new robot_state::RobotState(kinematic_model_));
    kinematic_state_ = k_s;
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup("manipulator");
    shared_variable_ptr_->joint_names = joint_model_group_->getVariableNames();

    pkf_.SetParammeters(1.0/500.0, 0.1, 40.0);
	pkf_.InitializeKalmanFilter();
}

StatesHub::~StatesHub()
{
    std::cout << "~StatesHub()" << std::endl;
}

void StatesHub::force_torque_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr msg)
{
    Eigen::VectorXd force_torque(6);
    force_torque << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    pkf_.UpdateStateEulerAngle(force_torque);
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
	shared_variable_ptr_->wrench = force_torque;
    shared_variable_ptr_->filtered_wrench = pkf_.GetEstimateState();
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    // std::cout << "force_torque" << shared_variable_ptr_->force_torque << std::endl;

}

void StatesHub::joint_states_callback(const sensor_msgs::JointState::ConstPtr msg)
{
    Eigen::VectorXd joint_states(6);
    Eigen::VectorXd joint_velocity(6);

    // the order of joint angles is arranged alphabetically according to joint's name
    joint_states << msg->position[2], msg->position[1], msg->position[0], 
        msg->position[3], msg->position[4], msg->position[5];

    joint_velocity << msg->velocity[2], msg->velocity[1], msg->velocity[0], 
        msg->velocity[3], msg->velocity[4], msg->velocity[5];

    kinematic_state_->setVariableValues(*msg);
    const Eigen::Affine3d& end_effector_state = kinematic_state_->getGlobalLinkTransform("ee_link");

    Eigen::MatrixXd jacobian;
    kinematic_state_->getJacobian(joint_model_group_,
                               kinematic_state_->getLinkModel("ee_link"),
                               Eigen::Vector3d(0, 0, 0), jacobian);
                      
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
	shared_variable_ptr_->joint_states = joint_states;
    shared_variable_ptr_->joint_velocity = joint_velocity;
    shared_variable_ptr_->end_effector_state = end_effector_state;
    shared_variable_ptr_->jacobian = jacobian;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    
    // std::cout << "joint_states" << joint_states << std::endl;

}
