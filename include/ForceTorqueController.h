#ifndef FORCE_TORQUE_CONTROLLER_H
#define FORCE_TORQUE_CONTROLLER_H

#include <thread>
#include <Eigen/Eigen>
#include "SharedVariable.h"
#include "utils/utils.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "StatesHub.h"
#include <tf/transform_listener.h>
#include <control_msgs/JointJog.h>


using namespace std;
namespace pt = boost::property_tree;

class ForceTorqueController
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        Eigen::Affine3d ft_link_2_end_effector_;
        Eigen::MatrixXd wrench_scaling_;
        Eigen::MatrixXd stiffness_;
        Eigen::MatrixXd damping_;
        Eigen::MatrixXd mass_;
        Eigen::MatrixXd mass_inverse_;
        const double delta_t_;
        double adaptive_sigma;
        double force_threshold_;
        double torque_threshold_;
        double position_threshold_;
        double orientation_threshold_;

        ros::NodeHandle nh_;
        ros::Publisher vel_pub_;
        ros::Publisher jog_vel_pub_;
        Eigen::VectorXd pose_error_;
        Eigen::VectorXd zero_ft_link_wrench_;

        
        
    public:
        ForceTorqueController(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~ForceTorqueController();

        Eigen::VectorXd AdmittanceVelocityController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);
        Eigen::VectorXd AdmittancePositionController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);
        Eigen::VectorXd ZeroMomentVelocityController();
        Eigen::VectorXd AdaptiveForceVelocityController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);
        Eigen::VectorXd ForceVelocityController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);

        Eigen::VectorXd FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error);
        Eigen::MatrixXd AdjointTransformationMatrix(const Eigen::Affine3d& transformation);
        Eigen::MatrixXd AdjointNoTranslation(const Eigen::Affine3d& transformation);
        Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& vector);
        Eigen::VectorXd WrenchTruncation(Eigen::VectorXd original_wrench, double force_threshold, double torque_threshold);
        Eigen::VectorXd PoseErrorEpsilon(Eigen::VectorXd original_vector, double position_threshold, double orientation_threshold);
        Eigen::VectorXd CalculateTaskFrameWrench(Eigen::Affine3d& task_frame_2_end_effector);
        Eigen::VectorXd ToJointSpaceVelocity(Eigen::VectorXd velocity_base_frame);
        void UpdateZeroWrench();

        Eigen::Affine3d get_base_2_end_effector();
        Eigen::VectorXd get_ft_link_wrench();
        Eigen::MatrixXd get_jacobian();
        Eigen::VectorXd get_joint_velocity();
        Eigen::VectorXd get_joint_states();

        void Approach(double z_force_threshold, double approaching_speed);
        bool StaticPointControl(const Eigen::Affine3d task_frame_pose, const Eigen::VectorXd& expected_wrench, bool keep_moving);
        bool MultiplePointsControl(geometry_msgs::PoseArray task_frame_pose_array);
        void SetZeroVelocity();
        void PubJointVelCmd(Eigen::VectorXd jonit_velocity);


};

#endif
