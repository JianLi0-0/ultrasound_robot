#ifndef FORCE_TORQUE_CONTROLLER_H
#define FORCE_TORQUE_CONTROLLER_H

#include <thread>
#include <Eigen/Eigen>
#include "SharedVariable.h"
#include "utils/utils.hpp"

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
        double force_threshold;
        double torque_threshold;
        
    public:
        ForceTorqueController(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~ForceTorqueController();

        Eigen::VectorXd AdmittanceVelocityController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);
        Eigen::VectorXd AdmittancePositionController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);
        Eigen::VectorXd ZeroMomentVelocityController();
        Eigen::VectorXd AdaptiveForceVelocityController(const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench);

        Eigen::VectorXd FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error);
        Eigen::MatrixXd AdjointTransformationMatrix(const Eigen::Affine3d& transformation);
        Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& vector);
        Eigen::VectorXd WrenchTruncation(Eigen::VectorXd original_wrench, double force_threshold, double torque_threshold);

        Eigen::Affine3d get_base_2_end_effector();
        Eigen::VectorXd get_ft_link_wrench();
        Eigen::MatrixXd get_jacobian();
        Eigen::VectorXd get_joint_velocity();
        Eigen::VectorXd get_joint_states();


};

#endif
