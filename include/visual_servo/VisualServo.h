#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "ros/ros.h"
#include <Eigen/Eigen>
#include "SharedVariable.h"
#include <tf/transform_listener.h>
using namespace std;

class VisualServo
{
    public:
        VisualServo(std::shared_ptr<SharedVariable> ptr);
        ~VisualServo();

        Eigen::VectorXd PBVS1(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object);
        Eigen::VectorXd PBVS2(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object);
        Eigen::VectorXd IBVS();
        void set_lambda(double lambda);
        void set_link_name_(const string end_effector_link, const string camera_link);
        Eigen::VectorXd ToJointSpaceVelocity(Eigen::VectorXd velocity_base_frame);

    private:
        Eigen::VectorXd FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error);
        Eigen::MatrixXd AdjointTransformationMatrix(const Eigen::Affine3d& transformation);
        Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& vector);
        Eigen::VectorXd WrenchTruncation(Eigen::VectorXd original_wrench, double force_threshold, double torque_threshold);
        Eigen::VectorXd PoseErrorEpsilon(Eigen::VectorXd original_vector, double position_threshold, double orientation_threshold);
        Eigen::VectorXd CalculateTaskFrameWrench(Eigen::Affine3d& task_frame_2_end_effector);
        

        Eigen::Affine3d get_base_2_end_effector();
        Eigen::VectorXd get_ft_link_wrench();
        Eigen::MatrixXd get_jacobian();
        Eigen::VectorXd get_joint_velocity();
        Eigen::VectorXd get_joint_states();

        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        double lambda_ = 0.001;
        Eigen::Affine3d end_effector_to_camera_;
        string end_effector_link_ = "ee_link";
        string camera_link_ = "camera_color_optical_frame";

};

#endif
