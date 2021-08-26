#include "VisualServo.h"

VisualServo::VisualServo(std::shared_ptr<SharedVariable> ptr)
{
    shared_variable_ptr_ = ptr;
}

VisualServo::~VisualServo()
{
    cout << "VisualServo destructor" << endl;
}

Eigen::VectorXd VisualServo::PBVS(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object)
{
    // Eigen::Affine3d desired_cam_to_cam = desired_camera_to_object * camera_to_object.inverse();
    // Eigen::Affine3d task_frame_2_end_effector = end_effector_to_camera_ * desired_cam_to_cam * end_effector_to_camera_.inverse();
    
    // auto pose_error = FromeMatrixToErrorAxisAngle(task_frame_2_end_effector.inverse());
    // const Eigen::VectorXd velocity_task_frame =  lambda_ * pose_error;
    
    // const auto base_to_end_effector = get_base_2_end_effector();
    // const Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix( base_to_end_effector*task_frame_2_end_effector.inverse() ) * velocity_task_frame;
    
    Eigen::Affine3d cam_to_desired_cam = camera_to_object * desired_camera_to_object.inverse();
    auto pose_error = FromeMatrixToErrorAxisAngle(cam_to_desired_cam);

    const Eigen::VectorXd velocity_camera_frame =  lambda_ * pose_error;

    auto velocity_end_effector_frame = AdjointTransformationMatrix(end_effector_to_camera_) * velocity_camera_frame;
    const auto base_to_end_effector = get_base_2_end_effector();
    Eigen::VectorXd velocity_base_frame = AdjointNoTranslation(base_to_end_effector) * velocity_end_effector_frame;

    return velocity_base_frame;
}

Eigen::VectorXd VisualServo::PBVS_TR(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object)
{
    Eigen::Affine3d cam_to_desired_cam = camera_to_object * desired_camera_to_object.inverse();
    Eigen::VectorXd pose_error = FromeMatrixToErrorAxisAngle(cam_to_desired_cam);

    Eigen::VectorXd velocity_camera_frame(6);
    velocity_camera_frame.block(0,0,3,1) = lambda_ * pose_error.block(0,0,3,1);
    velocity_camera_frame.block(3,0,3,1) = rotation_lambda_ * pose_error.block(3,0,3,1);

    Eigen::VectorXd velocity_end_effector_frame = AdjointTransformationMatrix(end_effector_to_camera_) * velocity_camera_frame;
    auto base_to_end_effector = get_base_2_end_effector();
    Eigen::VectorXd velocity_base_frame = AdjointNoTranslation(base_to_end_effector) * velocity_end_effector_frame;
    
    // cout << "pose_error: " << endl << pose_error << endl;
    // cout << "velocity_end_effector_frame:" << endl << velocity_end_effector_frame << endl;
    // cout << "velocity_end_effector_frame:" << endl << velocity_end_effector_frame << endl;
    // cout << "velocity_base_frame:" << endl << velocity_base_frame << endl;

    return velocity_base_frame;
}

Eigen::VectorXd VisualServo::PBVS3_TR(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object)
{
    Eigen::Affine3d desired_cam_to_cam = desired_camera_to_object * camera_to_object.inverse();
    auto pose_error = FromeMatrixToErrorAxisAngle(desired_cam_to_cam.inverse());
    
    Eigen::VectorXd velocity_task_frame(6);
    velocity_task_frame.block(0,0,3,1) = lambda_ * pose_error.block(0,0,3,1);
    velocity_task_frame.block(3,0,3,1) = rotation_lambda_ * pose_error.block(3,0,3,1);
    const auto base_to_end_effector = get_base_2_end_effector();
    const Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix( base_to_end_effector*end_effector_to_camera_.inverse() ) * velocity_task_frame;
    // cout << "velocity_task_frame :" << endl << velocity_task_frame << endl;
    // cout << "velocity_base_frame :" << endl << velocity_base_frame << endl;
    return velocity_base_frame;
}

Eigen::VectorXd VisualServo::PBVS1(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object)
{
    auto desired_cam_to_cam = desired_camera_to_object * camera_to_object.inverse();
    auto vec_error = FromeMatrixToErrorAxisAngle(desired_cam_to_cam);
    // vec_error = vec_desired_cam_to_cam - 0 = s - s*

    Eigen::MatrixXd jacobian_e(6, 6);
    jacobian_e.block(0,0,6,6) = Eigen::MatrixXd::Zero(6,6);
    jacobian_e.block(0,0,3,3) = desired_cam_to_cam.rotation().inverse();
    jacobian_e.block(3,3,3,3) = Eigen::Matrix3d::Identity();

    auto velocity_camera_frame = -lambda_ * jacobian_e * vec_error;

    const auto base_to_end_effector = get_base_2_end_effector();
    auto base_to_camera = base_to_end_effector * end_effector_to_camera_;
    Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix(base_to_camera) * velocity_camera_frame;
    // cout << "velocity_base_frame: " << endl << velocity_base_frame << endl;
    return velocity_base_frame;
}

Eigen::VectorXd VisualServo::PBVS2(const Eigen::Affine3d& camera_to_object, const Eigen::Affine3d& desired_camera_to_object)
{
    auto vec_error = FromeMatrixToErrorAxisAngle(camera_to_object);
    vec_error.block(0,0,3,1) = vec_error.block(0,0,3,1)  - desired_camera_to_object.matrix().block(0,3,3,1);
    // vec_error = vec_camera_to_object - (c*^t_o, 0)task_frame_2_end_effector = s - s*

    Eigen::MatrixXd jacobian_e(6, 6);
    jacobian_e.block(0,0,6,6) = Eigen::MatrixXd::Zero(6,6);
    jacobian_e.block(0,0,3,3) = - Eigen::Matrix3d::Identity();
    jacobian_e.block(0,3,3,3) = SkewSymmetricMatrix(camera_to_object.matrix().block(0,3,3,1));
    jacobian_e.block(3,3,3,3) = Eigen::Matrix3d::Identity();

    auto velocity_camera_frame = -lambda_ * jacobian_e * vec_error;

    const auto jacobian = get_jacobian();
    const auto base_to_end_effector = get_base_2_end_effector();
    auto base_to_camera = base_to_end_effector * end_effector_to_camera_;
    Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix(base_to_camera) * velocity_camera_frame;
    Eigen::VectorXd velocity_joint_space = jacobian.inverse() * velocity_base_frame;
    cout << "PBVS2 velocity_base_frame: " << endl << velocity_base_frame << endl;

    return velocity_joint_space;
}

Eigen::VectorXd VisualServo::IBVS()
{

}

void VisualServo::set_lambda(double lambda)
{
    lambda_ = lambda;
}

void VisualServo::set_rotation_lambda(double rotation_lambda)
{
    rotation_lambda_ = rotation_lambda;
}

double VisualServo::get_lambda()
{
    return lambda_;
}

double VisualServo::get_rotation_lambda()
{
    return rotation_lambda_;
}

void VisualServo::set_link_name_(const string end_effector_link, const string camera_link)
{
    end_effector_link_ = end_effector_link;
    camera_link_ = camera_link;
    
    tf::StampedTransform transform;
    static tf::TransformListener listener(ros::Duration(5));
    listener.waitForTransform(end_effector_link_, camera_link_, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(end_effector_link_, camera_link_, ros::Time(0), transform);
    
    end_effector_to_camera_.setIdentity();
    end_effector_to_camera_.translate( Eigen::Vector3d(transform.getOrigin()) );
    end_effector_to_camera_.rotate( Eigen::AngleAxisd(transform.getRotation().getAngle(), Eigen::Vector3d(transform.getRotation().getAxis())) );
    
    cout << "end_effector_to_camera_:" << endl << end_effector_to_camera_.matrix() << endl;
}


Eigen::VectorXd VisualServo::FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error)
{
    Eigen::AngleAxisd rotation_error(transformation_error.linear());
    Eigen::VectorXd pose_error(6);
    pose_error.head(3) = transformation_error.translation();
    pose_error.tail(3) = rotation_error.axis()*rotation_error.angle();
    return pose_error;
}

Eigen::MatrixXd VisualServo::AdjointTransformationMatrix(const Eigen::Affine3d& transformation)
{
    const auto rotation = transformation.rotation();
    const auto translation = transformation.translation();
    Eigen::MatrixXd adjoint_tf(6, 6);
    adjoint_tf.block(0,0,3,3) = rotation;
    adjoint_tf.block(0,3,3,3) = SkewSymmetricMatrix(translation) * rotation;
    adjoint_tf.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    adjoint_tf.block(3,3,3,3) = rotation;
    return adjoint_tf;
}

Eigen::MatrixXd VisualServo::AdjointNoTranslation(const Eigen::Affine3d& transformation)
{
    const auto rotation = transformation.rotation();
    const auto translation = transformation.translation();
    Eigen::MatrixXd adjoint_tf(6, 6);
    adjoint_tf.block(0,0,3,3) = rotation;
    adjoint_tf.block(0,3,3,3) = Eigen::Matrix3d::Zero(3,3);
    adjoint_tf.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    adjoint_tf.block(3,3,3,3) = rotation;
    return adjoint_tf;
}

Eigen::Matrix3d VisualServo::SkewSymmetricMatrix(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0,             -1*vector(2),   vector(1), 
                            vector(2),    0,               -1*vector(0), 
                            -1*vector(1), vector(0),      0;
    return skew_symmetric_matrix;
}

Eigen::Affine3d VisualServo::get_base_2_end_effector()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::Affine3d base_2_end_effector = shared_variable_ptr_->end_effector_state;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return base_2_end_effector;
}

Eigen::VectorXd VisualServo::get_ft_link_wrench()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd ft_link_wrench = shared_variable_ptr_->wrench;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return ft_link_wrench;
}

Eigen::MatrixXd VisualServo::get_jacobian()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::MatrixXd jacobian = shared_variable_ptr_->jacobian;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return jacobian;
}

Eigen::VectorXd VisualServo::get_joint_velocity()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd joint_velocity = shared_variable_ptr_->joint_velocity;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return joint_velocity;
}

Eigen::VectorXd VisualServo::get_joint_states()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd joint_states = shared_variable_ptr_->joint_states;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return joint_states;
}

Eigen::VectorXd VisualServo::ToJointSpaceVelocity(Eigen::VectorXd velocity_base_frame)
{
    const auto jacobian = get_jacobian();
    Eigen::VectorXd velocity_joint_space = jacobian.inverse() * velocity_base_frame;
    return velocity_joint_space;
}
