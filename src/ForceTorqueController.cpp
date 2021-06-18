#include "ForceTorqueController.h"

ForceTorqueController::ForceTorqueController(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree)
    :mass_(6,6), damping_(6,6), stiffness_(6,6), wrench_scaling_(6,6), 
    delta_t_(config_tree.get<double>("delta_t", 0.01))
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;

    auto translation = AsVector<double>(config_tree_, "robor_params.end_effector_2_ft_link.translation");
    auto quaternion = AsVector<double>(config_tree_, "robor_params.end_effector_2_ft_link.quaternion");
    Eigen::Affine3d end_effector_2_ft_link;
    end_effector_2_ft_link.setIdentity();
    end_effector_2_ft_link.translate( Eigen::Vector3d(translation.data()) );
    end_effector_2_ft_link.rotate( Eigen::Quaterniond(quaternion.data()) );
    ft_link_2_end_effector_ = end_effector_2_ft_link.inverse();

    // mass-damping-stiffness system parameters initialization
    auto mass = AsVector<double>(config_tree_, "admittance_params.mass");
    Eigen::Map<Eigen::VectorXd> mass_data(mass.data(), 6);
    mass_ = mass_data.asDiagonal();
    mass_inverse_ = mass_.inverse();

    auto stiffness = AsVector<double>(config_tree_, "admittance_params.stiffness");
    Eigen::Map<Eigen::VectorXd> stiffness_data(stiffness.data(), 6);
    stiffness_ = stiffness_data.asDiagonal();

    auto damping_scaling = AsVector<double>(config_tree_, "admittance_params.damping_scaling");
    Eigen::Map<Eigen::VectorXd> damping_scaling_data(damping_scaling.data(), 6);
    auto damping_scaling_matrix = damping_scaling_data.asDiagonal();
    auto critical_damping = 2*(mass_*stiffness_).array().sqrt();
    damping_ = critical_damping.matrix() * damping_scaling_matrix;

    auto wrench_scaling = AsVector<double>(config_tree_, "admittance_params.wrench_scaling");
    Eigen::Map<Eigen::VectorXd> wrench_scaling_data(wrench_scaling.data(), 6);
    wrench_scaling_ = wrench_scaling_data.asDiagonal();

    force_threshold = config_tree_.get<double>("admittance_params.ft_truncation.f", 1.5);
    torque_threshold = config_tree_.get<double>("admittance_params.ft_truncation.t", 0.5);
    position_threshold = config_tree_.get<double>("admittance_params.pose_epsilon.p", 0.001);
    orientation_threshold = config_tree_.get<double>("admittance_params.pose_epsilon.o", 0.001);

    // stiffness_ = stiffness_ * 0;
    // std::cout << "Free Drive" << std::endl;
    adaptive_sigma = config_tree.get<double>("adaptive_coefficient", 0.0);


    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);
    jog_vel_pub = nh_.advertise<control_msgs::JointJog>("/jog_arm_server/joint_delta_jog_cmds", 1);
}

ForceTorqueController::~ForceTorqueController()
{
    std::cout << "~ForceTorqueController()" << std::endl;
}

Eigen::VectorXd ForceTorqueController::AdmittanceVelocityController(const Eigen::Affine3d& base_2_task_frame, 
                                                                    const Eigen::VectorXd& expected_wrench)
{
    const Eigen::Affine3d task_frame_2_base = base_2_task_frame.inverse();
    const Eigen::Affine3d base_2_end_effector = get_base_2_end_effector();
    const Eigen::Affine3d task_frame_2_end_effector = task_frame_2_base * base_2_end_effector;
    // use task_frame_2_end_effector to calculate transformation error in task frame
    Eigen::VectorXd pose_error = FromeMatrixToErrorAxisAngle(task_frame_2_end_effector); // pose_error = X-Xd
    pose_error = PoseErrorEpsilon(pose_error, position_threshold, orientation_threshold);
    

    const Eigen::VectorXd original_wrench = get_ft_link_wrench();
    const Eigen::VectorXd ft_link_wrench = WrenchTruncation(original_wrench, force_threshold, torque_threshold);
    Eigen::Affine3d ft_link_2_task_frame = ft_link_2_end_effector_ * task_frame_2_end_effector.inverse(); // not done yet
    // transform the wrench from ft_link to the task frame: F_c = inv(Adjoint_bc.T) * F_b
    const Eigen::VectorXd task_frame_wrench = AdjointTransformationMatrix(ft_link_2_task_frame).transpose() * ft_link_wrench;
    const Eigen::VectorXd wrench_error = task_frame_wrench - expected_wrench; // wrench_error = F-Fd

    // apply admittance control law in task frame: X_dotdot = M^-1*( Kf(F-Fd)-Kd*X_dot-K(X-Xd) ) 
    const auto jacobian = get_jacobian();
    // const auto joint_velocity = get_joint_velocity();
    const auto joint_velocity = PoseErrorEpsilon(get_joint_velocity(), 0.001, 0.001);
    const Eigen::VectorXd end_effector_velocity = jacobian * joint_velocity;
    Eigen::VectorXd end_effector_velocity_task_frame = AdjointTransformationMatrix(task_frame_2_base) * end_effector_velocity;
    end_effector_velocity_task_frame = PoseErrorEpsilon(end_effector_velocity_task_frame, 0.005, 0.005);
    const Eigen::VectorXd acceleration_task_frame = mass_inverse_ * 
                ( wrench_scaling_*wrench_error - damping_*end_effector_velocity_task_frame - stiffness_*pose_error );
    const Eigen::VectorXd velocity_task_frame = end_effector_velocity_task_frame + acceleration_task_frame * delta_t_;

    // transform the velocity from task frame to the base frame: V_b = inv(Adjoint_tb) * V_t
    const Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix(task_frame_2_base.inverse()) * velocity_task_frame;
    const Eigen::VectorXd velocity_joint_space = jacobian.inverse() * velocity_base_frame;    

    return velocity_joint_space;
}

Eigen::VectorXd ForceTorqueController::ForceVelocityController(const Eigen::Affine3d& base_2_task_frame, 
                                                                    const Eigen::VectorXd& expected_wrench)
{
    const Eigen::Affine3d task_frame_2_base = base_2_task_frame.inverse();
    const Eigen::Affine3d base_2_end_effector = get_base_2_end_effector();
    const Eigen::Affine3d task_frame_2_end_effector = task_frame_2_base * base_2_end_effector;
    // use task_frame_2_end_effector to calculate transformation error in task frame
    pose_error_ = FromeMatrixToErrorAxisAngle(task_frame_2_end_effector); // pose_error = X-Xd
    Eigen::VectorXd pose_error = PoseErrorEpsilon(pose_error_, position_threshold, orientation_threshold);
    

    const Eigen::VectorXd original_wrench = get_ft_link_wrench();
    const Eigen::VectorXd ft_link_wrench = WrenchTruncation(original_wrench, force_threshold, torque_threshold);
    Eigen::Affine3d ft_link_2_task_frame = ft_link_2_end_effector_ * task_frame_2_end_effector.inverse(); // not done yet
    // transform the wrench from ft_link to the task frame: F_c = inv(Adjoint_bc.T) * F_b
    const Eigen::VectorXd task_frame_wrench = AdjointTransformationMatrix(ft_link_2_task_frame).transpose() * ft_link_wrench;
    const Eigen::VectorXd wrench_error = task_frame_wrench - expected_wrench; // wrench_error = F-Fd

    // apply admittance control law in task frame: X_dotdot = M^-1*( Kf(F-Fd)-Kd*X_dot-K(X-Xd) ) 
    const auto jacobian = get_jacobian();
    // const auto joint_velocity = get_joint_velocity();
    const auto joint_velocity = PoseErrorEpsilon(get_joint_velocity(), 0.001, 0.001);
    const Eigen::VectorXd end_effector_velocity = jacobian * joint_velocity;
    Eigen::VectorXd end_effector_velocity_task_frame = AdjointTransformationMatrix(task_frame_2_base) * end_effector_velocity;
    end_effector_velocity_task_frame = PoseErrorEpsilon(end_effector_velocity_task_frame, 0.005, 0.005);
    // const Eigen::VectorXd acceleration_task_frame = mass_inverse_ * 
    //             ( wrench_scaling_*wrench_error - damping_*end_effector_velocity_task_frame - stiffness_*pose_error );
    // const Eigen::VectorXd velocity_task_frame = end_effector_velocity_task_frame + acceleration_task_frame * delta_t_;
    const Eigen::VectorXd velocity_task_frame = wrench_scaling_*wrench_error - stiffness_*pose_error;

    // transform the velocity from task frame to the base frame: V_b = inv(Adjoint_tb) * V_t
    const Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix(task_frame_2_base.inverse()) * velocity_task_frame;
    const Eigen::VectorXd velocity_joint_space = jacobian.inverse() * velocity_base_frame;

    // cout << "original_wrench: " << endl << original_wrench << endl;
    // cout << "truncated_link_wrench: " << endl << ft_link_wrench << endl;
    // cout << "task_frame_wrench: " << endl << task_frame_wrench << endl;
    // cout << "end_effector_velocity_task_frame: " << endl << end_effector_velocity_task_frame << endl;
    // cout << "velocity_base_frame: " << endl << velocity_base_frame << endl;


    return velocity_joint_space;
}


Eigen::VectorXd ForceTorqueController::AdmittancePositionController(const Eigen::Affine3d& base_2_task_frame, 
                                                const Eigen::VectorXd& expected_wrench)
{
    auto velocity_joint_space = AdmittancePositionController(base_2_task_frame, expected_wrench);
    auto current_joint_angle = get_joint_states();
    const Eigen::VectorXd position_joint_space = current_joint_angle + velocity_joint_space * delta_t_;
    return position_joint_space;
}

Eigen::VectorXd ForceTorqueController::ZeroMomentVelocityController()
{
    stiffness_ = stiffness_ * 0;
    auto velocity_joint_space = AdmittanceVelocityController(get_base_2_end_effector(), Eigen::VectorXd::Zero(6));
    return velocity_joint_space;
}

Eigen::VectorXd ForceTorqueController::AdaptiveForceVelocityController
    (const Eigen::Affine3d& base_2_task_frame, const Eigen::VectorXd& expected_wrench)
{
    static Eigen::VectorXd Phi(Eigen::VectorXd::Zero(6));

    const Eigen::Affine3d task_frame_2_base = base_2_task_frame.inverse();
    const Eigen::Affine3d base_2_end_effector = get_base_2_end_effector();
    const Eigen::Affine3d task_frame_2_end_effector = task_frame_2_base * base_2_end_effector;
    // use task_frame_2_end_effector to calculate transformation error in task frame
    const Eigen::VectorXd pose_error = FromeMatrixToErrorAxisAngle(task_frame_2_end_effector); // pose_error = X-Xd

    const Eigen::VectorXd ft_link_wrench = get_ft_link_wrench();
    Eigen::Affine3d ft_link_2_task_frame = ft_link_2_end_effector_ * task_frame_2_end_effector.inverse(); // not done yet
    // transform the wrench from ft_link to the task frame: F_c = inv(Adjoint_bc.T) * F_b
    const Eigen::VectorXd task_frame_wrench = AdjointTransformationMatrix(ft_link_2_task_frame).transpose() * ft_link_wrench;
    const Eigen::VectorXd wrench_error = task_frame_wrench - expected_wrench; // wrench_error = F-Fd
    std::cout << "base_frame_wrench" << std::endl << AdjointTransformationMatrix(ft_link_2_task_frame*task_frame_2_base).transpose() * ft_link_wrench << std::endl;

    // apply admittance control law in task frame: X_dotdot = M^-1*( Kf(F-Fd)-Kd*X_dot-K(X-Xd) ) 
    const auto jacobian = get_jacobian();
    const auto joint_velocity = get_joint_velocity();
    const Eigen::VectorXd end_effector_velocity = jacobian * joint_velocity;
    const Eigen::VectorXd end_effector_velocity_task_frame = AdjointTransformationMatrix(task_frame_2_base) * end_effector_velocity;
    
    // only control the diretion specified by wrench_scaling_
    const Eigen::VectorXd acceleration_task_frame = wrench_scaling_ * mass_inverse_ * 
                ( wrench_scaling_*wrench_error - damping_*end_effector_velocity_task_frame - damping_*Phi );
    std::cout << "- damping_*end_effector_velocity_task_frame" << std::endl << - damping_*end_effector_velocity_task_frame << std::endl;
    
    Phi = Phi + adaptive_sigma*(-wrench_scaling_*wrench_error).cwiseProduct( damping_.diagonal().cwiseInverse() );

    const Eigen::VectorXd velocity_task_frame = end_effector_velocity_task_frame + acceleration_task_frame * delta_t_;
    // transform the velocity from task frame to the base frame: V_b = inv(Adjoint_tb) * V_t
    const Eigen::VectorXd velocity_base_frame = AdjointTransformationMatrix(task_frame_2_base.inverse()) * velocity_task_frame;
    const Eigen::VectorXd velocity_joint_space = jacobian.inverse() * velocity_base_frame;
    std::cout << "velocity_joint_space: " << velocity_joint_space << std::endl;

    return velocity_joint_space;
}


Eigen::VectorXd ForceTorqueController::FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error)
{
    Eigen::AngleAxisd rotation_error(transformation_error.linear());
    Eigen::VectorXd pose_error(6);
    pose_error.head(3) = transformation_error.translation();
    pose_error.tail(3) = rotation_error.axis()*rotation_error.angle();
    return pose_error;
}

Eigen::MatrixXd ForceTorqueController::AdjointTransformationMatrix(const Eigen::Affine3d& transformation)
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

Eigen::Matrix3d ForceTorqueController::SkewSymmetricMatrix(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0,             -1*vector(2),   vector(1), 
                            vector(2),    0,               -1*vector(0), 
                            -1*vector(1), vector(0),      0;
    return skew_symmetric_matrix;
}

Eigen::VectorXd ForceTorqueController::WrenchTruncation(Eigen::VectorXd original_wrench, double force_threshold, double torque_threshold)
{
    Eigen::VectorXd truncated_wrench(6);
    
    for(int i=0;i<3;i++)
    {
        if(original_wrench(i)>force_threshold)
            truncated_wrench(i) = original_wrench(i) - force_threshold;
        else if(original_wrench(i)<-force_threshold)
            truncated_wrench(i) = original_wrench(i) + force_threshold;
        else
            truncated_wrench(i) = 0.0;
    }
    for(int i=3;i<6;i++)
    {
        if(original_wrench(i)>torque_threshold)
            truncated_wrench(i) = original_wrench(i) - torque_threshold;
        else if(original_wrench(i)<-torque_threshold)
            truncated_wrench(i) = original_wrench(i) + torque_threshold;
        else
            truncated_wrench(i) = 0.0;
    }

    return truncated_wrench;
}

Eigen::VectorXd ForceTorqueController::PoseErrorEpsilon(Eigen::VectorXd original_vector, double position_threshold, double orientation_threshold)
{
    Eigen::VectorXd output(6);
    
    for(int i=0;i<3;i++)
    {
        if(original_vector(i)>-position_threshold && original_vector(i)<position_threshold)
            output(i) = 0.0;
        else
            output(i) = original_vector(i);
    }
    for(int i=3;i<6;i++)
    {
        if(original_vector(i)>-orientation_threshold && original_vector(i)<orientation_threshold)
            output(i) = 0.0;
        else
            output(i) = original_vector(i);
    }

    return output;

}

Eigen::Affine3d ForceTorqueController::get_base_2_end_effector()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::Affine3d base_2_end_effector = shared_variable_ptr_->end_effector_state;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return base_2_end_effector;
}

Eigen::VectorXd ForceTorqueController::get_ft_link_wrench()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd ft_link_wrench = shared_variable_ptr_->wrench;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return ft_link_wrench;
}

Eigen::MatrixXd ForceTorqueController::get_jacobian()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::MatrixXd jacobian = shared_variable_ptr_->jacobian;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return jacobian;
}

Eigen::VectorXd ForceTorqueController::get_joint_velocity()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd joint_velocity = shared_variable_ptr_->joint_velocity;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return joint_velocity;
}

Eigen::VectorXd ForceTorqueController::get_joint_states()
{
    pthread_rwlock_wrlock(&shared_variable_ptr_->shared_variables_rwlock);
    const Eigen::VectorXd joint_states = shared_variable_ptr_->joint_states;
    pthread_rwlock_unlock(&shared_variable_ptr_->shared_variables_rwlock);
    return joint_states;
}

bool ForceTorqueController::StaticPointControl(const Eigen::Affine3d task_frame_pose, const Eigen::VectorXd& expected_wrench, bool keep_moving)
{
    cout << "Going to pose: " << endl << task_frame_pose.matrix() << endl;
    const double control_tol = config_tree_.get<double>("control_tol", 0.01);
    ros::Rate loop_rate( int(1.0/config_tree_.get<double>("delta_t", 0.008)) );
    vector<string> str_name = shared_variable_ptr_->joint_names;
    while (ros::ok())
    {
        auto jonit_velocity = ForceVelocityController(task_frame_pose, expected_wrench);
        control_msgs::JointJog joint_deltas;
        for(int i=0;i<jonit_velocity.size();i++)
        {
            joint_deltas.joint_names.push_back(str_name[i]);
            joint_deltas.velocities.push_back(jonit_velocity(i));
        }
        joint_deltas.header.stamp = ros::Time::now();
        jog_vel_pub.publish(joint_deltas);
        loop_rate.sleep();

        if(pose_error_.norm() < control_tol && keep_moving == true) break;
    }
    return true;
}

bool ForceTorqueController::MultiplePointsControl(geometry_msgs::PoseArray cam_to_task_frame_pose_array)
{
    auto expected_wrench_data = AsVector<double>(config_tree_, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);

    tf::StampedTransform transform1, transform2;
    static tf::TransformListener listener(ros::Duration(5));
    listener.waitForTransform("base_link", "wrist_3_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("base_link", "wrist_3_link", ros::Time(0), transform1);
    listener.waitForTransform("wrist_3_link", "camera_color_frame", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("wrist_3_link", "camera_color_frame", ros::Time(0), transform2);
    auto transform = transform1 * transform2;

    Eigen::Affine3d base_2_camera;
    base_2_camera.setIdentity();
    base_2_camera.translate( Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()) );
    base_2_camera.rotate( Eigen::Quaterniond(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()) );

    

    for(auto cam_to_task_frame_pose:cam_to_task_frame_pose_array.poses)
    {
        Eigen::Affine3d eigen_cam_to_task_frame;
        eigen_cam_to_task_frame.setIdentity();
        eigen_cam_to_task_frame.translate( Eigen::Vector3d(cam_to_task_frame_pose.position.x, cam_to_task_frame_pose.position.y, cam_to_task_frame_pose.position.z) );
        eigen_cam_to_task_frame.rotate( Eigen::Quaterniond(cam_to_task_frame_pose.orientation.w, cam_to_task_frame_pose.orientation.x, cam_to_task_frame_pose.orientation.y, cam_to_task_frame_pose.orientation.z) );
        auto task_frame_pose = base_2_camera * eigen_cam_to_task_frame;
        // cout << "base2cam2:" << endl << base_2_camera.matrix() << endl;
        // cout << "eigen_cam_to_task_frame:" << endl << eigen_cam_to_task_frame.matrix() << endl;
        // cout << "task_frame_pose: " << endl << task_frame_pose.matrix() << endl;
        Eigen::Affine3d no_rotation;
        no_rotation.setIdentity();
        no_rotation.translate(task_frame_pose.translation());
        no_rotation.translate(Eigen::Vector3d(0, 0, 0.01));
        cout << "haha task_frame_pose" << endl <<  task_frame_pose.matrix() << endl; 
        cout << "haha no_rotation" << endl <<  no_rotation.matrix() << endl; 
        StaticPointControl(no_rotation, Eigen::VectorXd::Zero(6), true);
        break;
    }
    
    SetZeroVelocity();

    cout << "Press e to proceed." << endl;
    char proceed;
    cin >> proceed;
    if(proceed!='e') throw("Start point not suitable ");

    for(auto cam_to_task_frame_pose:cam_to_task_frame_pose_array.poses)
    {
        Eigen::Affine3d eigen_cam_to_task_frame;
        eigen_cam_to_task_frame.setIdentity();
        eigen_cam_to_task_frame.translate( Eigen::Vector3d(cam_to_task_frame_pose.position.x, cam_to_task_frame_pose.position.y, cam_to_task_frame_pose.position.z) );
        eigen_cam_to_task_frame.rotate( Eigen::Quaterniond(cam_to_task_frame_pose.orientation.w, cam_to_task_frame_pose.orientation.x, cam_to_task_frame_pose.orientation.y, cam_to_task_frame_pose.orientation.z) );
        auto task_frame_pose = base_2_camera * eigen_cam_to_task_frame;
        // cout << "base2cam2:" << endl << base_2_camera.matrix() << endl;
        // cout << "eigen_cam_to_task_frame:" << endl << eigen_cam_to_task_frame.matrix() << endl;
        // cout << "task_frame_pose: " << endl << task_frame_pose.matrix() << endl;
        Eigen::Affine3d no_rotation;
        no_rotation.setIdentity();
        no_rotation.translate(task_frame_pose.translation());
        // StaticPointControl(task_frame_pose);
        StaticPointControl(no_rotation, expected_wrench, true);
    }

    SetZeroVelocity();

    return true;
}

void ForceTorqueController::SetZeroVelocity()
{
    vector<string> str_name = shared_variable_ptr_->joint_names;
    ros::Rate loop_rate(100);
    for(int i=0;i<20;i++)
    {
        control_msgs::JointJog joint_deltas;
        for(int i=0;i<6;i++)
        {
            joint_deltas.joint_names.push_back(str_name[i]);
            joint_deltas.velocities.push_back(0.0);
        }
        joint_deltas.header.stamp = ros::Time::now();
        jog_vel_pub.publish(joint_deltas);
        loop_rate.sleep();
    }
    cout << "SetZeroVelocity()" << endl;
}