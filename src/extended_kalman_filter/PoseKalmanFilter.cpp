#include "PoseKalmanFilter.h"

PoseKalmanFilter::PoseKalmanFilter()
{

}

PoseKalmanFilter::~PoseKalmanFilter()
{
}

void PoseKalmanFilter::InitializeKalmanFilter()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12,12);
    A.block(6,6,6,6) = A.block(6,6,6,6) * dt_;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(12,1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6,12);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12,12) * process_noise_;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6,6) * measure_noise_;
    
    A_ = EigenToArma(A);
    B_ = EigenToArma(B);
    C_ = EigenToArma(C);
    Q_ = EigenToArma(Q);
    R_ = EigenToArma(R);

    // std::cout << "Kalman Filter Initializaton" << std::endl;
    // std::cout << "A_: " << std::endl << A_ << std::endl;
    // std::cout << "B_: " << std::endl << B_ << std::endl;
    // std::cout << "C_: " << std::endl << C_ << std::endl;
    // std::cout << "Q_: " << std::endl << Q_ << std::endl;
    // std::cout << "R_: " << std::endl << R_ << std::endl;

    kf_ = new kf::KalmanFilter(A_, B_, C_);
    kf_->setProcessCovariance(Q_);
    kf_->setOutputCovariance(R_);
    std::cout << "Kalman Filter Initializaton Done" << std::endl;
}

void PoseKalmanFilter::UpdateState(const Eigen::Affine3d& pose)
{
    auto y = FromeMatrixToErrorAxisAngle(pose);
    arma::mat arma_y = {y(0), y(1), y(2), y(3), y(4), y(5)};
    kf_->updateState({0}, arma_y.as_col());
}

void PoseKalmanFilter::UpdateStateEulerAngle(const Eigen::VectorXd& position_euler_angle)
{
    auto y = position_euler_angle;
    arma::mat arma_y = {y(0), y(1), y(2), y(3), y(4), y(5)};
    kf_->updateState({0}, arma_y.as_col());
}

Eigen::Affine3d PoseKalmanFilter::GetEstimate()
{
    Eigen::Affine3d output;
    auto arma_x_hat = kf_->getEstimate();
    output.translation() << arma_x_hat(0), arma_x_hat(1), arma_x_hat(2);
    Eigen::AngleAxisd angle_axisd;
    auto angleXaxisd = Eigen::Vector3d(arma_x_hat(3), arma_x_hat(4), arma_x_hat(5));
    angle_axisd.angle() = angleXaxisd.norm();
    angle_axisd.axis() = angleXaxisd.normalized();
    output.linear() = angle_axisd.toRotationMatrix();
    // std::cout << "arma_x_hat: " << std::endl << arma_x_hat << std::endl;
    // std::cout << "output: " << std::endl << output.matrix() << std::endl;
    return output;
}

Eigen::VectorXd PoseKalmanFilter::GetEstimateState()
{
    auto arma_x_hat = kf_->getEstimate();
    Eigen::VectorXd output(6);
    output << arma_x_hat(0), arma_x_hat(1), arma_x_hat(2), arma_x_hat(3), arma_x_hat(4), arma_x_hat(5);
    return output;
}

Eigen::Affine3d PoseKalmanFilter::GetEstimateFromEluerAngle()
{
    auto arma_x_hat = kf_->getEstimate();
    Eigen::VectorXd position_euler_angle(6);
    position_euler_angle << arma_x_hat(0), arma_x_hat(1), arma_x_hat(2), arma_x_hat(3), arma_x_hat(4), arma_x_hat(5);
    Eigen::Affine3d output = Position_Eluer_Angle_To_Affine3d(position_euler_angle);
    return output;
}

Eigen::VectorXd PoseKalmanFilter::NextStatePrediction(const double prediction_time)
{
    Eigen::VectorXd output;
    auto arma_x_hat = kf_->getEstimate();
    Eigen::VectorXd state(6);
    Eigen::VectorXd state_dot(6);
    state << arma_x_hat(0), arma_x_hat(1), arma_x_hat(2), arma_x_hat(3), arma_x_hat(4), arma_x_hat(5);
    state_dot <<arma_x_hat(6), arma_x_hat(7), arma_x_hat(8), arma_x_hat(9), arma_x_hat(10), arma_x_hat(11);
    output = state + prediction_time * state_dot;
    return output;
}

Eigen::VectorXd PoseKalmanFilter::FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error)
{
    Eigen::AngleAxisd rotation_error(transformation_error.linear());
    Eigen::VectorXd pose_error(6);
    pose_error.head(3) = transformation_error.translation();
    pose_error.tail(3) = rotation_error.axis()*rotation_error.angle();
    return pose_error;
}

void PoseKalmanFilter::SetInitialXhat(const Eigen::Affine3d& X)
{
    auto y = FromeMatrixToErrorAxisAngle(X);
    arma::mat arma_y = {y(0), y(1), y(2), y(3), y(4), y(5), 0, 0, 0, 0, 0, 0};
    kf_->setEstimate(arma_y.as_col());
}

void PoseKalmanFilter::SetInitialXhatEluerAngle(const Eigen::VectorXd& position_euler_angle)
{
    auto y = position_euler_angle;
    arma::mat arma_y = {y(0), y(1), y(2), y(3), y(4), y(5), 0, 0, 0, 0, 0, 0};
    kf_->setEstimate(arma_y.as_col());
}

Eigen::Affine3d PoseKalmanFilter::Position_Eluer_Angle_To_Affine3d(const Eigen::VectorXd& position_euler_angle)
{
    // ZYX
    Eigen::Affine3d X = Eigen::Affine3d::Identity();
	X.translate( Eigen::Vector3d( position_euler_angle(0), position_euler_angle(1), position_euler_angle(2) ));
	X.rotate( Eigen::Quaterniond( Eigen::AngleAxisd(position_euler_angle(3),Eigen::Vector3d::UnitX()) * 
                                Eigen::AngleAxisd(position_euler_angle(4),Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(position_euler_angle(5),Eigen::Vector3d::UnitZ())  ) );

    return X;
}

Eigen::VectorXd PoseKalmanFilter::Position_Axis_Angle_To_Position_Eluer_Angle(const Eigen::VectorXd& position_axis_angle)
{
    Eigen::VectorXd vecX(6);
    return vecX;
}