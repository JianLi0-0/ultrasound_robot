#ifndef Pose_Kalman_Filter_H
#define Pose_Kalman_Filter_H
#include "kalman_filter.h"
#include <Eigen/Eigen>

class PoseKalmanFilter
{
    private:
        arma::mat A_;
        arma::mat B_;
        arma::mat C_;
        arma::mat Q_;
        arma::mat R_;
        kf::KalmanFilter* kf_;
        double dt_;
        double process_noise_;
        double measure_noise_;

    public:
        PoseKalmanFilter();
        ~PoseKalmanFilter();
        void InitializeKalmanFilter();
        void UpdateState(const Eigen::Affine3d& pose);
        Eigen::Affine3d GetEstimate();
        Eigen::VectorXd FromeMatrixToErrorAxisAngle(const Eigen::Affine3d& transformation_error);
        arma::mat EigenToArma(Eigen::MatrixXd eigen_A) {
            arma::mat arma_B = arma::mat(eigen_A.data(), eigen_A.rows(), eigen_A.cols(), true, true);
            return arma_B;
        }
        void SetParammeters(const double &dt, const double &process_noise, const double &measure_noise) {
            process_noise_ = process_noise; 
            measure_noise_ = measure_noise;
            dt_ = dt;
        }
        void SetInitialXhat(const Eigen::Affine3d& X);
};

#endif