#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

#include "ITKImgProcess.h"

class Optimization
{
    private:
        VolumeType::Pointer volume;
        VolumeType::Pointer slice;
        VolumeType::SpacingType outputSpacing;
        FilterType::Pointer filter;
        float sliceWidth;
        float sliceHeight;

    public:
        Optimization();
        ~Optimization();
        void SetVolume(VolumeType::Pointer volume);
        void SetSlice(VolumeType::Pointer slice);
        IntImgType::Pointer squaredDiffImg;
        IntImgType::Pointer GetSquaredDiffImg();
        TransformType::Pointer EigenToITKTransform(const Eigen::VectorXd eigenVec);
        Eigen::VectorXd ITKTransformToEigen(const TransformType::Pointer tf);
        double SimilaritySSDFcn(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data);
        bool Optimize(VolumeType::Pointer goalSlice, Eigen::VectorXd& initialTransform, double rel_sol_change_tol, double rel_obj_change_tol);
        void SetSliceSize(float width, float height);
};


#endif