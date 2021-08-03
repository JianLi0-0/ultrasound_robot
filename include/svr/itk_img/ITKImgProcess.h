#ifndef ITK_IMG_PROGRESS_H
#define ITK_IMG_PROGRESS_H
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkEuler3DTransform.h"
#include <chrono>
#include <cmath>

#include <algorithm>
// #include <execution>
#include <numeric>
#include <vector>

#include <omp.h>

#include "itkSquaredDifferenceImageFilter.h"
#include "itkStatisticsImageFilter.h"
#include "itkBinaryFunctorImageFilter.h"
#include "itkMeanSquaresImageToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkTranslationTransform.h"
#include "itkImageRegionIterator.h"
#include "itkImageBufferRange.h"

using namespace std;
using VolumeType = itk::Image<unsigned char, 3>;
using SliceType = itk::Image<unsigned char, 2>;
using IntImgType = itk::Image<int, 3>;
using TransformType = itk::Euler3DTransform< double >;
using ExtractFilterType = itk::ExtractImageFilter< VolumeType, VolumeType>;

// using TransformType = itk::VersorRigid3DTransform< double >;
VolumeType::Pointer ExtractSliceFromVolume(VolumeType::Pointer volume, TransformType::Pointer transformation, float sliceWidth, float sliceHeight, VolumeType::SpacingType outputSpacing);
VolumeType::Pointer ScaleVolume(VolumeType::Pointer volume, TransformType::Pointer transformation, float resol_reduction);
double SumOfSquaredDifferences(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo, float sliceWidth, float sliceHeight);
double SSD2(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo, float sliceWidth, float sliceHeight, IntImgType::Pointer& squaredDiffImg);
double SSD3(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo);
double SSD4(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo);
double SSD5(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo);
void SquaredDifferences(VolumeType::Pointer& image, VolumeType::Pointer& image2, IntImgType::Pointer& squaredDiffImg);

namespace Functor
{
template <class TPixel>
class MySquaredDifference
{
public:
  MySquaredDifference() = default;
  ~MySquaredDifference() = default;
  bool
  operator!=(const MySquaredDifference &) const
  {
    return false;
  }

  bool
  operator==(const MySquaredDifference & other) const
  {
    return !(*this != other);
  }

  inline TPixel
  operator()(const TPixel & A, const TPixel & B) const
  {
    const auto   dA = static_cast<double>(A);
    const auto   dB = static_cast<double>(B);
    const double diff = dA - dB;

    return static_cast<TPixel>(diff * diff);
  }
};
} // namespace Functor

using FilterType = itk::BinaryFunctorImageFilter<VolumeType, VolumeType, VolumeType, Functor::MySquaredDifference<VolumeType::PixelType>>;
#endif