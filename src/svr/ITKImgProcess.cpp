#include "ITKImgProcess.h"

VolumeType::Pointer ExtractSliceFromVolume(VolumeType::Pointer volume, TransformType::Pointer transformation, float sliceWidth, float sliceHeight, VolumeType::SpacingType outputSpacing)
{
    typename VolumeType::SizeType inputSize;
    inputSize[0] = sliceWidth; inputSize[1] = sliceHeight; inputSize[2] = 1;

    using LinearInterpolatorType = itk::LinearInterpolateImageFunction<VolumeType, double>;
    typename LinearInterpolatorType::Pointer interpolator = LinearInterpolatorType::New();
    using ResampleFilterType = itk::ResampleImageFilter<VolumeType, VolumeType>;
    typename ResampleFilterType::Pointer resampleFilter = ResampleFilterType::New();

    resampleFilter->SetInput(volume);
    resampleFilter->SetTransform(transformation);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetSize(inputSize);
    resampleFilter->SetOutputSpacing(outputSpacing);
    // resampleFilter->SetOutputOrigin(inputOrigin);

    // auto begin = std::chrono::high_resolution_clock::now();
    resampleFilter->Update();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    // cout<<"Extract Update(): "<< elapsed.count() * 1e-9 <<endl;
    
    return resampleFilter->GetOutput();
}

VolumeType::Pointer ScaleVolume(VolumeType::Pointer volume, TransformType::Pointer transformation, float resol_reduction)
{
    VolumeType::RegionType  inputRegion = volume->GetLargestPossibleRegion();
    VolumeType::SizeType    inputSize = inputRegion.GetSize();
    VolumeType::SpacingType inputSpacing = volume->GetSpacing();

    using LinearInterpolatorType = itk::LinearInterpolateImageFunction<VolumeType, double>;
    typename LinearInterpolatorType::Pointer interpolator = LinearInterpolatorType::New();
    using ResampleFilterType = itk::ResampleImageFilter<VolumeType, VolumeType>;
    typename ResampleFilterType::Pointer resampleFilter = ResampleFilterType::New();
    cout << inputSize << endl;
    cout << inputSpacing << endl;
    inputSize[0]=inputSize[0]/resol_reduction; inputSize[1]=inputSize[1]/resol_reduction; inputSize[2]=inputSize[2]/resol_reduction;
    inputSpacing[0]=inputSpacing[0]*resol_reduction; inputSpacing[1]=inputSpacing[1]*resol_reduction; inputSpacing[2]=inputSpacing[2]*resol_reduction;
    cout << inputSize << endl;
    cout << inputSpacing << endl;
    resampleFilter->SetInput(volume);
    resampleFilter->SetTransform(transformation);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetSize(inputSize);
    resampleFilter->SetOutputSpacing(inputSpacing);
    // resampleFilter->SetOutputOrigin(inputOrigin);

    // auto begin = std::chrono::high_resolution_clock::now();
    resampleFilter->Update();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    // cout<<"Extract Update(): "<< elapsed.count() * 1e-9 <<endl;
    
    return resampleFilter->GetOutput();

}

double SumOfSquaredDifferences(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo, float sliceWidth, float sliceHeight)
{
    itk::Index<3U> idx;
    idx[2] = 0;
    double sum = 0;
    for(int i=0; i<sliceWidth; i++)
    {
        idx[0] = i;
        for(int j=0; j<sliceHeight; j++)
        {
            idx[1] = j;
            sum += pow(imageOne->GetPixel(idx)-imageTwo->GetPixel(idx), 2);
        }
    }
    
    return sum;
}

struct accum_sum_of_squares {
    // x contains the sum-of-squares so far, y is the next value.
    int operator()(int x, int y) const {
        return x + y;
    }
};

double SSD2(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo, float sliceWidth, float sliceHeight, IntImgType::Pointer& squaredDiffImg)
{
    // using FloatImageType = itk::Image<float, 3>;
    // using SDImageFilterType = itk::SquaredDifferenceImageFilter<VolumeType, VolumeType, FloatImageType>;
    // SDImageFilterType::Pointer squaredDifferenceFilter = SDImageFilterType::New();
    // squaredDifferenceFilter->SetInput1(imageOne);
    // squaredDifferenceFilter->SetInput2(imageTwo);
    // squaredDifferenceFilter->Update();
    // auto squaredDifference = squaredDifferenceFilter->GetOutput();
    
    // auto begin = std::chrono::high_resolution_clock::now();
    SquaredDifferences(imageOne, imageTwo, squaredDiffImg);
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    // cout<<"SD: "<< elapsed.count() * 1e-9 <<endl;


    // begin = std::chrono::high_resolution_clock::now();

    // using StatisticsImageFilterType = itk::StatisticsImageFilter<IntImgType>;
    // StatisticsImageFilterType::Pointer statisticsImageFilter = StatisticsImageFilterType::New ();
    // statisticsImageFilter->SetInput(squaredDiffImg);
    // statisticsImageFilter->Update();
    // auto sum = statisticsImageFilter->GetSum();

    double sum = 0;
    // itk::ImageBufferRange<VolumeType> range;
    itk::ImageBufferRange<IntImgType> bufferRange{ *squaredDiffImg };
    // sum = std::reduce(std::execution::par, bufferRange.begin(), bufferRange.end(), 0.0);
    auto aa = accumulate(bufferRange.begin(), bufferRange.end(), 0); //, accum_sum_of_squares()
    sum = double(aa);
    // squaredDiffImg


    // elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    // cout<<"sum: "<< elapsed.count() * 1e-9 <<endl;

    // double sum = 0;
    // using IteratorType = itk::ImageRegionIterator<FloatImageType>;
    // IteratorType it( squaredDifference, squaredDifference->GetRequestedRegion() );
    // it.GoToBegin();
    // while (!it.IsAtEnd())
    // {
    //     sum += it.Get();
    //     ++it;
    // }
    
    return sum;
}

struct accum_sum_of_squared_differences {
    // x contains the sum-of-squares so far, y is the next value.
    int operator()(int x, int y) const {
        auto temp = x - y;
        return temp * temp;
    }
};

int myaccumulator(int x, int y)
{
    return x + y;
}
int myproduct(int x, int y)
{
    auto temp = x - y;
    return temp * temp;
}

double SSD3(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo)
{
    // auto begin = std::chrono::high_resolution_clock::now();
    itk::ImageBufferRange<VolumeType> bufferRange1{ *imageOne };
    itk::ImageBufferRange<VolumeType> bufferRange2{ *imageTwo };
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    // cout<<"SD bufferRange: "<< elapsed.count() * 1e-9 <<endl;
    auto sum = std::inner_product(bufferRange1.begin(), bufferRange1.end(), bufferRange2.begin(), 0, myaccumulator, myproduct);
    // auto sum = std::transform_reduce(std::execution::par_unseq, bufferRange1.begin(), bufferRange1.end(), bufferRange2.begin(), 0, myaccumulator, myproduct);
    return double(sum);
}

double SSD4(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo)
{
    itk::ImageBufferRange<VolumeType> bufferRange1{ *imageOne };
    itk::ImageBufferRange<VolumeType> bufferRange2{ *imageTwo };
    int sum = 0;
    auto it_1 = bufferRange1.begin(),it_2 = bufferRange2.begin();
    for(int i = 0; i<bufferRange1.size(); ++i)
    {
        auto temp = *(it_1+i) - *(it_2+i);
        sum += temp * temp;
    }
    cout << "no omp ";
    return double(sum);
}

double SSD5(VolumeType::Pointer imageOne, VolumeType::Pointer imageTwo)
{
    itk::ImageBufferRange<VolumeType> bufferRange1{ *imageOne };
    itk::ImageBufferRange<VolumeType> bufferRange2{ *imageTwo };
    int sum = 0;
    auto it_1 = bufferRange1.begin(),it_2 = bufferRange2.begin();
    #pragma omp parallel for num_threads(2) reduction(+:sum)
    for(int i = 0; i<bufferRange1.size(); i++)
    {
        auto temp = *(it_1+i) - *(it_2+i);
        sum += temp * temp;
    }
    cout << "omp ";
    return double(sum);
}

void SquaredDifferences(VolumeType::Pointer& image, VolumeType::Pointer& image2, IntImgType::Pointer& squaredDiffImg)
{    
    itk::MultiThreaderBase::Pointer mt = itk::MultiThreaderBase::New();
    // ParallelizeImageRegion invokes the provided lambda function in parallel
    // each invocation will contain a piece of the overall region
    mt->ParallelizeImageRegion<3>(
        image->GetBufferedRegion(),
        // here we creat an ad-hoc lambda function to process the region pieces
        // the lambda will have access to variable 'image' from the outer function
        // it will have parameter 'region', which needs to be processed
        [image, image2, squaredDiffImg](const VolumeType::RegionType & region) {
        itk::ImageRegionIterator<VolumeType> it(image, region);
        itk::ImageRegionIterator<VolumeType> it2(image2, region);
        itk::ImageRegionIterator<IntImgType> it3(squaredDiffImg, region);
        for (; !it.IsAtEnd(); ++it, ++it2, ++it3)
        {
            auto diff = it.Get()-it2.Get();
            it3.Set(diff*diff);
        }
        },
        nullptr); // we don't have a filter whose progress needs to be updated
}