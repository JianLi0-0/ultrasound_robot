#include "Optimization.h"

Optimization::Optimization():squaredDiffImg(IntImgType::New()), filter(FilterType::New())
{
    // using ReaderType = itk::ImageFileReader<VolumeType>;
    // ReaderType::Pointer reader = ReaderType::New();
    // reader->SetFileName("/home/sunlab/Desktop/SVR/program/data/volumen/thyroid.mhd");
    // reader->Update();
    // volume = reader->GetOutput();
    // outputSpacing = volume->GetSpacing();

    IntImgType::IndexType start;
    start[0] = 0; start[1] = 0; start[2] = 0; // first index on Z
    IntImgType::SizeType size;
    size[0] = sliceWidth;  size[1] = sliceHeight;  size[2] = 1;   // size along Z
    IntImgType::RegionType region;
    region.SetSize(size);
    region.SetIndex(start);
    squaredDiffImg->SetRegions(region);
    squaredDiffImg->Allocate();
}
Optimization::~Optimization()
{
}

void Optimization::SetVolume(VolumeType::Pointer volumePtr)
{
    volume = volumePtr;
    outputSpacing = volume->GetSpacing();
}

void Optimization::SetSlice(VolumeType::Pointer slicePtr)
{
    slice = slicePtr;
}

IntImgType::Pointer Optimization::GetSquaredDiffImg()
{
    return squaredDiffImg;
}

TransformType::Pointer Optimization::EigenToITKTransform(const Eigen::VectorXd eigenVec)
{
    TransformType::Pointer transformation = TransformType::New();
    itk::Vector<double, 3> t;
    t[0] = eigenVec(0);t[1] = eigenVec(1);t[2] = eigenVec(2);
    transformation->SetTranslation(t);
    transformation->SetRotation(eigenVec(3),eigenVec(4),eigenVec(5));
    return transformation;
}

Eigen::VectorXd Optimization::ITKTransformToEigen(const TransformType::Pointer tf)
{
    Eigen::VectorXd eigenVec = Eigen::VectorXd::Zero(6);
    auto t = tf->GetTranslation();
    eigenVec << t[0], t[1], t[2], tf->GetAngleX(), tf->GetAngleY(), tf->GetAngleZ();
    return eigenVec;
}

double Optimization::SimilaritySSDFcn(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
{
    // std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();


    auto transformation = this->EigenToITKTransform(x);
    auto tempSlice = ExtractSliceFromVolume(this->volume, transformation, this->sliceWidth, this->sliceHeight, this->outputSpacing);
    // auto metric = SumOfSquaredDifferences(this->slice, tempSlice, this->sliceWidth, this->sliceHeight);
    auto metric = SSD3(this->slice, tempSlice);

    // std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end-start;

    // std::cout <<  "SimilaritySSDFcn elapsed time: " << elapsed_seconds.count() << endl;
    return metric;
}

// double haha(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
// {
//     return 0;
// }

bool Optimization::Optimize(VolumeType::Pointer goalSlice, Eigen::VectorXd& initialTransform)
{
    slice = goalSlice;
    std::function<double(const Eigen::VectorXd&,Eigen::VectorXd*,void*)> func = std::bind(&Optimization::SimilaritySSDFcn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    bool success = optim::nm(initialTransform, func, nullptr);
    return success;
}


void Optimization::SetSliceSize(float width, float height)
{
    sliceWidth = width;
    sliceHeight = height;
}
