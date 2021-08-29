#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "ITKImgProcess.h"
#include "Optimization.h"

double ackley_fn(const Eigen::VectorXd& vals_inp, Eigen::VectorXd* grad_out, void* opt_data)
{
    const double x = vals_inp(0);
    const double y = vals_inp(1);
    const double pi = 3.1415926;
    double obj_val = -20*std::exp( -0.2*std::sqrt(0.5*(x*x + y*y)) ) - std::exp( 0.5*(std::cos(2*pi*x) + std::cos(2*pi*y)) ) + 22.718282L;
    return obj_val;
}


int main(int, char * argv[])
{
    // using PixelType = unsigned char;
    // constexpr unsigned int Dimension = 3;
    // using ImageType = itk::Image<PixelType, Dimension>;
    using ReaderType = itk::ImageFileReader<VolumeType>;
    ReaderType::Pointer reader = ReaderType::New();
    // const char * filename = argv[1];
    reader->SetFileName("/home/kuka/SVR/program/data/volumen/thyroid.mhd");
    reader->Update();

    TransformType::Pointer transformation = TransformType::New();
    transformation->SetIdentity();

    VolumeType::Pointer volume = reader->GetOutput();
    volume = ScaleVolume(volume, transformation, 1);

    transformation->SetRotation(0.11,0.12,0.13);
    itk::Vector<double, 3> t;
    t[0] = 15;t[1] = 5;t[2] = 40;
    transformation->SetTranslation(t);

    const typename VolumeType::SpacingType spacing = volume->GetSpacing();
    
    // std::cout << transformation << std::endl;
    float sliceWidth = 100;
    float sliceHeight = 150;
    SliceType::SpacingType outputSpacing;
    // outputSpacing[0] = 
    // try
    // {
        // std::cout << volume << std::endl;


    Optimization optimization;
    optimization.SetSliceSize(sliceWidth, sliceHeight);

    cout<<"start:"<<endl;
    auto begin = std::chrono::high_resolution_clock::now();
    auto V1 = ExtractSliceFromVolume(volume, transformation, sliceWidth, sliceHeight, spacing);
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    cout<<"Extract runtime:   "<< elapsed.count() * 1e-9 <<endl;

    t[2] = 60;
    transformation->SetTranslation(t);
    auto V2 = ExtractSliceFromVolume(volume, transformation, sliceWidth, sliceHeight, spacing);

    begin = std::chrono::high_resolution_clock::now();
    auto sum = SSD3(V1, V2);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    cout<<"SSD3 runtime:   "<< elapsed.count() * 1e-9 <<endl;
    cout << "sum: " << sum << endl;

    begin = std::chrono::high_resolution_clock::now();
    sum = SSD4(V1, V2);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    cout<<"SSD4 runtime:   "<< elapsed.count() * 1e-9 <<endl;
    cout << "sum: " << sum << endl;

    begin = std::chrono::high_resolution_clock::now();
    sum = SSD5(V1, V2);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    cout<<"SSD5 runtime:   "<< elapsed.count() * 1e-9 <<endl;
    cout << "sum: " << sum << endl;
    

    
    // using WriterType = itk::ImageFileWriter<VolumeType>;
    // typename WriterType::Pointer writer = WriterType::New();
    // writer->SetFileName("/Users/lijian/Local/ITK/program/data/volumen/new_thyroid.mhd");
    // writer->SetInput(tempV);
    // try
    // {
    //     writer->Update();
    // }
    // catch (itk::ExceptionObject & error)
    // {
    //     std::cerr << "Error: " << error << std::endl;
    //     return EXIT_FAILURE;
    // }


    // Optimization optimization;
    VolumeType::Pointer goalSlice = V1;
    Eigen::VectorXd initialTransform;
    optimization.SetVolume(volume);
    t[2] = 40;
    transformation->SetTranslation(t);
    cout << "Ground Truth: \n " << optimization.ITKTransformToEigen(transformation) << endl;
    TransformType::Pointer stored_ground_truth = TransformType::New();
    stored_ground_truth->SetParameters(transformation->GetParameters());
    t[0] = 16; t[1] = 5; t[2] = 40;  transformation->SetRotation(0.11,0.12,0.13);
    transformation->SetTranslation(t);
    initialTransform =  optimization.ITKTransformToEigen(transformation);
    cout << "initialTransform: \n " << initialTransform << endl;

    auto V4 = ExtractSliceFromVolume(volume, optimization.EigenToITKTransform(initialTransform), sliceWidth, sliceHeight, spacing);
    
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    auto success = optimization.Optimize(goalSlice, initialTransform, 1E-3, 1E-3);
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    
    if (success) {
        std::cout << "Registration successfully.\n";
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"
                << "errors: \n" << initialTransform - optimization.ITKTransformToEigen(stored_ground_truth) << std::endl;
    } else {
        std::cout << "Failed." << std::endl;
    }

    std::cout << "\n solution :\n" << initialTransform << std::endl;

    auto V3 = ExtractSliceFromVolume(volume, optimization.EigenToITKTransform(initialTransform), sliceWidth, sliceHeight, spacing);
    cout << "SSD: " << SSD3(V1, V3) << endl;
    using WriterType = itk::ImageFileWriter<VolumeType>;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName("current.png");
    writer->SetInput(V1);
    writer->Update();
    writer->SetFileName("result.png");
    writer->SetInput(V3);
    writer->Update();
    writer->SetFileName("init.png");
    writer->SetInput(V4);
    writer->Update();




    return 0;
}
