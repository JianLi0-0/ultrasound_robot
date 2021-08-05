#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "ITKImgProcess.h"
#include "Optimization.h"

#include "utils/utils.hpp"

#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "CustomRosLib.h"

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

class ThyroidBiopsy
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Publisher joint_velocity_command_pub_;
        ros::Publisher py_sample_command_pub_;
        ros::Subscriber us_img_sub_;
        ForceTorqueController* force_controller_;
        CustomRosLib custom_ros_lib_;

        TransformType::Pointer transformation_;
        Optimization optimization_;
        VolumeType::Pointer volume_;
        VolumeType::SpacingType spacing_;
        float slice_width_;
        float slice_height_;

        sensor_msgs::Image us_image_;
        VolumeType::Pointer itk_us_image_;
        bool is_registration_successful_ = false;
        std::thread cv_thread_;
        Eigen::VectorXd transformation_x0_;


    public:
        
        ThyroidBiopsy(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~ThyroidBiopsy();
        void MainLoop();
        void RegistrationInitialization();
        void UsImageCallback(const sensor_msgs::Image::ConstPtr msg);
        void RegistrationVisualization(sensor_msgs::Image current_image, sensor_msgs::Image result_image, double ssd);
        void CombineImages(cv::Mat & dst, cv::Mat &src1, cv::Mat &src2);
        // void TimerDisplayComparison(const ros::TimerEvent&);
        void TimerDisplayComparison();
        void ROSImageToITK(VolumeType::Pointer& itk_img, sensor_msgs::Image ros_image);
        sensor_msgs::Image ITKImageToROS(VolumeType::Pointer itk_image);
        void SaveImage(VolumeType::Pointer image, string image_name);
};

ThyroidBiopsy::ThyroidBiopsy(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
custom_ros_lib_(this->nh_),
cv_thread_(&ThyroidBiopsy::TimerDisplayComparison, this)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    us_img_sub_ = nh_.subscribe("/us_image", 1, &ThyroidBiopsy::UsImageCallback, this);
    // py_sample_command_pub_ = nh_.advertise<std_msgs::String>("/sample_command", 1);
    force_controller_ = new ForceTorqueController(shared_variable_ptr_, config_tree_);

    nh_.getParam("/svr/w", slice_width_);
    nh_.getParam("/svr/h", slice_height_);

}

ThyroidBiopsy::~ThyroidBiopsy()
{
    std::cout << "~ThyroidBiopsy()" << std::endl;
}

void ThyroidBiopsy::RegistrationInitialization()
{
    transformation_ = TransformType::New();
    transformation_->SetIdentity();

    using ReaderType = itk::ImageFileReader<VolumeType>;
    ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName("/home/sunlab/Desktop/SVR/program/data/volumen/thyroid.mhd");
    reader->Update();
    volume_ = reader->GetOutput();
    volume_ = ScaleVolume(volume_, transformation_, 1);
    spacing_ = volume_->GetSpacing();

    optimization_.SetVolume(volume_);
    optimization_.SetSliceSize(slice_width_, slice_height_);

    itk::Vector<double, 3> t; t[0] = 15;t[1] = 5;t[2] = 40;
    transformation_->SetTranslation(t);
    transformation_->SetRotation(0.0,0.0,0.0);
    itk_us_image_ = ExtractSliceFromVolume(volume_, transformation_, slice_width_, slice_height_, spacing_);
}

void ThyroidBiopsy::MainLoop()
{
    RegistrationInitialization();

    ROSImageToITK(itk_us_image_, us_image_);
    
    // transformation_->SetRotation(0.113,0.123,0.133);
    
    transformation_x0_ =  optimization_.ITKTransformToEigen(transformation_);
    auto timer = CustomTimer();
    timer.tic();
    auto success = optimization_.Optimize(itk_us_image_, transformation_x0_);
    auto elapsed_seconds = timer.toc(true);
    
    if (success) {
        std::cout << "Registration succeeded." << std::endl;
        is_registration_successful_ = true;
    } else {
        std::cout << "Failed." << std::endl;
    }
    std::cout << "solution :\n" << transformation_x0_ << std::endl;

    ros::waitForShutdown();
    cv_thread_.join();
}

void ThyroidBiopsy::TimerDisplayComparison()
// void ThyroidBiopsy::TimerDisplayComparison(const ros::TimerEvent&)
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        if(is_registration_successful_)
        {
            // itk::Vector<double, 3> t; t[0] = 15;t[1] = 5;t[2] = 40;
            // transformation_->SetTranslation(t);
            // transformation_->SetRotation(0.0,0.0,0.0);
            // transformation_x0_ =  optimization_.ITKTransformToEigen(transformation_);
            auto solution_slice = ExtractSliceFromVolume(volume_, optimization_.EigenToITKTransform(transformation_x0_), slice_width_, slice_height_, spacing_);
            // cout << "SSD: " << SSD3(itk_us_image_, solution_slice) << endl;
            RegistrationVisualization(us_image_, ITKImageToROS(solution_slice), SSD3(itk_us_image_, solution_slice));
            cv::waitKey(1);
            // is_registration_successful_ = false;
            // SaveImage(solution_slice, "solution_slice.png");
        }
        rate.sleep();
    }
}

void ThyroidBiopsy::ROSImageToITK(VolumeType::Pointer& itk_img, sensor_msgs::Image ros_image)
{
    using IteratorType = itk::ImageRegionIterator< VolumeType >;
    IteratorType iterator( itk_img, itk_img->GetRequestedRegion() );
    
    assert( iterator.GetImageIteratorDimension() == ros_image.data.size() );
    if(iterator.GetImageIteratorDimension() == ros_image.data.size())
        cout << "iterator.GetImageIteratorDimension() == ros_image.data.size()" << endl;
    cout << "iterator.GetImageIteratorDimension():" << iterator.GetImageIteratorDimension() << "  ros_image.data.size():" << ros_image.data.size() << endl;
    auto in = ros_image.data.begin();
    for (iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator, ++in)
    {
        iterator.Set( *in );
    }
}

sensor_msgs::Image ThyroidBiopsy::ITKImageToROS(VolumeType::Pointer itk_image)
{
    sensor_msgs::Image ros_image;
    ros_image = us_image_;
    auto out = ros_image.data.begin();

    using IteratorType = itk::ImageRegionIterator< VolumeType >;
    IteratorType iterator( itk_image, itk_image->GetRequestedRegion() );
    for (iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator, ++out)
    {
        *out = iterator.Get();
    }

    return ros_image;
}


void ThyroidBiopsy::UsImageCallback(const sensor_msgs::Image::ConstPtr msg)
{
    us_image_ = *msg;
}

void ThyroidBiopsy::CombineImages(cv::Mat & dst, cv::Mat &src1, cv::Mat &src2)
{
    int rows = src1.rows;
    int cols = src1.cols+src2.cols;
    CV_Assert(src1.type () == src2.type ());
    dst.create (rows,cols,src1.type ());
    src1.copyTo (dst(cv::Rect(0,0,src1.cols,src1.rows)));
    src2.copyTo (dst(cv::Rect(src1.cols,0,src2.cols,src2.rows)));
}

void ThyroidBiopsy::RegistrationVisualization(sensor_msgs::Image current_image, sensor_msgs::Image result_image, double ssd)
{
    cv_bridge::CvImagePtr current_cv_ptr;
    cv_bridge::CvImagePtr result_cv_ptr;
    try
    {
        current_cv_ptr = cv_bridge::toCvCopy(current_image, sensor_msgs::image_encodings::TYPE_8UC1);
        result_cv_ptr = cv_bridge::toCvCopy(result_image, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    string str1 = "SSD: ";
    char ch[256];
    sprintf(ch, "%lf", ssd);
    string str2 = ch;
    string str = str1 + str2;
    cv::putText(result_cv_ptr->image,
            str,
            cv::Point(result_cv_ptr->image.cols / 20, result_cv_ptr->image.rows / 1.05), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(255, 255, 255), //font color
            0.5);
    cv::Mat comparison;
    CombineImages(comparison, current_cv_ptr->image, result_cv_ptr->image);
    cv::imshow("OPENCV_WINDOW", comparison);
}

void ThyroidBiopsy::SaveImage(VolumeType::Pointer image, string image_name)
{
    using WriterType = itk::ImageFileWriter<VolumeType>;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(image_name);
    writer->SetInput(image);
    writer->Update();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thyroid_biopsy");
    ros::NodeHandle nh;
    
    pt::ptree root;
	pt::read_json("src/ultrasound_robot/config/force_controller.json", root);
    
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    shared_variable_ptr->config_tree = root;

    ThyroidBiopsy thyroid_biopsy(shared_variable_ptr, root);
    
    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();

    // StatesHub states_hub(shared_variable_ptr);
    sleep(1);

    thyroid_biopsy.MainLoop();

    return 0;

}