#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include <math.h>

#include "ITKImgProcess.h"
#include "Optimization.h"
#include "utils/utils.hpp"
#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "CustomRosLib.h"
#include "VisualServo.h"
#include "PoseKalmanFilter.h"

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <ultrasound_robot/svrConfig.h>
#include "geometry_msgs/WrenchStamped.h"

class ThyroidBiopsy
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Publisher joint_velocity_command_pub_;
        ros::Subscriber us_img_sub_;
        ForceTorqueController* force_controller_;
        CustomRosLib* custom_ros_lib_;
		VisualServo* visual_servo_;
        PoseKalmanFilter pkf_;

        TransformType::Pointer transformation_;
        Optimization optimization_;
        VolumeType::Pointer volume_;
        VolumeType::SpacingType spacing_;
        float slice_width_;
        float slice_height_;
        Eigen::Affine3d target_slice_pose_;
        Eigen::Affine3d current_slice_pose_;
        Eigen::Affine3d probe_to_target_;
        Eigen::VectorXd expected_wrench_;
        std::thread control_thread_;
        ros::Time registration_time_stamp_;
        std::vector<double> initial_joint_angle_;
        bool end_control_ = false;

        sensor_msgs::Image us_image_;
        VolumeType::Pointer probe_itk_us_image_, itk_us_image_,taget_slice_;
        bool is_registration_successful_ = false;
        std::thread cv_thread_;
        Eigen::VectorXd transformation_x0_;
        itk::Vector<double, 3> t_; 
        double volume_resol_reduction_ = 1;
        double elasped_time_ = 1;

        dynamic_reconfigure::Server<ultrasound_robot::svrConfig> dynamic_reconfig_server_;
        dynamic_reconfigure::Server<ultrasound_robot::svrConfig>::CallbackType dynamic_reconfig_f_;
        ultrasound_robot::svrConfig dynamic_config_;


    public:
        
        ThyroidBiopsy(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~ThyroidBiopsy();
        void MainLoop();
        void RegistrationInitialization();
        void RobotInitialization();
        void UsImageCallback(const sensor_msgs::Image::ConstPtr msg);
        void RegistrationVisualization(sensor_msgs::Image current_image, sensor_msgs::Image result_image, double ssd);
        void CombineImages(cv::Mat & dst, cv::Mat &src1, cv::Mat &src2);
        void ControlLoop();
        void DisplayComparisonLoop();
        void ROSImageToITK(VolumeType::Pointer& itk_img, sensor_msgs::Image ros_image);
        sensor_msgs::Image ITKImageToROS(VolumeType::Pointer itk_image);
        void SaveImage(VolumeType::Pointer image, string image_name);
        void DynamicReconfigCallback(ultrasound_robot::svrConfig &config, uint32_t level);
        Eigen::Affine3d MmToM(Eigen::Affine3d tf);
};

ThyroidBiopsy::ThyroidBiopsy(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
cv_thread_(&ThyroidBiopsy::DisplayComparisonLoop, this),
control_thread_(&ThyroidBiopsy::ControlLoop, this)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    us_img_sub_ = nh_.subscribe("/us_image", 1, &ThyroidBiopsy::UsImageCallback, this);
    force_controller_ = new ForceTorqueController(shared_variable_ptr_, config_tree_);
    custom_ros_lib_ = new CustomRosLib(shared_variable_ptr_);
	visual_servo_ = new VisualServo(shared_variable_ptr_);
    double lamda; nh_.getParam("/svr/visual_servo_lamda", lamda);
	visual_servo_->set_lambda(lamda);
    visual_servo_->set_link_name_("ee_link", "probe");

    nh_.getParam("/svr/servo_w", slice_width_);
    nh_.getParam("/svr/servo_h", slice_height_);
    nh_.getParam("/svr/volume_resol_reduction", volume_resol_reduction_);
    nh_.getParam("/svr/initial_pose", initial_joint_angle_);
    // slice_width_ = slice_width_ / volume_resol_reduction_;
    // slice_height_ = slice_height_ / volume_resol_reduction_;
    // cout << "slice_width: " << slice_width_ << " slice_height: " << slice_height_ << endl;
    dynamic_reconfig_f_ = boost::bind(&ThyroidBiopsy::DynamicReconfigCallback, this, _1, _2);
    dynamic_reconfig_server_.setCallback(dynamic_reconfig_f_);

    auto expected_wrench_data = AsVector<double>(config_tree_, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);
    expected_wrench_ = expected_wrench;

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
    reader->SetFileName("/home/sunlab/Desktop/lee_ws/src/ultrasound_robot/src/python/new_biopsy_single.mha");
    reader->Update();
    volume_ = reader->GetOutput();
    volume_ = ScaleVolume(volume_, transformation_, volume_resol_reduction_);
    spacing_ = volume_->GetSpacing();

    optimization_.SetVolume(volume_);
    optimization_.SetSliceSize(slice_width_, slice_height_);

    // t_[0] = 100*spacing_[0]/volume_resol_reduction_;t_[1] = 120*spacing_[0]/volume_resol_reduction_;t_[2] = 300*spacing_[0]/volume_resol_reduction_;
    t_[0] = dynamic_config_.tx; t_[1] = dynamic_config_.ty; t_[2] = dynamic_config_.tz;
    transformation_->SetTranslation(t_);
    transformation_->SetRotation(dynamic_config_.rx, dynamic_config_.ry, dynamic_config_.rz);
    transformation_x0_ =  optimization_.ITKTransformToEigen(transformation_); // Initialization variables x0 for optimization
    itk_us_image_ = ExtractSliceFromVolume(volume_, transformation_, slice_width_, slice_height_, spacing_);

    //*******Kalman Filter*******//
    pkf_.SetParammeters(1.0/5.0, 0.05, 0.05);
	pkf_.InitializeKalmanFilter();
    Eigen::VectorXd position_euler_angle(6);
    position_euler_angle << dynamic_config_.tx, dynamic_config_.ty, dynamic_config_.tz, dynamic_config_.rx, dynamic_config_.ry, dynamic_config_.rz;
	pkf_.SetInitialXhatEluerAngle(position_euler_angle);
    // cout << "transformation_->GetMatrix(): " << endl << transformation_->GetMatrix() << endl;
    // cout << "pkf_.GetEstimateFromEluerAngle(): " << endl << pkf_.GetEstimateFromEluerAngle().matrix() << endl;
    // cout << "pkf_.NextStatePrediction( 0.2 ): " << endl << pkf_.Position_Eluer_Angle_To_Affine3d(pkf_.NextStatePrediction( 0.2 )).matrix() << endl;
    
    itk::Vector<double, 3> r; 
    nh_.getParam("/svr/target_tx", t_[0]); nh_.getParam("/svr/target_ty", t_[1]); nh_.getParam("/svr/target_tz", t_[2]);
    nh_.getParam("/svr/target_rx", r[0]); nh_.getParam("/svr/target_ry", r[1]); nh_.getParam("/svr/target_rz", r[2]);
    transformation_->SetTranslation(t_);
    transformation_->SetRotation(r[0], r[1], r[2]);
    taget_slice_ = ExtractSliceFromVolume(volume_, transformation_, slice_width_, slice_height_, spacing_);
    SaveImage(taget_slice_, "taget_slice.png");
    target_slice_pose_ = pkf_.Position_Eluer_Angle_To_Affine3d( optimization_.ITKTransformToEigen(transformation_) );
    cout << "target_slice_pose_: " << endl << target_slice_pose_.matrix() << endl;
}

void ThyroidBiopsy::RobotInitialization()
{
    custom_ros_lib_->SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");
    custom_ros_lib_->JointPositionControl(initial_joint_angle_, 3.0);
    custom_ros_lib_->SwitchController("pos_based_pos_traj_controller", "joint_group_vel_controller");
    force_controller_->UpdateZeroWrench();
    force_controller_->Approach(-0.1, 0.005);
    ros::Duration(1.0).sleep();
}

void ThyroidBiopsy::MainLoop()
{
    RobotInitialization();
    RegistrationInitialization();
    auto timer = CustomTimer();
    auto timer2 = CustomTimer();

    while(ros::ok())
    {
        ROSImageToITK(itk_us_image_, us_image_);

        if(dynamic_config_.simulation || dynamic_config_.reset)
        {
            t_[0] = dynamic_config_.tx; t_[1] = dynamic_config_.ty; t_[2] = dynamic_config_.tz;
            transformation_->SetTranslation(t_);
            transformation_->SetRotation(dynamic_config_.rx, dynamic_config_.ry, dynamic_config_.rz);

            if(dynamic_config_.simulation) // use rqt to control the left image
                itk_us_image_ = ExtractSliceFromVolume(volume_, transformation_, slice_width_, slice_height_, spacing_);
            if(dynamic_config_.reset) // use rqt to control the right image
                transformation_x0_ =  optimization_.ITKTransformToEigen(transformation_);
        }

        timer.tic();
        Eigen::VectorXd x0 = transformation_x0_;
        auto success = optimization_.Optimize(itk_us_image_, x0);
        if (success) {
            probe_itk_us_image_ = CopyImage(itk_us_image_);
            transformation_x0_ = x0;
            // cout << "transformation_x0_:" << endl << transformation_x0_ << endl;
            is_registration_successful_ = true;
            registration_time_stamp_ = ros::Time::now();
        } else {
            std::cout << "Failed." << std::endl;
        }
        elasped_time_ = timer.toc(false);
        pkf_.UpdateStateEulerAngle(transformation_x0_);
    }

    cv_thread_.join();
}

void ThyroidBiopsy::ControlLoop()
{
    ros::Rate rate(200);
    while(ros::ok() && is_registration_successful_ == false) {rate.sleep();};
    std::cout << "Start control loop." << std::endl;
    while(ros::ok())
    {
        auto time_interval = ros::Time::now() - registration_time_stamp_;
        auto current_slice_pose = pkf_.Position_Eluer_Angle_To_Affine3d(pkf_.NextStatePrediction( time_interval.toSec() ));
        auto probe_to_target = current_slice_pose.inverse() * target_slice_pose_;
		auto velocity_base_visual_servo_ = visual_servo_->PBVS1(MmToM(probe_to_target), Eigen::Affine3d::Identity());
		auto velocity_base_frame_ft_ = force_controller_->ForceVelocityController(force_controller_->get_base_2_end_effector(), expected_wrench_);
        // auto jonit_velocity = visual_servo_->ToJointSpaceVelocity(velocity_base_frame_ft_);
        auto jonit_velocity = visual_servo_->ToJointSpaceVelocity(velocity_base_visual_servo_ + velocity_base_frame_ft_);
        
        custom_ros_lib_->WrenchRvizDisplay(velocity_base_visual_servo_, "base_link", 10);
        custom_ros_lib_->BroadcastTransform("probe", "target_slice", MmToM(probe_to_target));

        custom_ros_lib_->PublishJointVelocity(jonit_velocity);
        rate.sleep();
        if(end_control_) break;
    }

    force_controller_->SetZeroVelocity();
    custom_ros_lib_->SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");
    custom_ros_lib_->JointPositionControl(initial_joint_angle_, 3.0);
    ros::shutdown();

    control_thread_.join();
}
void ThyroidBiopsy::DisplayComparisonLoop()
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        if(is_registration_successful_)
        {
            auto solution_slice = ExtractSliceFromVolume(volume_, optimization_.EigenToITKTransform(transformation_x0_), slice_width_, slice_height_, spacing_);
            // cout << "SSD: " << SSD3(probe_itk_us_image_, solution_slice) << endl;
            RegistrationVisualization(ITKImageToROS(probe_itk_us_image_), ITKImageToROS(solution_slice), SSD3(probe_itk_us_image_, solution_slice));
            auto taget_slice = cv_bridge::toCvCopy(ITKImageToROS(taget_slice_), sensor_msgs::image_encodings::TYPE_8UC1);
            cv::Mat resized_up;
            cv::resize(taget_slice->image, resized_up, cv::Size(slice_width_*volume_resol_reduction_, slice_height_*volume_resol_reduction_), cv::INTER_LINEAR);
            cv::imshow("Target Slice", resized_up);
            char key = cv::waitKey(1);
            is_registration_successful_ = false;
            // SaveImage(solution_slice, "solution_slice.png");
            if(key == 'q')
            {
                end_control_ = true;
            }
        }
        rate.sleep();
    }
}

void ThyroidBiopsy::ROSImageToITK(VolumeType::Pointer& itk_img, sensor_msgs::Image ros_image)
{
    using IteratorType = itk::ImageRegionIterator< VolumeType >;
    IteratorType iterator( itk_img, itk_img->GetRequestedRegion() );
    
    int itk_img_size = iterator.GetRegion().GetSize()[0] * iterator.GetRegion().GetSize()[1] * iterator.GetRegion().GetSize()[2];
    if(itk_img_size != ros_image.data.size())
    {
        cout << "ROSImageToITK(): size does not match!" << endl;
        cout << "itk_img_size:" << itk_img_size << "  ros_image.data.size():" << ros_image.data.size() << endl;
        exit(0);
    }
    
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

    cv::Mat comparison;
    CombineImages(comparison, current_cv_ptr->image, result_cv_ptr->image);
    cv::Mat resized_up;
    cv::resize(comparison, resized_up, cv::Size(slice_width_*volume_resol_reduction_*2, slice_height_*volume_resol_reduction_), cv::INTER_LINEAR);

    string str1 = "SSD: ";
    char ch[256];
    sprintf(ch, "%lf", ssd);
    string str2 = ch;
    string str = str1 + str2;
    cv::putText(resized_up,
            str,
            cv::Point(resized_up.cols / 2, resized_up.rows / 1.05), //top-left position
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            CV_RGB(255, 255, 255), //font color
            0.2);
    str1 = "Frequence: ";
    sprintf(ch, "%lf", 1.0/elasped_time_);
    str2 = ch;
    str = str1 + str2;
    cv::putText(resized_up,
            str,
            cv::Point(resized_up.cols / 2, resized_up.rows / 1.1), //top-left position
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            CV_RGB(255, 255, 255), //font color
            0.2);
    cv::putText(resized_up,
            "real-time probe image",
            cv::Point(resized_up.cols / 20, resized_up.rows / 20), //top-left position
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            CV_RGB(255, 255, 255), //font color
            0.2);
    cv::putText(resized_up,
            "slice from model",
            cv::Point(resized_up.cols / 2 + resized_up.cols / 20, resized_up.rows / 20), //top-left position
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            CV_RGB(255, 255, 255), //font color
            0.2);
    cv::imshow("Slice-to-Volume Registration", resized_up);
}

void ThyroidBiopsy::SaveImage(VolumeType::Pointer image, string image_name)
{
    using WriterType = itk::ImageFileWriter<VolumeType>;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(image_name);
    writer->SetInput(image);
    writer->Update();
}

void ThyroidBiopsy::DynamicReconfigCallback(ultrasound_robot::svrConfig &config, uint32_t level)
{
    dynamic_config_ = config;
}

Eigen::Affine3d ThyroidBiopsy::MmToM(Eigen::Affine3d tf)
{
    tf.translation() = tf.translation() * 0.001;
    return tf;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "thyroid_biopsy");
    ros::NodeHandle nh;
    
    pt::ptree root;
	pt::read_json("src/ultrasound_robot/config/force_controller.json", root);
    
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    shared_variable_ptr->config_tree = root;
    
    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();

    StatesHub states_hub(shared_variable_ptr);
    sleep(1);

    ThyroidBiopsy thyroid_biopsy(shared_variable_ptr, root);
    thyroid_biopsy.MainLoop();

    return 0;

}