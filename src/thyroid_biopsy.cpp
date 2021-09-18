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
#include "ultrasound_robot/bayesian_svr.h"

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
        ros::ServiceClient bayesian_svr_client_;
        ros::Publisher frequecy_pub_;

        TransformType::Pointer transformation_;
        TransformType::Pointer target_transformation_;
        Optimization optimization_;
        VolumeType::Pointer volume_;
        VolumeType::SpacingType spacing_;
        float slice_width_;
        float slice_height_;
        Eigen::Affine3d target_world_;
        Eigen::Affine3d target_slice_pose_;
        Eigen::Affine3d current_slice_pose_;
        Eigen::Affine3d probe_to_target_;
        Eigen::VectorXd expected_wrench_;
        std::thread control_thread_;
        ros::Time registration_time_stamp_;
        std::vector<double> initial_joint_angle_;
        bool end_control_ = false;
        bool start_navigation_ = false;
        bool kf_regist_pred_ = false;
        double kf_pred_ = 0.5;
        bool record_tracking_error_ = false;
        bool save_the_world = true;
        bool reset_starting_point = false;

        sensor_msgs::Image us_image_;
        VolumeType::Pointer probe_itk_us_image_, itk_us_image_,taget_slice_;
        bool is_registration_successful_ = false;
        bool is_robot_initializated_ = false;
        std::thread cv_thread_;
        Eigen::VectorXd transformation_x0_;
        itk::Vector<double, 3> t_, r_; 
        double volume_resol_reduction_ = 1;
        double elasped_time_ = 1;
        double nm_rel_x_change_tol_ = 0;
        double nm_rel_obj_change_tol_ = 0;

        dynamic_reconfigure::Server<ultrasound_robot::svrConfig> dynamic_reconfig_server_;
        dynamic_reconfigure::Server<ultrasound_robot::svrConfig>::CallbackType dynamic_reconfig_f_;
        ultrasound_robot::svrConfig dynamic_config_;

        ros::Subscriber aruco_marker_sub_;
		geometry_msgs::PoseStamped marker_pose_;

        std::thread recording_thread_;


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
        void BayesianSVR();
        void ArucoMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
        Eigen::Affine3d StoreTargetWorld();
        void Recording();
};

ThyroidBiopsy::ThyroidBiopsy(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
control_thread_(&ThyroidBiopsy::ControlLoop, this),
cv_thread_(&ThyroidBiopsy::DisplayComparisonLoop, this),
recording_thread_(&ThyroidBiopsy::Recording, this)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    us_img_sub_ = nh_.subscribe("/us_image", 1, &ThyroidBiopsy::UsImageCallback, this);
    force_controller_ = new ForceTorqueController(shared_variable_ptr_, config_tree_);
    custom_ros_lib_ = new CustomRosLib(shared_variable_ptr_);
	visual_servo_ = new VisualServo(shared_variable_ptr_);
    double lamda; nh_.getParam("/svr/visual_servo_lamda", lamda);
    double rotation_lamda; nh_.getParam("/svr/visual_servo_rotation_lamda", rotation_lamda);
	visual_servo_->set_lambda(lamda);
	visual_servo_->set_rotation_lambda(rotation_lamda);
    visual_servo_->set_link_name_("ee_link", "probe");

    nh_.getParam("/svr/servo_w", slice_width_);
    nh_.getParam("/svr/servo_h", slice_height_);
    nh_.getParam("/svr/volume_resol_reduction", volume_resol_reduction_);
    nh_.getParam("/svr/initial_pose", initial_joint_angle_);
    auto temp = initial_joint_angle_[0];initial_joint_angle_[0]=initial_joint_angle_[2];initial_joint_angle_[2]=temp;
    nh_.getParam("/svr/nm_rel_x_change_tol", nm_rel_x_change_tol_);
    nh_.getParam("/svr/nm_rel_obj_change_tol", nm_rel_obj_change_tol_);
    cout << "nm_rel_x_change_tol_: " << nm_rel_x_change_tol_ << " nm_rel_obj_change_tol: " << nm_rel_obj_change_tol_ << endl;
    // slice_width_ = slice_width_ / volume_resol_reduction_;
    // slice_height_ = slice_height_ / volume_resol_reduction_;
    // cout << "slice_width: " << slice_width_ << " slice_height: " << slice_height_ << endl;
    dynamic_reconfig_f_ = boost::bind(&ThyroidBiopsy::DynamicReconfigCallback, this, _1, _2);
    dynamic_reconfig_server_.setCallback(dynamic_reconfig_f_);

    auto expected_wrench_data = AsVector<double>(config_tree_, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);
    expected_wrench_ = expected_wrench;

    bayesian_svr_client_ = nh_.serviceClient<ultrasound_robot::bayesian_svr>("bayesian_svr");
    frequecy_pub_ = nh_.advertise<std_msgs::Float32>("/frequency", 1);
    nh_.getParam("/svr/kf_pred", kf_pred_);

    aruco_marker_sub_ = nh_.subscribe("/aruco_single/pose", 1, &ThyroidBiopsy::ArucoMarkerCallback, this);
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
    reader->SetFileName("/home/kuka/lee_ws/src/ultrasound_robot/src/python/new_biopsy_single.mha");
    reader->Update();
    volume_ = reader->GetOutput();
    volume_ = ScaleVolume(volume_, transformation_, volume_resol_reduction_);
    SaveImage(volume_, "/home/kuka/lee_ws/src/ultrasound_robot/src/python/new_biopsy_single_scaled.mha");
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
    target_transformation_ = transformation_;
    taget_slice_ = ExtractSliceFromVolume(volume_, transformation_, slice_width_, slice_height_, spacing_);
    SaveImage(taget_slice_, "target_slice.png");
    target_slice_pose_ = pkf_.Position_Eluer_Angle_To_Affine3d( optimization_.ITKTransformToEigen(transformation_) );
    cout << "target_slice_pose_: " << endl << target_slice_pose_.matrix() << endl;
}

void ThyroidBiopsy::RobotInitialization()
{
    custom_ros_lib_->SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");
    custom_ros_lib_->JointPositionControl(initial_joint_angle_, 3.0);
    custom_ros_lib_->SwitchController("pos_based_pos_traj_controller", "joint_group_vel_controller");
    force_controller_->UpdateZeroWrench();
    force_controller_->Approach(-1, 0.005);
    // ros::Duration(1.0).sleep();
}

void ThyroidBiopsy::BayesianSVR()
{
    bool bayesian = false;
    nh_.getParam("/svr/bayesian", bayesian);
    if(!bayesian) return ;

    ultrasound_robot::bayesian_svr srv;
    srv.request.command = false;
    cout << "Calling service BayesianSVR ..." << endl;
    if (bayesian_svr_client_.call(srv))
    {
        auto angle_position = srv.response.angle_position.data;
        assert(angle_position.size() == 6);
        cout << "srv.response.angle_position:" << srv.response.angle_position << endl;
        transformation_x0_ << angle_position[3], angle_position[4], angle_position[5], angle_position[0], angle_position[1], angle_position[2];
    }
    else
    {
        ROS_ERROR("Failed to call service BayesianSVR");
        end_control_ = true;
    }
}

void ThyroidBiopsy::MainLoop()
{
    RobotInitialization();
    RegistrationInitialization();
    is_robot_initializated_ = true;
    BayesianSVR();
    // ofstream OutFile("/home/kuka/lee_ws/src/ultrasound_robot/data/pose.txt");
    // OutFile << "p_x " << "p_y " << "p_z " << "p_a " << "p_b " << "p_c " << "r_x " << "r_y " << "r_z " << "r_a " << "r_b " << "r_c" << endl;
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
        // if(reset_starting_point) {x0 =  optimization_.ITKTransformToEigen(target_transformation_); reset_starting_point = false;}
        // if(kf_regist_pred_) x0 = pkf_.NextStatePredictionFromObservation( (ros::Time::now() - registration_time_stamp_).toSec() * kf_pred_ );
        registration_time_stamp_ = ros::Time::now();

        auto success = optimization_.Optimize(itk_us_image_, x0, nm_rel_x_change_tol_, nm_rel_obj_change_tol_);
        success = true; // large tol would lead to failure, we set 'success' to true manually
        if (success) {
            probe_itk_us_image_ = CopyImage(itk_us_image_);
            transformation_x0_ = x0;
            // cout << "transformation_x0_:" << endl << transformation_x0_ << endl;
            is_registration_successful_ = true;
            
        } else {
            std::cout << "Failed." << std::endl;
        }
        elasped_time_ = timer.toc(false);
        pkf_.UpdateStateEulerAngle(transformation_x0_);
    }

    cv_thread_.join();
}

#include <dirent.h>
void getFiles(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);
    }
    }
    closedir(pDir);
}

void ThyroidBiopsy::Recording()
{
    ros::Rate rate1(200);
    while(ros::ok() && is_registration_successful_ == false) {rate1.sleep();};
    ros::Rate rate(0.3);
    bool recording = false;
    string folder;
    nh_.getParam("/svr/recording", recording);
    nh_.getParam("/svr/recording_folder", folder);
    while(ros::ok())
    {
        if(!recording) break;
        vector<string> files;
        
        getFiles(folder, files);
        string num = std::to_string(files.size()+1);
        string file_name = folder + "/" + num + ".png";
        SaveImage(probe_itk_us_image_, file_name);
        rate.sleep();
        if(end_control_) break;
    }

}

void ThyroidBiopsy::ArucoMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    static ofstream OutFile_v("/home/kuka/lee_ws/src/ultrasound_robot/data/tracking_error_virtual.txt");
    static ofstream OutFile_r("/home/kuka/lee_ws/src/ultrasound_robot/data/tracking_error_real.txt");
    static Eigen::Affine3d probe_to_marker;
    if(record_tracking_error_)
    {
        if(end_control_) {record_tracking_error_=false;}
        auto actual_marker = custom_ros_lib_->ListenToTransform("base_link", "aruco_marker_frame");
        geometry_msgs::Point p = actual_marker.position;
        geometry_msgs::Quaternion r = actual_marker.orientation;
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(r.x, r.y, r.z, r.w)).getRPY(roll, pitch, yaw);
        OutFile_r << p.x << " " << p.y << " " << p.z << " " << roll << " " << pitch << " " << yaw << endl;

        Eigen::Affine3d probe_world = custom_ros_lib_->ListenToTransform_Eigen("base_link", "probe");
        Eigen::Affine3d virtual_marker = probe_world * probe_to_marker;
        auto pp = virtual_marker.translation();
        Eigen::Quaterniond q(virtual_marker.rotation());
        tf::Matrix3x3(tf::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(roll, pitch, yaw);
        OutFile_v << pp(0) << " " << pp(1) << " " << pp(2) << " " << roll << " " << pitch << " " << yaw << endl;
    }
    else{
        marker_pose_.header = msg->header;
        marker_pose_.pose = msg->pose;
        probe_to_marker = custom_ros_lib_->ListenToTransform_Eigen("probe", "aruco_marker_frame");
    }
}

Eigen::Affine3d ThyroidBiopsy::StoreTargetWorld()
{
        // Eigen::Affine3d current_slice_pose = pkf_.Position_Eluer_Angle_To_Affine3d(pkf_.NextStatePrediction(0));
        // Eigen::Affine3d probe_to_target = current_slice_pose.inverse() * target_slice_pose_;
        // Eigen::Affine3d probe_world = custom_ros_lib_->ListenToTransform_Eigen("base_link", "probe");
        // Eigen::Affine3d output = probe_world * MmToM(probe_to_target);
        Eigen::Affine3d output = custom_ros_lib_->ListenToTransform_Eigen("base_link", "target_world");
        cout << "StoreTargetWorld(): " << endl << output.matrix() << endl;
        return output;
}

void ThyroidBiopsy::ControlLoop()
{
    WrenchRvizDisplay rivz_visual_sevo("/vel_visual_servo");
    WrenchRvizDisplay rivz_ft("/vel_ft");

    ros::Rate rate(200);
    while(ros::ok() && is_robot_initializated_ == false) {rate.sleep();};
    std::cout << "Start control loop." << std::endl;

    nh_.getParam("/svr/save_the_world", save_the_world);
    if(save_the_world)
    {
        ros::Duration(1.0).sleep();
        target_world_ =  StoreTargetWorld();
    }
    while(ros::ok())
    {
        auto time_interval = ros::Time::now() - registration_time_stamp_;
        auto current_slice_pose = pkf_.Position_Eluer_Angle_To_Affine3d(pkf_.NextStatePrediction( time_interval.toSec() ));
        // auto current_slice_pose = pkf_.GetEstimateFromEluerAngle();
        auto probe_to_target = MmToM(current_slice_pose.inverse() * target_slice_pose_);
        if(save_the_world)
        {
            double translation_threshold;
            nh_.getParam("/svr/translation_threshold", translation_threshold);
            // cout << "probe_to_target.translation().norm()" << probe_to_target.translation().norm() << endl;
            auto probe_world = custom_ros_lib_->ListenToTransform_Eigen("base_link", "probe");
            probe_to_target = probe_world.inverse() * target_world_;
            if (probe_to_target.translation().norm() < translation_threshold) 
            {
                reset_starting_point = true;
                save_the_world=false;
                std::cout << "start tracking." << std::endl;
            }
        }
		auto velocity_base_visual_servo_ = visual_servo_->PBVS_TR(probe_to_target, Eigen::Affine3d::Identity());
		auto velocity_base_frame_ft_ = force_controller_->ForceVelocityController(force_controller_->get_base_2_end_effector(), expected_wrench_);
        
        custom_ros_lib_->BroadcastTransform("probe", "target_slice", probe_to_target);
        rivz_visual_sevo.Display(velocity_base_visual_servo_, "base_link", 10);
        rivz_ft.Display(velocity_base_frame_ft_, "base_link", 10);

        if(!start_navigation_) 
            velocity_base_visual_servo_ = Eigen::VectorXd::Zero(6);
        
        auto jonit_velocity = visual_servo_->ToJointSpaceVelocity(velocity_base_visual_servo_ + velocity_base_frame_ft_);
        
        custom_ros_lib_->PublishJointVelocity(jonit_velocity);
        rate.sleep();
        if(end_control_) break;

        if(start_navigation_)
        {
            static ofstream OutFile("/home/kuka/lee_ws/src/ultrasound_robot/data/feature_errors.txt");
            double roll, pitch, yaw;
            auto error = pkf_.FromeMatrixToErrorAxisAngle(probe_to_target);
            OutFile << error(0) << " " << error(1) << " " << error(2) << " " << error(3) << " " << error(4) << " " << error(5) << " "  << endl;
        }
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
            if(key == 'q')
            {
                SaveImage(solution_slice, "final_slice.png");
                end_control_ = true;
            }
            else if(key == 'a') start_navigation_ = true;
            // else if(key == 't') visual_servo_->set_lambda(visual_servo_->get_lambda() + 0.05);
            // else if(key == 'r') visual_servo_->set_rotation_lambda(visual_servo_->get_rotation_lambda() + 0.01);
            // else if(key == 'k') {kf_regist_pred_ = !kf_regist_pred_;  cout << "kf_prediction:" << kf_regist_pred_ << endl;}
            else if(key == 't') 
            {
                record_tracking_error_ = true;
                target_slice_pose_ = pkf_.Position_Eluer_Angle_To_Affine3d( transformation_x0_ ); 
                cout << "record_tracking_error !" << endl;
                // visual_servo_->set_lambda(visual_servo_->get_lambda() + 0.2);
                // visual_servo_->set_rotation_lambda(visual_servo_->get_rotation_lambda() + 0.1);
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
    static double mean_elasped_time = 0.0; 
    mean_elasped_time = 0.1*elasped_time_ + 0.9*mean_elasped_time;
    std_msgs::Float32 f; f.data = 1.0/mean_elasped_time;
    frequecy_pub_.publish(f);
    sprintf(ch, "%lf", f.data);
    // sprintf(ch, "%lf", f.data);
    
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
	pt::read_json("/home/kuka/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    
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