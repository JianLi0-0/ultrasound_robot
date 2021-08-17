#include "VisualServo.h"
#include "StatesHub.h"
#include "CustomRosLib.h"
#include "PoseKalmanFilter.h"

class VisualServoTest
{
    private:
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Subscriber aruco_marker_sub_;
		geometry_msgs::PoseStamped marker_pose_;
		VisualServo* visual_servo_;
		CustomRosLib* custom_ros_lib_;
		PoseKalmanFilter pkf_;

    public:
        
        VisualServoTest(std::shared_ptr<SharedVariable> ptr);
        ~VisualServoTest();
        void MainLoop();
        void ArucoMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
};

VisualServoTest::VisualServoTest(std::shared_ptr<SharedVariable> ptr)
{
    shared_variable_ptr_ = ptr;
	visual_servo_ = new VisualServo(shared_variable_ptr_);
	custom_ros_lib_ = new CustomRosLib(shared_variable_ptr_);

    aruco_marker_sub_ = nh_.subscribe("/aruco_single/pose", 1, &VisualServoTest::ArucoMarkerCallback, this);

}

VisualServoTest::~VisualServoTest()
{
    std::cout << "~VisualServoTest()" << std::endl;
}

void VisualServoTest::MainLoop()
{
	custom_ros_lib_->SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");
	std::cout << "Take your time to adjust the robot. Then press 'q' to quit. Otherwise proceed." << std::endl;
	char a;
	cin >> a;
	if(a == 'q') exit(0);

	custom_ros_lib_->SwitchController("pos_based_pos_traj_controller", "joint_group_vel_controller");

	Eigen::Affine3d camera_to_object;
	camera_to_object.setIdentity();

	Eigen::Affine3d desired_camera_to_object;
	desired_camera_to_object.setIdentity();
	desired_camera_to_object.translate( Eigen::Vector3d(0,0,0.3) );
	desired_camera_to_object.rotate( Eigen::Quaterniond( 0.7, -0.7, 0.0, 0.0 ) ); //(w, x, y, z)

	// visual_servo_->set_lambda(0.5);
	visual_servo_->set_lambda(0.4);

	pkf_.SetParammeters(1.0/30.0, 0.03, 2.0);
	pkf_.InitializeKalmanFilter();
	Eigen::Affine3d X;
	auto orient = marker_pose_.pose.orientation;
	auto position = marker_pose_.pose.position;
	X.setIdentity();
	X.translate( Eigen::Vector3d( position.x, position.y, position.z ));
	X.rotate( Eigen::Quaterniond( orient.w, orient.x, orient.y, orient.z ) );
	pkf_.SetInitialXhat(X);

	ros::Rate rate(30);
	while ( ros::ok() )
	{
		auto orient = marker_pose_.pose.orientation;
		auto position = marker_pose_.pose.position;
		camera_to_object.setIdentity();
		camera_to_object.translate( Eigen::Vector3d( position.x, position.y, position.z ));
		camera_to_object.rotate( Eigen::Quaterniond( orient.w, orient.x, orient.y, orient.z ) );

		pkf_.UpdateState(camera_to_object);

		auto joint_velocity = visual_servo_->PBVS1(pkf_.GetEstimate(), desired_camera_to_object);
		// auto joint_velocity = visual_servo_->PBVS1(camera_to_object, desired_camera_to_object);
		// auto joint_velocity1 = visual_servo_->PBVS2(camera_to_object, desired_camera_to_object);

		// exit(0);
		custom_ros_lib_->BroadcastTransform("camera_color_optical_frame", "kf_marker", pkf_.GetEstimate());
		custom_ros_lib_->PublishJointVelocity(joint_velocity);
		custom_ros_lib_->CheckBox(Eigen::Vector3d(0.5, -0.25, 0.24), Eigen::Vector3d(0.20, 0.15, 0.1));
		rate.sleep();
	}
}

void VisualServoTest::ArucoMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	marker_pose_.header = msg->header;
	marker_pose_.pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VisualServoTest");
    ros::NodeHandle nh;

	std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    StatesHub states_hub(shared_variable_ptr);
    sleep(1);

	ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

	auto visual_servo_test = new VisualServoTest(shared_variable_ptr);
	sleep(2);
	visual_servo_test->MainLoop();

    return 0;
}