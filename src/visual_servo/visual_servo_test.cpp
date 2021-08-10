#include "VisualServo.h"
#include "StatesHub.h"
#include "CustomRosLib.h"

class VisualServoTest
{
    private:
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Subscriber aruco_marker_sub_;
		geometry_msgs::PoseStamped marker_pose_;
		VisualServo* visual_servo;
		CustomRosLib* custom_ros_lib;

    public:
        
        VisualServoTest(std::shared_ptr<SharedVariable> ptr);
        ~VisualServoTest();
        void MainLoop();
        void ArucoMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
};

VisualServoTest::VisualServoTest(std::shared_ptr<SharedVariable> ptr)
{
    shared_variable_ptr_ = ptr;
	visual_servo = new VisualServo(shared_variable_ptr_);
	custom_ros_lib = new CustomRosLib(shared_variable_ptr_);

    aruco_marker_sub_ = nh_.subscribe("/aruco_single/pose", 1, &VisualServoTest::ArucoMarkerCallback, this);

}

VisualServoTest::~VisualServoTest()
{
    std::cout << "~VisualServoTest()" << std::endl;
}

void VisualServoTest::MainLoop()
{
	custom_ros_lib->SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");

	Eigen::Affine3d camera_to_object;
	camera_to_object.setIdentity();

	Eigen::Affine3d desired_camera_to_object;
	desired_camera_to_object.setIdentity();
	desired_camera_to_object.translate( Eigen::Vector3d(0,0,0.3) );

	ros::Rate rate(30);
	while ( ros::ok() )
	{
		auto orient = marker_pose_.pose.orientation;
		auto position = marker_pose_.pose.position;
		camera_to_object.setIdentity();
		camera_to_object.translate( Eigen::Vector3d( position.x, position.y, position.z ));
		camera_to_object.rotate( Eigen::Quaterniond( orient.w, orient.x, orient.y, orient.z ) );
		// cout << "camera_to_object" << endl << camera_to_object.matrix() << endl;
		
		auto joint_velocity = visual_servo->PBVS1(camera_to_object, desired_camera_to_object);
		custom_ros_lib->PublishJointVelocity(joint_velocity);
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