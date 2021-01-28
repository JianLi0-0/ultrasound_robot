#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <ultrasound_robot/wipe_bb.h>
#include "PointCloudCustomLib.h"

void start_states_hub_thread(std::shared_ptr<SharedVariable> ptr)
{
	std::cout << "states_hub_thread" << std::endl;
    StatesHub states_hub(ptr);
    ros::waitForShutdown();
};

class WipeBlackboard
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Subscriber pointclound_sub_;
        ros::Publisher joint_velocity_command_pub_;
        ros::Publisher pointcloud_pub_;
        ros::Publisher pose_array_pub_;
        ros::Publisher vel_pub_;
        ros::ServiceServer wipe_blackboard_service_;
        PointCloudCustomLib pccl_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        ForceTorqueController* force_controller_;


    public:
        
        WipeBlackboard(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~WipeBlackboard();
        void PointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
        void PublishPoseArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
        bool WipeBlackboardServiceCallback(ultrasound_robot::wipe_bb::Request  &req, ultrasound_robot::wipe_bb::Response &res);
        void MainLoop();
};

WipeBlackboard::WipeBlackboard(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
    cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    pointclound_sub_ = nh_.subscribe< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth_registered/points", 1, &WipeBlackboard::PointcloudCallback, this);
    pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/pointcloud", 1);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/normal_vectors", 1);
    wipe_blackboard_service_ = nh_.advertiseService("wipe_blackboard_service", &WipeBlackboard::WipeBlackboardServiceCallback, this);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    force_controller_ = new ForceTorqueController(shared_variable_ptr_, config_tree_);
}

WipeBlackboard::~WipeBlackboard()
{
    std::cout << "~WipeBlackboard()" << std::endl;
}

void WipeBlackboard::PointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    cloud_ = msg->makeShared();
}

void WipeBlackboard::PublishPoseArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    pointcloud_pub_.publish(*pointcloud);
}

bool WipeBlackboard::WipeBlackboardServiceCallback(ultrasound_robot::wipe_bb::Request  &req, ultrasound_robot::wipe_bb::Response &res)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    auto pointcloud = cloud_;
    pccl_.EstimatePointcloudNormals(pointcloud, normals);
    auto pose_array = pccl_.FromNormalsToPoseArray(req.waypoints, pointcloud, normals);
    pose_array_pub_.publish(pose_array);
    return true;
}

void WipeBlackboard::MainLoop()
{
    ros::Rate loop_rate( int(1.0/config_tree_.get<double>("delta_t", 0.005)) );
    Eigen::Affine3d base_2_task_frame;
    base_2_task_frame.setIdentity();
    auto translation = AsVector<double>(config_tree_, "admittance_params.static_target");
    base_2_task_frame.translate( Eigen::Vector3d(translation.data()) );
    // base_2_task_frame.rotate(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY()));
    auto expected_wrench_data = AsVector<double>(config_tree_, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);
    while (ros::ok())
    {
        // auto velocity = force_controller_ ->AdaptiveForceVelocityController(base_2_task_frame, expected_wrench);
        auto velocity = force_controller_ ->ZeroMomentVelocityController();

        std_msgs::Float64MultiArray velocity_msg;
        for(int i=0;i<velocity.size();i++)
            velocity_msg.data.push_back(velocity(i));

        vel_pub_.publish(velocity_msg);
        loop_rate.sleep();
    }
    
    delete(force_controller_);
    // ros::waitForShutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wipe_blackboard");
    ros::NodeHandle nh;
    
    pt::ptree root;
	pt::read_json("/home/qwe/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    shared_variable_ptr->config_tree = root;

    WipeBlackboard wipe_blackboard(shared_variable_ptr, root);
    
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    sleep(1);
    std::thread states_hub_thread(start_states_hub_thread, shared_variable_ptr);
    sleep(1);

    wipe_blackboard.MainLoop();

    states_hub_thread.join();

    return 0;
}