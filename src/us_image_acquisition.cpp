#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "std_msgs/Float64MultiArray.h"
#include "CustomRosLib.h"

class UltrasoundCalibration
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Subscriber pointclound_sub_;
        ros::Publisher joint_velocity_command_pub_;
        ros::Publisher py_sample_command_pub_;
        ros::Publisher py_ee_link_tf_pub_;
        ros::Publisher vel_pub_;
        ros::ServiceServer wipe_blackboard_service_;
        ForceTorqueController* force_controller_;
        CustomRosLib custom_ros_lib_;

    public:
        
        UltrasoundCalibration(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~UltrasoundCalibration();
        void MainLoop();
        void SendTfAndSampleCommand();
        void SendEndCommand();
};

UltrasoundCalibration::UltrasoundCalibration(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
custom_ros_lib_(this->nh_)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    // pointclound_sub_ = nh_.subscribe< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth_registered/points", 1, &UltrasoundCalibration::PointcloudCallback, this);
    py_sample_command_pub_ = nh_.advertise<std_msgs::String>("/sample_command", 1);
    py_ee_link_tf_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ee_link_tf", 1);
    // vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    force_controller_ = new ForceTorqueController(shared_variable_ptr_, config_tree_);
}

UltrasoundCalibration::~UltrasoundCalibration()
{
    std::cout << "~UltrasoundCalibration()" << std::endl;
}

void UltrasoundCalibration::SendTfAndSampleCommand()
{
    auto ee_link = custom_ros_lib_.ListenToTransform_TF("base_link", "ee_link");
    std_msgs::Float64MultiArray py_ee_link_tf;
    {
        py_ee_link_tf.data.push_back(ee_link.getOrigin().getX());
        py_ee_link_tf.data.push_back(ee_link.getOrigin().getY());
        py_ee_link_tf.data.push_back(ee_link.getOrigin().getZ());
        py_ee_link_tf.data.push_back(ee_link.getRotation().getX());
        py_ee_link_tf.data.push_back(ee_link.getRotation().getY());
        py_ee_link_tf.data.push_back(ee_link.getRotation().getZ());
        py_ee_link_tf.data.push_back(ee_link.getRotation().getW());
    }
    // std::cout << py_ee_link_tf << std::endl;
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    py_ee_link_tf_pub_.publish(py_ee_link_tf);
    std_msgs::String command;
    command.data = "1";
    py_sample_command_pub_.publish(command);

}

void UltrasoundCalibration::SendEndCommand()
{
    std_msgs::String command;
    command.data = "2";
    py_sample_command_pub_.publish(command);

}

void UltrasoundCalibration::MainLoop()
{
    custom_ros_lib_.DeactivateController("pos_based_pos_traj_controller");
    custom_ros_lib_.DeactivateController("joint_group_vel_controller");

    ros::Rate loop_rate(1);
    geometry_msgs::Pose target_position;
    geometry_msgs::PoseArray target_pose_array;
    
    for(int i=0;i<6;i++)
    {
        char command;
        std::cout << "Press e to record waypoint, s to start" << i << std::endl;
        std::cin >> command;
        if(command == 's') {std::cout << "Start sampling ..." << std::endl; break;}
        if(command != 'e') {std::cout << "Kill this program" << std::endl; exit(0);}
        geometry_msgs::Pose ee_link = custom_ros_lib_.ListenToTransform("base_link", "ee_link");
        target_pose_array.poses.push_back(ee_link);
        cout << ee_link << endl;
    }
    cout << "num of points: " << target_pose_array.poses.size() << endl;
    
    custom_ros_lib_.SwitchController("joint_group_vel_controller", "pos_based_pos_traj_controller");

    while (ros::ok())
    {
        int interval = 100;
        custom_ros_lib_.UpdateJointSeed(shared_variable_ptr_->joint_states);
        for(int cnt=0;cnt<target_pose_array.poses.size()/2;cnt++)
        {
            cout << "cnt: " << cnt << endl;
            cout << "approaching... " << cnt << endl;
            target_position = target_pose_array.poses[cnt*2];
            target_position.position.z += 0.05;
            custom_ros_lib_.CartesianPositionControl(target_position, 5, 0.0);

            target_position = target_pose_array.poses[cnt*2];
            custom_ros_lib_.CartesianPositionControl(target_position, 5, 0.0);
            for(int i = 0; i<interval; i++)
            {
                cout << i << endl;
                target_position.position.x = target_pose_array.poses[cnt*2].position.x + (i/double(interval))*(target_pose_array.poses[cnt*2+1].position.x-target_pose_array.poses[cnt*2].position.x);
                target_position.position.y = target_pose_array.poses[cnt*2].position.y + (i/double(interval))*(target_pose_array.poses[cnt*2+1].position.y-target_pose_array.poses[cnt*2].position.y);
                target_position.position.z = target_pose_array.poses[cnt*2].position.z + (i/double(interval))*(target_pose_array.poses[cnt*2+1].position.z-target_pose_array.poses[cnt*2].position.z);
                
                try
                {
                    custom_ros_lib_.CartesianPositionControl(target_position, 0.4, 0.0);
                }
                catch(const char* msg)
                {
                    continue;
                }
                ros::Duration(0.1).sleep();
                SendTfAndSampleCommand();
                ros::Duration(0.2).sleep();
            }

            target_position.position.z += 0.05;
            cout << "lifting... " << cnt << endl;
            custom_ros_lib_.CartesianPositionControl(target_position, 2, 0.0);
        }
        SendEndCommand();
        break;
    }
    
    delete(force_controller_);
    // ros::waitForShutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasound_calibration");
    ros::NodeHandle nh;
    
    pt::ptree root;
	pt::read_json("/home/sunlab/Desktop/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    shared_variable_ptr->config_tree = root;

    UltrasoundCalibration us_calib(shared_variable_ptr, root);
    
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    StatesHub states_hub(shared_variable_ptr);
    sleep(1);

    us_calib.MainLoop();

    return 0;
}