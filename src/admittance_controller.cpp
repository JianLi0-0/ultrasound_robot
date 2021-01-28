#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "std_msgs/Float64MultiArray.h"

void start_states_hub_thread(std::shared_ptr<SharedVariable> ptr)
{
	std::cout << "states_hub_thread" << std::endl;
    StatesHub states_hub(ptr);
    ros::waitForShutdown();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_controller");
    ros::NodeHandle nh;
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    pt::ptree root;
	pt::read_json("/home/qwe/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    ForceTorqueController ft_controller(shared_variable_ptr, root);
    shared_variable_ptr->config_tree = root;
    ros::Rate loop_rate( int(1.0/root.get<double>("delta_t", 0.005)) );
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    sleep(1);
    std::thread states_hub_thread(start_states_hub_thread, shared_variable_ptr);
    sleep(1);

    Eigen::Affine3d task_frame_pose;
    task_frame_pose.setIdentity();
    auto translation = AsVector<double>(root, "admittance_params.static_target");
    task_frame_pose.translate( Eigen::Vector3d(translation.data()) );
    task_frame_pose.rotate(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY()));

    Eigen::Affine3d task_frame_pose2;
    task_frame_pose2.setIdentity();
    auto translation2 = AsVector<double>(root, "admittance_params.static_target2");
    task_frame_pose2.translate( Eigen::Vector3d(translation2.data()) );
    task_frame_pose2.rotate(Eigen::AngleAxisd(0.3*M_PI, Eigen::Vector3d::UnitY()));
    task_frame_pose2.rotate(Eigen::AngleAxisd(0.2*M_PI, Eigen::Vector3d::UnitZ()));

    auto expected_wrench_data = AsVector<double>(root, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);
    // std::cout << "expected_wrench: " << std::endl << expected_wrench << std::endl;
    while(ros::ok())
    {
        auto past = ros::Time::now();
        while((ros::Time::now()-past).toSec() < 5.0)
        {
            // task_frame_pose = ft_controller.get_base_2_end_effector(); // drag
            auto jonit_velocity = ft_controller.AdmittanceVelocityController(task_frame_pose, expected_wrench);
            // std::cout << "ros::Time::now()-past: " << ros::Time::now()-past << std::endl;
            std_msgs::Float64MultiArray vel;
            for(int i=0;i<jonit_velocity.size();i++)
            {
                vel.data.push_back(jonit_velocity(i));
            }
            vel_pub.publish(vel);
            loop_rate.sleep();
        }
        past = ros::Time::now();
        while((ros::Time::now()-past).toSec() < 5.0)
        {
            // task_frame_pose = ft_controller.get_base_2_end_effector(); // drag
            auto jonit_velocity = ft_controller.AdmittanceVelocityController(task_frame_pose2, expected_wrench);
            // std::cout << "ros::Time::now()-past: " << ros::Time::now()-past << std::endl;
            std_msgs::Float64MultiArray vel;
            for(int i=0;i<jonit_velocity.size();i++)
            {
                vel.data.push_back(jonit_velocity(i));
            }
            vel_pub.publish(vel);
            loop_rate.sleep();
        }
        

        // std::cout << "jonit_velocity: " << jonit_velocity << std::endl;
        // std::cout << "vel_pub: " << vel << std::endl;
        // std::cout << "shared_variable_ptr.use_count()" <<  shared_variable_ptr.use_count() << std::endl;
    }

    ros::waitForShutdown();
    states_hub_thread.join();

    return 0;
}