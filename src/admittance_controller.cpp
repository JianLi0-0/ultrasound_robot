#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "std_msgs/Float64MultiArray.h"
#include <control_msgs/JointJog.h>

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
	pt::read_json("/home/sunlab/Desktop/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    ForceTorqueController ft_controller(shared_variable_ptr, root);
    shared_variable_ptr->config_tree = root;
    ros::Rate loop_rate( int(1.0/root.get<double>("delta_t", 0.005)) );
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);
    ros::Publisher jog_vel_pub = nh.advertise<control_msgs::JointJog>("/jog_arm_server/joint_delta_jog_cmds", 1);

    ros::AsyncSpinner async_spinner(5);
    async_spinner.start();
    sleep(1);
    std::thread states_hub_thread(start_states_hub_thread, shared_variable_ptr);
    sleep(1);

    // Eigen::Affine3d task_frame_pose;
    // task_frame_pose.setIdentity();
    // auto translation = AsVector<double>(root, "admittance_params.static_target");
    // task_frame_pose.translate( Eigen::Vector3d(translation.data()) );
    // task_frame_pose.rotate(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY()));

    auto translation = AsVector<double>(root, "admittance_params.static_target.translation");
    auto quaternion = AsVector<double>(root, "admittance_params.static_target.quaternion");
    Eigen::Affine3d task_frame_pose;
    task_frame_pose.setIdentity();
    task_frame_pose.translate( Eigen::Vector3d(translation.data()) );
    task_frame_pose.rotate( Eigen::Quaterniond(quaternion.data()) );

    // Eigen::Affine3d task_frame_pose2;
    // task_frame_pose2.setIdentity();
    // auto translation2 = AsVector<double>(root, "admittance_params.static_target2");
    // task_frame_pose2.translate( Eigen::Vector3d(translation2.data()) );
    // task_frame_pose2.rotate(Eigen::AngleAxisd(0.3*M_PI, Eigen::Vector3d::UnitY()));
    // task_frame_pose2.rotate(Eigen::AngleAxisd(0.2*M_PI, Eigen::Vector3d::UnitZ()));

    translation = AsVector<double>(root, "admittance_params.static_target2.translation");
    quaternion = AsVector<double>(root, "admittance_params.static_target2.quaternion");
    Eigen::Affine3d task_frame_pose2;
    task_frame_pose2.setIdentity();
    task_frame_pose2.translate( Eigen::Vector3d(translation.data()) );
    task_frame_pose2.rotate( Eigen::Quaterniond(quaternion.data()) );

    auto expected_wrench_data = AsVector<double>(root, "admittance_params.expected_wrench");
    Eigen::Map<Eigen::VectorXd> expected_wrench(expected_wrench_data.data(), 6);

    // task_frame_pose = ft_controller.get_base_2_end_effector(); // drag
    // vector<string> str_name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    vector<string> str_name = shared_variable_ptr->joint_names;
    double limit = root.get<double>("joint_speed_limit", 0.1);
    while(ros::ok())
    {
        auto past = ros::Time::now();
        while((ros::Time::now()-past).toSec() < 2.0)
        {
            auto jonit_velocity = ft_controller.ForceVelocityController(task_frame_pose, Eigen::VectorXd::Zero(6));
            // auto jonit_velocity = ft_controller.ZeroMomentVelocityController();
            control_msgs::JointJog joint_deltas;
            for(int i = 0; i < 6; i++)
            {
                if(jonit_velocity(i) > limit ) jonit_velocity(i)=limit, std::cout << "out of limit:  " << i << std::endl;
                else if(jonit_velocity(i) < -limit ) jonit_velocity(i)=-limit, std::cout << "out of limit:  " << i << std::endl;
                joint_deltas.joint_names.push_back(str_name[i]);
                joint_deltas.velocities.push_back(jonit_velocity(i));
            }
            joint_deltas.header.stamp = ros::Time::now();
            jog_vel_pub.publish(joint_deltas);

            loop_rate.sleep();
        }
        // task_frame_pose = task_frame_pose2;
        while(ros::ok())
        {
            Eigen::VectorXd jonit_velocity;
            if((ros::Time::now()-past).toSec() < 10.0)
                jonit_velocity = ft_controller.ForceVelocityController(task_frame_pose, expected_wrench);
            else
                jonit_velocity = ft_controller.ForceVelocityController(task_frame_pose, Eigen::VectorXd::Zero(6));
            
            // auto jonit_velocity = ft_controller.ZeroMomentVelocityController();
            control_msgs::JointJog joint_deltas;
            for(int i = 0; i < 6; i++)
            {
                if(jonit_velocity(i) > limit ) jonit_velocity(i)=limit, std::cout << "out of limit:  " << i << std::endl;
                else if(jonit_velocity(i) < -limit ) jonit_velocity(i)=-limit, std::cout << "out of limit:  " << i << std::endl;
                joint_deltas.joint_names.push_back(str_name[i]);
                joint_deltas.velocities.push_back(jonit_velocity(i));
            }
            joint_deltas.header.stamp = ros::Time::now();
            jog_vel_pub.publish(joint_deltas);

            loop_rate.sleep();
        }
    }

    ros::waitForShutdown();
    states_hub_thread.join();

    return 0;
}