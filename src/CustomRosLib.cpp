#include "CustomRosLib.h"

CustomRosLib::CustomRosLib(ros::NodeHandle &nh):nh_(nh), joint_seed_(6),
    ik_solver_("base_link", "ee_link", "/robot_description", 0.008, 1e-5, TRAC_IK::Distance)
{
    // set up publisher and subscriber
    pos_tra_controller_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/pos_based_pos_traj_controller/command", 1);
    // variables initialization
    for(int i=0;i<6;i++){
        // home_angles_.data.push_back(json_joint_angle[i].asDouble());
        last_computed_angle_.data.push_back(0.0);    // home postion
    }
    
}

CustomRosLib::CustomRosLib(std::shared_ptr<SharedVariable> ptr): joint_seed_(6),
    ik_solver_("base_link", "ee_link", "/robot_description", 0.008, 1e-5, TRAC_IK::Distance)
{
    // set up publisher and subscriber
    string controller_name;
    ros::ServiceClient list_controllers_client = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers_srv;
    if(list_controllers_client.call(list_controllers_srv))
    {
        auto controller_lists = list_controllers_srv.response.controller;
        for(auto controller = controller_lists.begin();controller!=controller_lists.end();controller++)
        {
            if(controller->name ==  "pos_based_pos_traj_controller" || controller->name ==  "vel_based_pos_traj_controller")
            {
                controller_name = controller->name;
                ROS_INFO("The pos controller %s is running", controller->name.c_str());
                break;
            }
        }
    }
    pos_tra_controller_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/"+controller_name+"/command", 1);
    wrench_display_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/servo_vel_display", 1);
    // variables initialization
    for(int i=0;i<6;i++){
        // home_angles_.data.push_back(json_joint_angle[i].asDouble());
        last_computed_angle_.data.push_back(0.0);    // home postion
    }

    shared_variable_ptr_ = ptr;
    joints_name_ = shared_variable_ptr_->joint_names;
    jog_vel_pub_ = nh_.advertise<control_msgs::JointJog>("/jog_arm_server/joint_delta_jog_cmds", 1);
}

CustomRosLib::~CustomRosLib() {}

bool CustomRosLib::CartesianPositionControl(geometry_msgs::Pose target_position, double duration, double delay)
{
    static trajectory_msgs::JointTrajectory joint_tra;
    static std::vector<string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std_msgs::Float64MultiArray target_joint_position;

    joint_tra.joint_names.assign(joint_names.begin(), joint_names.end());
    joint_tra.points.resize(1);
    // inverse kinematics
    target_joint_position = ManipulatorIK(target_position);

    joint_tra.points[0].positions.assign(target_joint_position.data.begin(), target_joint_position.data.end());
    joint_tra.points[0].time_from_start = ros::Duration(duration);
    pos_tra_controller_.publish(joint_tra);
    ros::Duration(duration + delay).sleep();
    return true;
}

void CustomRosLib::JointPositionControl(std::vector<double> target_joint_position, double duration)
{
    static trajectory_msgs::JointTrajectory joint_tra;
    static std::vector<string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    joint_tra.joint_names.assign(joint_names.begin(), joint_names.end());
    joint_tra.points.resize(1);

    joint_tra.points[0].positions.assign(target_joint_position.begin(), target_joint_position.end());
    joint_tra.points[0].time_from_start = ros::Duration(duration);
    pos_tra_controller_.publish(joint_tra);
    ros::Duration(duration).sleep();
}

std_msgs::Float64MultiArray CustomRosLib::ManipulatorIK(geometry_msgs::Pose target_pose)
{
    KDL::JntArray return_joints;
    double it[] = {target_pose.position.x, target_pose.position.y, target_pose.position.z};
    memcpy(desired_eef_pose_.p.data, it, sizeof(it));
    memcpy(desired_eef_pose_.M.data, desired_eef_pose_.M.Quaternion(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w).data,sizeof(desired_eef_pose_.M.data));
    joint_seed_.data << last_computed_angle_.data[0],last_computed_angle_.data[1],last_computed_angle_.data[2],last_computed_angle_.data[3],last_computed_angle_.data[4],last_computed_angle_.data[5];
    int rc = ik_solver_.CartToJnt(joint_seed_, desired_eef_pose_, return_joints);
    
    if(rc<0){
        ROS_ERROR("Did not find IK solution!!!");
        cout << "Did not find IK solution for " << target_pose << endl;
        throw "Did not find IK solution";
    }

    for(int i=0;i<6;i++) 
        last_computed_angle_.data[i] = return_joints.data(i);
        
    return last_computed_angle_;
}

geometry_msgs::Pose CustomRosLib::ListenToTransform(string base_frame, string target_frame)
{
    tf::StampedTransform transform;
    geometry_msgs::Pose target_pose;

    static tf::TransformListener listener(ros::Duration(10));
    listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);

    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z();
    target_pose.orientation.x = transform.getRotation().x();
    target_pose.orientation.y = transform.getRotation().y();
    target_pose.orientation.z = transform.getRotation().z();
    target_pose.orientation.w = transform.getRotation().w();

    return target_pose;
}

tf::StampedTransform CustomRosLib::ListenToTransform_TF(string base_frame, string target_frame)
{
    tf::StampedTransform transform;
    static tf::TransformListener listener(ros::Duration(5));
    listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    return transform;
}

void CustomRosLib::UpdateJointSeed(Eigen::VectorXd joint_states)
{
    last_computed_angle_.data[0] = joint_states[0];
    last_computed_angle_.data[1] = joint_states[1];
    last_computed_angle_.data[2] = joint_states[2];
    last_computed_angle_.data[3] = joint_states[3];
    last_computed_angle_.data[4] = joint_states[4];
    last_computed_angle_.data[5] = joint_states[5];
}

bool CustomRosLib::ActivateController(string controller_name)
{
    ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers.push_back(controller_name);

    if (client.call(srv))
    {
        ROS_INFO("Activate controller %s successfully!", controller_name);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to activate controller %s!", controller_name);
    }

    return false;
}

bool CustomRosLib::DeactivateController(string controller_name)
{
    ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers.push_back(controller_name);

    if (client.call(srv))
    {
        ROS_INFO("Deactivate controller %s successfully!", controller_name.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to deactivate controller %s!", controller_name.c_str());
    }

    return false;
}
bool CustomRosLib::SwitchController(string stop_controller_name, string start_controller_name)
{
    ros::ServiceClient list_controllers_client = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers_srv;
    if(list_controllers_client.call(list_controllers_srv))
    {
        auto controller_lists = list_controllers_srv.response.controller;
        for(auto controller = controller_lists.begin();controller!=controller_lists.end();controller++)
        {
            if(controller->name == start_controller_name && controller->state == "running")
            {
                ROS_INFO("The controller %s is already running", start_controller_name.c_str());
                return true;
            }
        }
    }

    ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers.push_back(start_controller_name);
    srv.request.stop_controllers.push_back(stop_controller_name);

    if (client.call(srv))
    {
        ROS_INFO("Switch controller from %s to %s successfully!", stop_controller_name.c_str(), start_controller_name.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to switch controller from %s to %s successfully!", stop_controller_name.c_str(), start_controller_name.c_str());
    }

    return false;
}

void CustomRosLib::PublishJointVelocity(Eigen::VectorXd joint_velocity)
{
    control_msgs::JointJog joint_deltas;
    for(int i=0;i<joint_velocity.size();i++)
    {
        joint_deltas.joint_names.push_back(joints_name_[i]);
        joint_deltas.velocities.push_back(joint_velocity(i));
    }
    joint_deltas.header.stamp = ros::Time::now();
    jog_vel_pub_.publish(joint_deltas);
}

void CustomRosLib::CheckBox(Eigen::Vector3d center, Eigen::Vector3d range)
{
    auto eef = shared_variable_ptr_->end_effector_state;
    auto upper = center + range;
    auto lower = center - range;
    if(eef.translation()[0] > upper[0] || eef.translation()[0] < lower[0] ||
        eef.translation()[1] > upper[1] || eef.translation()[1] < lower[1] ||
        eef.translation()[2] > upper[2] || eef.translation()[2] < lower[2])
    {
        cout << "The robot is out of bound!" << endl;
        cout << "upper:" << endl << upper << endl << "lower:" << endl << lower << endl;
        cout << "eef" << endl << eef.translation() << endl;

        while(ros::ok())
        {
            ros::Rate rate(200);
            control_msgs::JointJog joint_deltas;
            for(int i=0;i<6;i++)
            {
                joint_deltas.joint_names.push_back(joints_name_[i]);
                joint_deltas.velocities.push_back(0);
            }
            joint_deltas.header.stamp = ros::Time::now();
            jog_vel_pub_.publish(joint_deltas);
            rate.sleep();
        }

    }

}

void CustomRosLib::BroadcastTransform(string parent_frame, string child_frame, geometry_msgs::PoseStamped transformation)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    auto orient = transformation.pose.orientation;
    auto position = transformation.pose.position;
    transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
    transform.setRotation( tf::Quaternion(orient.x, orient.y, orient.z, orient.w) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}


void CustomRosLib::BroadcastTransform(string parent_frame, string child_frame, Eigen::Affine3d transformation)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    auto orient = Eigen::Quaterniond( transformation.linear() );
    auto position = transformation.translation();
    transform.setOrigin( tf::Vector3(position(0,0), position(1,0), position(2,0)) );
    transform.setRotation( tf::Quaternion(orient.x(), orient.y(), orient.z(), orient.w()) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}

void CustomRosLib::WrenchRvizDisplay(Eigen::VectorXd vel, string frame_name, double sacle)
{
    geometry_msgs::WrenchStamped wrench;
    wrench.header.frame_id = frame_name;
    wrench.header.stamp = ros::Time::now();
    vel = vel * sacle;
    wrench.wrench.force.x = vel(0);
    wrench.wrench.force.y = vel(1);
    wrench.wrench.force.z = vel(2);
    wrench.wrench.torque.x = vel(3);
    wrench.wrench.torque.y = vel(4);
    wrench.wrench.torque.z = vel(5);
    wrench_display_pub_.publish(wrench);
}

void CustomRosLib::WrenchRvizDisplay2(Eigen::VectorXd vel, string frame_name, string topic_name, double sacle)
{
    static ros::Publisher wrench_display_pub;
    wrench_display_pub = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
    geometry_msgs::WrenchStamped wrench;
    wrench.header.frame_id = frame_name;
    wrench.header.stamp = ros::Time::now();
    vel = vel * sacle;
    wrench.wrench.force.x = vel(0);
    wrench.wrench.force.y = vel(1);
    wrench.wrench.force.z = vel(2);
    wrench.wrench.torque.x = vel(3);
    wrench.wrench.torque.y = vel(4);
    wrench.wrench.torque.z = vel(5);
    wrench_display_pub.publish(wrench);
}
