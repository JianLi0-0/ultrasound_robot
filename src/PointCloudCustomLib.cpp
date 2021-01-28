#include "PointCloudCustomLib.h"

PointCloudCustomLib::PointCloudCustomLib(/* args */):
    cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{

}

PointCloudCustomLib::~PointCloudCustomLib()
{

}

void PointCloudCustomLib::set_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud)
{
    cloud_ = pointcloud->makeShared();
}

void PointCloudCustomLib::VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(pointcloud);
    viewer.spinOnce();
    // while (!viewer.wasStopped ())
    // {
    //   viewer.spinOnce();
    // }
}

void PointCloudCustomLib::EstimatePointcloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // camera_color_optical_frame
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setNormalEstimationMethod (normal_estimation.COVARIANCE_MATRIX);
    normal_estimation.setMaxDepthChangeFactor(0.02f);
    normal_estimation.setNormalSmoothingSize(10.0f);
    normal_estimation.setInputCloud(pointcloud);
    normal_estimation.compute(*normals);
}

geometry_msgs::PoseArray PointCloudCustomLib::FromNormalsToPoseArray(const geometry_msgs::PoseArray pixel_points, 
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = pointcloud->header.frame_id;
    // std::cout << pose_array.header.frame_id << std::endl;
    // estimatePointcloudNormals(pointcloud, normals);
    for(int i=0;i<pixel_points.poses.size();i++)
    {
        auto pixel = pixel_points.poses[i];
        geometry_msgs::Pose temp_pose;
        auto width = pointcloud->width;
        auto height = pointcloud->height;
        // pixel.position.x : -> (right)  pixel.position.y: \|/ (down)
        auto index = (int)pixel.position.x + (int)pixel.position.y*width;
        
        auto position = pointcloud->points[index].data;
        temp_pose.position.x = position[0];
        temp_pose.position.y = position[1];
        temp_pose.position.z = position[2];

        Eigen::Vector3d z_axis(normals->points[index].normal_x, 
                                normals->points[index].normal_y, 
                                normals->points[index].normal_z);
        // std::cout << "z_axis " << z_axis << std::endl;
        static Eigen::Vector3d x_axis;
        if(i != pixel_points.poses.size()-1)
        {
            auto pixel = pixel_points.poses[i+1];
            auto next_index = (int)pixel.position.x + (int)pixel.position.y*width;
            // number of points must be larger than 1
            auto temp_x_axis = (pointcloud->points[next_index].getVector3fMap() - pointcloud->points[index].getVector3fMap());
            x_axis << temp_x_axis.matrix()(0,0), temp_x_axis.matrix()(1,0), temp_x_axis.matrix()(2,0);
            x_axis.normalize();
        }
        Eigen::Matrix3d R;
        auto y_axis = z_axis.cross(x_axis);
        y_axis.normalize();
        R << x_axis, y_axis, z_axis;
        Eigen::Quaterniond q(R);
        temp_pose.orientation.x = q.x();
        temp_pose.orientation.y = q.y();
        temp_pose.orientation.z = q.z();
        temp_pose.orientation.w = q.w();
        pose_array.poses.push_back(temp_pose);
    }

    return pose_array;
}