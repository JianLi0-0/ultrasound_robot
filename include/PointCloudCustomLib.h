#ifndef POINT_CLOUD_CUSTOM_LIB_H
#define POINT_CLOUD_CUSTOM_LIB_H

#include <geometry_msgs/PoseArray.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

class PointCloudCustomLib
{
    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    public:

        PointCloudCustomLib(/* args */);
        ~PointCloudCustomLib();
        void EstimatePointcloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals);
        geometry_msgs::PoseArray FromNormalsToPoseArray(const geometry_msgs::PoseArray pixel_points, 
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals);
        void set_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud);
        void VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud);
};




#endif