#include <ros/ros.h>
#include <termios.h>
#include <Eigen/Dense>
#include <thread>
#include <math.h> 
#include <pcl/filters/filter.h>
#include <geometry_msgs/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#define med_num 5
#define PI 3.14159265359
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud cloud;
pthread_mutex_t shared_variables_mutex_;
pthread_mutex_t point_shared_variables_mutex_;

std::vector<double> center = {0, 0};
double size = 0.015;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pthread_mutex_lock(&point_shared_variables_mutex_); 
    pcl::fromROSMsg(*msg, cloud);
    pthread_mutex_unlock(&point_shared_variables_mutex_);
    //cloud = *msg;
    cloud_ptr = cloud.makeShared();
    // printf ("Cloud: width = %d, height = %d\n", cloud_ptr->width, cloud_ptr->height);
    //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_extration(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr, double size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr return_point (new pcl::PointCloud<pcl::PointXYZ>); 
    pthread_mutex_lock(&shared_variables_mutex_); 
    double left = center[0] - size/2.;
    double up = center[1] - size/2.; 
    double right = center[0] + size/2.;
    double down = center[1] + size/2.;
    pthread_mutex_unlock(&shared_variables_mutex_); 
    for(int i = 0; i < in_pc_ptr->width; i++)
    {
       double width = in_pc_ptr->points[i].x;
       double height = in_pc_ptr->points[i].y;
       
       if(width > left &&
       width < right &&
       height > up &&
       height < down)
       {
          pcl::PointXYZ temp_cutted_point;
          temp_cutted_point.x = in_pc_ptr->points[i].x;
          temp_cutted_point.y = in_pc_ptr->points[i].y;
          temp_cutted_point.z = in_pc_ptr->points[i].z;
          if(temp_cutted_point.x)
          //cout << temp_cutted_point.x <<"  " << temp_cutted_point.y <<"  " << temp_cutted_point.z<< endl;
          return_point->push_back(temp_cutted_point);
       }
    }
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*return_point, *return_point, mapping);
    return return_point;
};

bool curvature_sort(pcl::Normal i, pcl::Normal j) { return (i.curvature <j.curvature); }

pcl::Normal med_filter(pcl::Normal input_norm)
{
    static std::vector<pcl::Normal> norm_ptr_vec;
    if(input_norm.curvature)
    if(norm_ptr_vec.empty())
    {      
       norm_ptr_vec.resize(med_num);
       
       for(int i = 0; i < med_num; i++)
       {
          norm_ptr_vec[i] = input_norm;
       }
       return input_norm;
    }
    else
    { 
       //cout <<"Here" <<endl;
       //norm_ptr_vec[med_num-1] is dropped;
       pcl::Normal temp_norm = norm_ptr_vec[0];
       //input
       for(int i = med_num -1; i > 0; i--)
       {
          norm_ptr_vec[i] = norm_ptr_vec[i-1];
       }

       norm_ptr_vec[0] = input_norm;

       std::vector<pcl::Normal> temp_norm_ptr_vec = norm_ptr_vec;

       std::sort(temp_norm_ptr_vec.begin(), temp_norm_ptr_vec.end(), curvature_sort);
       for(int i = 0; i < med_num; i++)
       {
          cout << temp_norm_ptr_vec[i].curvature << " ";  
       }
       cout << endl;
       for(int i = 0; i < med_num; i++)
       {
          cout << temp_norm_ptr_vec[i].normal_x << " ";  
       }
       cout << endl;
       for(int i = 0; i < med_num; i++)
       {
          cout << temp_norm_ptr_vec[i].normal_y << " ";  
       }
       cout << endl;
       for(int i = 0; i < med_num; i++)
       {
          cout << temp_norm_ptr_vec[i].normal_z << " ";  
       }
       cout << endl;

       return temp_norm_ptr_vec[2];
       //calculate media
       //out put
    }
}

pcl::PointXYZ central_point(pcl::ModelCoefficients::Ptr coefficients)
{
    pthread_mutex_lock(&shared_variables_mutex_); 
    std::vector<double> _center = center;
    pthread_mutex_unlock(&shared_variables_mutex_); 
        
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];
    pcl::PointXYZ point_center;
    point_center.x = _center[0];
    point_center.y = _center[1];
    point_center.z = -1 *(a* _center[0] +  b* _center[1] + d) / c;
    return point_center;
}

void change_center(char c)
{
      pthread_mutex_lock(&shared_variables_mutex_); 
      if (c == 'a')   //left
      {        
          center[0] = 0.01;
          center[1] = 0;
      }
      else if (c == 'd')  // right
      {
          center[0] = -0.01;
          center[1] = 0; 
      } 
      else if (c == 'w')  // up
      {
          center[0] = 0;      
          center[1] = 0.01;
      }
      else if (c == 's')  // down
      {        
          center[0] = 0;            
          center[1] = -0.01;
      }
      else 
      {
        center[0] = 0; 
        center[1] = 0; 
      };
      pthread_mutex_unlock(&shared_variables_mutex_);
}

void huang_s_help()
{
    cout << "You can press key board to contol the motion of the end-effector" << endl << endl;
    cout << "       w       " << endl << endl;
    cout << "    a  s  d       " << endl << endl; 
    cout << "    [space]       " << endl << endl;
}

int getch()
{
    struct termios oldSettings, newSettings;

    tcgetattr( fileno( stdin ), &oldSettings );
    newSettings = oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &newSettings );    
    huang_s_help();

    while ( ros::ok() )
    {
        fd_set set;
        struct timeval tv;

        tv.tv_sec = 10;
        tv.tv_usec = 0;

        FD_ZERO( &set );
        FD_SET( fileno( stdin ), &set );

        int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

        if( res > 0 )
        {
            char c;
            read( fileno( stdin ), &c, 1 );
            change_center(c);
        }
        else if( res < 0 )
        {
            perror( "select error" );
            break;
        }
        else;
        usleep(200);
    }

    tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
}

void normal_to_Tmatrix(pcl::PointXYZ normal_point, pcl::ModelCoefficients::Ptr coefficients)
{
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];
    Eigen::Vector3d z_axis = {-1*a, b, -1*c}; 
    Eigen::Vector3d z_Unit = {0,0,-1};
    double theta = acos(z_axis.dot(z_Unit) / z_axis.norm()) *180/PI;
    if(theta > 90) 
    {
        cout << "The quality of point clouds is poor" << endl;
        return;
    }
     
    Eigen::Vector3d z_a_normalized = z_axis.normalized();
    Eigen::Vector3d x_axis_projected;
    x_axis_projected[0] = 1;
    x_axis_projected[1] = 0;
    x_axis_projected[2] = -1 *(a* x_axis_projected[0] +  b* x_axis_projected[1] + d) / c; 

    Eigen::Vector3d x_a_normalized = x_axis_projected.normalized();    


    Eigen::Vector3d y_axis = z_a_normalized.cross(x_a_normalized);
    Eigen::Vector3d y_a_normalized = y_axis.normalized();

    Eigen::Matrix3d Rotation_matrix;
    Rotation_matrix << 
    x_a_normalized[0], x_a_normalized[1], x_a_normalized[2],
    y_a_normalized[0], y_a_normalized[1], y_a_normalized[2],
    z_a_normalized[0], z_a_normalized[1], z_a_normalized[2];
    Eigen::Quaterniond q(Rotation_matrix);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(normal_point.x, normal_point.y, normal_point.z));
    tf::Quaternion tfq(q.x(), q.y(), q.z(), q.w());
    transform.setRotation(tfq);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "servo_target"));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_subscribe");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, callback);
  ros::Publisher affine_pub = nh.advertise<geometry_msgs::Transform>("/current_transform", 1);
  ros::AsyncSpinner spinner(3); // Use 3 threads
  spinner.start(); 	
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  std::thread keyboard_thread(getch);
  
  // We add the point cloud to the viewer and pass the color handler
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  // Create the segmentation object 
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr result_point_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr result_norm_ptr(new pcl::PointCloud<pcl::Normal>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //viewer.setPosition(800, 400); // Setting visualiser window position
  while (!viewer.wasStopped () && ros::ok())
  { 
    if(!cloud_ptr->empty())
    {
        //Color single_color(cloud_ptr, 255, 0, 0);
        pthread_mutex_lock(&point_shared_variables_mutex_); 
        pcl::copyPointCloud(*cloud_ptr, *temp_pc);
        pthread_mutex_unlock(&point_shared_variables_mutex_);
        Color green(temp_pc->makeShared(), 0, 255, 0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr operate_pc_ptr = point_cloud_extration(temp_pc->makeShared(), size);
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.05);
        seg.setInputCloud (operate_pc_ptr);      
        seg.segment (*inliers, *coefficients); 

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        pcl::Normal norm_point;
        norm_point.normal_x = -1*coefficients->values[0];
        norm_point.normal_y = -1*coefficients->values[1];
        norm_point.normal_z = -1*coefficients->values[2];
        pcl::PointXYZ normal_point = central_point(coefficients);
        result_point_ptr -> push_back(normal_point);
        result_norm_ptr -> push_back(norm_point);

        normal_to_Tmatrix(normal_point, coefficients);
        //viewer.addCoordinateSystem (0.1, cMo, "normal_point", 0);

        //添加需要显示的点云数据
        viewer.addPointCloud<pcl::PointXYZ> (temp_pc->makeShared(), green, "sample cloud");
        Color red(operate_pc_ptr, 255, 0, 0);     

        viewer.addPointCloud<pcl::PointXYZ> (operate_pc_ptr, red, "operate");     
        
        //viewer.addPointCloud (cloud_ptr, source_cloud_color_handler, "original_cloud");
        viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>  (result_point_ptr, result_norm_ptr, 1, 0.1, "normals");
        viewer.addCoordinateSystem (0.05, "cloud", 0);
        viewer.setBackgroundColor(0.1, 0.1, 0.1, 0); // Setting background to a dark grey
        //viewer.setPointCloudRenderingPronormalsperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normals");
        operate_pc_ptr->~PointCloud();
        viewer.spinOnce ();
        while (!viewer.wasStopped () && ros::ok())
        {
          pthread_mutex_lock(&point_shared_variables_mutex_); 
          pcl::copyPointCloud(*cloud_ptr, *temp_pc);
          pthread_mutex_unlock(&point_shared_variables_mutex_);
          pcl::PointCloud<pcl::PointXYZ>::Ptr operate_pc_ptr_in = point_cloud_extration(temp_pc->makeShared(), size);
          //operate_pc_ptr = point_cloud_extration(temp_pc.makeShared(), size);
          if(operate_pc_ptr_in ->size() < 10) continue;
          viewer.updatePointCloud<pcl::PointXYZ> (temp_pc->makeShared(), green, "sample cloud");
          viewer.updatePointCloud<pcl::PointXYZ> (operate_pc_ptr_in, red, "operate");
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          seg.setInputCloud (operate_pc_ptr_in);
          seg.segment (*inliers, *coefficients);
          if (inliers->indices.size () == 0)
          {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
          }
          pcl::Normal norm_point;
          norm_point.normal_x = -1*coefficients->values[0];
          norm_point.normal_y = -1*coefficients->values[1];
          norm_point.normal_z = -1*coefficients->values[2];
          pcl::PointXYZ normal_point = central_point(coefficients);
          normal_to_Tmatrix(normal_point, coefficients);
          //viewer.updateCoordinateSystemPose ("normal_point", cMo);
          result_point_ptr -> clear();
          result_point_ptr -> push_back(normal_point);
          result_norm_ptr -> clear();
          result_norm_ptr -> push_back(norm_point);
          std::vector<pcl::visualization::Camera> cam; 
          viewer.getCameras(cam); 
          viewer.removePointCloud("normals", 0);
          //viewer.removePointCloud("operate", 0);
          //viewer.addPointCloud<pcl::PointXYZ> (operate_pc_ptr, red, "operate");     
          viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (result_point_ptr, result_norm_ptr, 1, 0.1, "normals");

          //operate_pc_ptr_in->~PointCloud();

          viewer.spinOnce ();
        }
     }
  } 
}