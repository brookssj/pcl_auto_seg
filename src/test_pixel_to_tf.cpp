#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>

#include <vector>
#include <iostream>

//--Declarations--//
ros::Publisher point_pub;
ros::Publisher cloud_pub;
sensor_msgs::PointCloud2 pCloud;
geometry_msgs::Point p;
int u;
int v;
int controllerState = 0;


void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
{
  if (controllerState != 4)
    return;

  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;
  std::cout << "w: " << width << " h:" << height << std::endl;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;
  std::cout << "array position: " << arrayPosition << std::endl;


  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
  std::cout << "x: " << arrayPosX << " y: " << arrayPosY << " z: " << arrayPosZ << std::endl;

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));
  std::cout << "x: " << pCloud.data[arrayPosX] << " y: " << pCloud.data[arrayPosY] << " z: " << pCloud.data[arrayPosZ] << std::endl;


  p.x = X;
  p.y = Y;
  p.z = Z;

  point_pub.publish(p);

}

const sensor_msgs::PointCloud2 getPcloud() 
{
  return pCloud;  
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg( *input, *cloud );
    sensor_msgs::PointCloud2 toPub;
    pcl::toROSMsg( *cloud, toPub );
  pCloud = toPub;*/
  pCloud=*input;
  cloud_pub.publish(pCloud);
}

void pixel_cb(geometry_msgs::Point pixel_point)
{
  u = pixel_point.x;
  v = pixel_point.y;
  //std::cout << "u: " << u << " v: " << v << std::endl;
  pCloud = getPcloud();
  pixelTo3DPoint(pCloud, u, v, p);
}

void current_state_cb( const std_msgs::Int32& state )
{
  controllerState = state.data;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "block_tf");
    ros::NodeHandle nh;

    //Creat a publisher for block point
    point_pub = nh.advertise<geometry_msgs::Point>("/block_point", 1);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);


    // Create a ROS subscriber for the input point cloud and block pixel location
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber pixel_sub = nh.subscribe ("/rgb_seg/block_location", 1, pixel_cb);

    // Create a subscriber for the current state
    ros::Subscriber stateSub = nh.subscribe("/control_current_state", 1, current_state_cb );
 
    // Spin
    ros::spin();
}