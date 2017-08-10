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


void pixelTo3DPoint(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pCloud = *input;

  const int u = 320;
  const int v = 320;
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;
  std::cout << "w: " << width << " h:" << height << std::endl;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  std::cout << "row_step: " << pCloud.row_step << " point_step: " << pCloud.point_step << std::endl;
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

  std::cout << "size of data: " << pCloud.data.size() << " , " << sizeof(pCloud.data) << std::endl;

  // memcpy(&X, &pCloud.data[arrayPosX], sizeof(uint));
  // memcpy(&Y, &pCloud.data[arrayPosY], sizeof(uint));
  // memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(uint));

  // for (int j=0; j < pCloud.row_step*pCloud.height; ++j){
  // 	std::cout << "x: " << pCloud.data[j] << " y: " << pCloud.data[j] << " z: " << pCloud.data[j] << std::endl;	
  // }
  std::cout << "x: " << pCloud.data[arrayPosX] << " y: " << pCloud.data[arrayPosY] << " z: " << pCloud.data[arrayPosZ] << std::endl;


  p.x = float(pCloud.data[arrayPosX]);
  p.y = float(pCloud.data[arrayPosY]);
  p.z = int(Z);

  point_pub.publish(p);

}

// const sensor_msgs::PointCloud2 getPcloud() 
// {
// 	return pCloud;	
// }

// void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
// {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
//     pcl::fromROSMsg( *input, *cloud );
//     sensor_msgs::PointCloud2 toPub;
//     pcl::toROSMsg( *cloud, toPub );
// 	pCloud = toPub;
// 	pCloud=*input;
// 	cloud_pub.publish(pCloud);
// }
// }

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "block_tf");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud and block pixel location
    point_pub = nh.advertise<geometry_msgs::Point>("/block_point", 1);
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, pixelTo3DPoint);

    // Spin
    ros::spin();
}