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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_auto_seg/feature_cloud.h"
#include "pcl_auto_seg/template_alignment.h"


#include <vector>
#include <iostream>

//--Declarations--//
ros::Publisher cloud_pub;
int u;
int v;

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
{
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  p.x = X;
  p.y = Y;
  p.z = Z;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg( *input, *cloud );
    sensor_msgs::PointCloud2 toPub;
    pcl::toROSMsg( *cloud, toPub );
	cloud_pub.publish(toPub);*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg( *input, *cloud );

	
	// --- Z-Filter And Downsample Cloud --- //

	// Preprocess the cloud by removing distant points
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud( cloud );
	pass_z.setFilterFieldName( "z" );
	pass_z.setFilterLimits( 0, 1.75 );
	pass_z.filter( *cloud );

	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud( cloud );
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits( -0.5, 0.2 );
	pass_y.filter( *cloud );
	
	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud( cloud );
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits( -0.5, 0.5 );
	pass_x.filter( *cloud );

	// It is possible to not have any points after z-filtering (ex. if we are looking up).
	// Just bail if there is nothing left.
	if( cloud->points.size() == 0 )
		return;

	//visualize( cloud, visualizer_o_Ptr );

	
	// --- Calculate Scene Normals --- //

	pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setKSearch(10);
	normEst.setInputCloud( cloud );
	normEst.compute( *pSceneNormals );


	// --- Get Rid Of Floor --- //

	pcl::PointIndices::Ptr inliers_plane( new pcl::PointIndices );
	pcl::ModelCoefficients::Ptr coefficients_plane( new pcl::ModelCoefficients );

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1; 
	seg1.setOptimizeCoefficients( true );
	seg1.setModelType( pcl::SACMODEL_NORMAL_PLANE );
	seg1.setNormalDistanceWeight( 0.05 );
	seg1.setMethodType( pcl::SAC_RANSAC );
	seg1.setMaxIterations( 100 );
	seg1.setDistanceThreshold( 0.075 );
	seg1.setInputCloud( cloud );
	seg1.setInputNormals( pSceneNormals );
	// Obtain the plane inliers and coefficients
	seg1.segment( *inliers_plane, *coefficients_plane );

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud( cloud );
	extract.setIndices( inliers_plane );
	extract.setNegative( false );

	// Write the planar inliers to disk
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ> );
	extract.filter( *cloud_plane );

	// Remove the planar inliers, extract the rest
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredScene( new pcl::PointCloud<pcl::PointXYZ> );
	extract.setNegative( true );
	extract.filter( *filteredScene );

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr filteredSceneNormals( new pcl::PointCloud<pcl::Normal> );
	extract_normals.setNegative( true );
	extract_normals.setInputCloud( pSceneNormals );
	extract_normals.setIndices( inliers_plane );
	extract_normals.filter( *filteredSceneNormals );	

	if( filteredScene->points.size() == 0 )
		return;
	
	
	// --- Set Our Target Cloud --- //

	// Assign to the target FeatureCloud
	FeatureCloud target_cloud;
	target_cloud.setInputCloud( filteredScene );
	

	// --- Publish --- //

	sensor_msgs::PointCloud2 toPub;
    pcl::toROSMsg( *filteredScene, toPub );
	cloud_pub.publish(toPub);
}

void pixel_cb(geometry_msgs::Point pixel_point)
{
	u = pixel_point.x;
	v = pixel_point.y;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "block_tf");
    ros::NodeHandle nh;

    // Create a ROS publisher for the pose of the block relative to the ASUS.
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

    // Create a ROS subscriber for the input point cloud and block pixel location
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber pixel_sub = nh.subscribe ("/rgb_seg/block_location", 1, pixel_cb);
    
    // Spin
    ros::spin();
}