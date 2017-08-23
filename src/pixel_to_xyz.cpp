#include <ros/ros.h>

#include <vector>
#include <iostream>

#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

/*
Code include methods derived from:
[0] http://sauravag.com/2016/11/how-to-get-depth-xyz-of-a-2d-pixel-from-pointcloud2-or-kinect-data/
[1] https://stackoverflow.com/questions/12299870/
*/

//--Declarations--//
int controllerState = 0;
sensor_msgs::PointCloud2 pCloud_global;
ros::Publisher point_pub;
cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
cv::Mat distCoeffs(5,1,cv::DataType<double>::type);

std::string robot_frame_global = "arm_link_0";//"camera_link";
std::string camera_frame_global = "camera_depth_optical_frame";//"camera_link";
double object_height_global =  -0.080; //-0.0687; //-0.0167221; //in m, from arm_link_0 frame //-0.035;

void transformCameraToRobot(geometry_msgs::PointStamped &camera_point,
  geometry_msgs::PointStamped &arm_link_point);

void pixelTo3DPointWithPartialCloud(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &arm_link_point);

bool pixelToCamera3DPoint(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &camera_point);

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &arm_link_point);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

void pixel_cb(geometry_msgs::Point pixel_point);

void camera_info_cb(const sensor_msgs::CameraInfo input);

void current_state_cb(const std_msgs::Int32& state );

const sensor_msgs::PointCloud2 getPcloud() {return pCloud_global;}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pixel_tf");
  ros::NodeHandle nh;

  // Create a publisher for pixel coordinates
  point_pub = nh.advertise<geometry_msgs::PointStamped>(
    "/block_point", 1);

  // Create a ROS subscriber for the point cloud
  ros::Subscriber sub = nh.subscribe (
    "camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS subscriber for the pixel values
  ros::Subscriber pixel_sub = nh.subscribe (
    "/rgb_seg/block_location", 1, pixel_cb);

  // Create a ROS subscriber for the camera calibration info
  ros::Subscriber camera_info_sub = nh.subscribe (
    "/camera/rgb/camera_info", 1, camera_info_cb);

  //Create a ROS subscriber for the current state
  ros::Subscriber current_state = nh.subscribe (
    "control_current_state", 1, current_state_cb);

  // Spin
  ros::spin();
}


void transformCameraToRobot(geometry_msgs::PointStamped &camera_point,
  geometry_msgs::PointStamped &arm_link_point){

  // listen for transform
  tf::TransformListener listener;
  listener.waitForTransform(camera_frame_global, robot_frame_global,
    ros::Time(0), ros::Duration(10.0));

  // transform camera_point to arm_link_point
  try
  {
    listener.transformPoint(robot_frame_global, camera_point, arm_link_point);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point : %s",
      ex.what());
  }

  return;
}

void pixelTo3DPointWithPartialCloud(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &arm_link_point){

  // read pixel points, depth information and convert them to robot frame
  std::vector<cv::Point2f> imagePoints;
  std::vector<cv::Point3f> objectPoints;
  geometry_msgs::PointStamped temp_camera_point, temp_robot_point;

  int points_added = 0;
  int temp_u[] = {100, 300, 500, 500, 300, 100, 100, 500, 300};
  int temp_v[] = {100, 200, 300, 100, 100, 300, 200, 200, 300};
  int idx = 0;
  while ( (idx < 9) ) //(points_added < 4) &&
  {
    if ( pixelToCamera3DPoint(pCloud,
      temp_u[idx], temp_v[idx], temp_camera_point) ){
      imagePoints.push_back(cv::Point2f(
        float(temp_u[idx]), float(temp_v[idx])));
      transformCameraToRobot(temp_camera_point, temp_robot_point);
      objectPoints.push_back(cv::Point3f(
        temp_robot_point.point.x, temp_robot_point.point.y,
        temp_robot_point.point.z));
      points_added++;
    }
    idx += 1;
  }

ROS_WARN("Points added %d", points_added);

  // read camera parameters
  cv::Mat rvec(1,3,cv::DataType<double>::type);
  cv::Mat tvec(1,3,cv::DataType<double>::type);
  cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

  // compute transform parameters
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Rodrigues(rvec,rotationMatrix);

  // compute transform
  cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); //u,v,1
  uvPoint.at<double>(0,0) = float(u);
  uvPoint.at<double>(1,0) = float(v);
  cv::Mat tempMat, tempMat2;
  double s;
  tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
  tempMat2 = rotationMatrix.inv() * tvec;
  s = object_height_global + tempMat2.at<double>(2,0); //0. represents the height
  s /= tempMat.at<double>(2,0);

ROS_WARN("Projection variable %.4f", s);

  cv::Mat temp_point = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
  std::cout << "P = " << rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec) << std::endl;

  // save the point in robot frame
  arm_link_point.header.frame_id = robot_frame_global;
  arm_link_point.header.stamp = ros::Time();
  arm_link_point.point.x = float(temp_point.at<double>(0,0));
  arm_link_point.point.y = float(temp_point.at<double>(1,0));
  arm_link_point.point.z = float(temp_point.at<double>(2,0));

  return;
}

bool pixelToCamera3DPoint(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &camera_point){

  int width = pCloud.width;
  int height =  pCloud.height;

  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;
// ROS_WARN("pixel and array position %d, %d, %d", u, v, arrayPosition);

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset;
  int arrayPosY = arrayPosition + pCloud.fields[1].offset;
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset;

  // copy point cloud values at these locations
  float X = 0.0, Y = 0.0, Z = 0.0;
  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  if ( isnan(X) ){
    return false;
  }
  else{
    // save information from depth cloud to point stamped
    camera_point.header.frame_id = camera_frame_global;
    camera_point.header.stamp = ros::Time();
    camera_point.point.x = X;
    camera_point.point.y = Y;
    camera_point.point.z = Z;
  }
}

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud,
  const int u, const int v, geometry_msgs::PointStamped &arm_link_point){

  int width = pCloud.width;
  int height =  pCloud.height;

  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;
if (pCloud.data.size() == 0) return;
ROS_WARN("pixel and array position %d, %d, %d", u, v, arrayPosition);
std::cout << "size of data: " << pCloud.data.size() << " , " << sizeof(pCloud.data) << std::endl;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset;
  int arrayPosY = arrayPosition + pCloud.fields[1].offset;
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset;

// ROS_WARN("array positions %d, %d, %d", arrayPosX, arrayPosY, arrayPosZ);
  // copy point cloud values at these locations
  float X = 0.0, Y = 0.0, Z = 0.0;
  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

// std::cout << "x: " << X << " y: " << Y << " z: " << Z << std::endl;
// ROS_WARN("coordinates %.3f, %.3f, %.3f", X, Y, Z);

  if ( isnan(float(X)) ){
    ROS_WARN("Corresponding point in depth cloud missing!");
    // need to co
    pixelTo3DPointWithPartialCloud(pCloud, u, v, arm_link_point);
  }
  else{
ROS_WARN("Point available");
    // save information from depth cloud to point stamped
    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = camera_frame_global;
    camera_point.header.stamp = ros::Time();
    camera_point.point.x = float(X);
    camera_point.point.y = float(Y);
    camera_point.point.z = float(Z);
    // transform point to arm_link_0
    transformCameraToRobot(camera_point, arm_link_point);
  }

  if (controllerState == 1)
  { 
    // publish the point
    point_pub.publish(arm_link_point);
  }
}

void current_state_cb( const std_msgs::Int32& state )
{
  controllerState = state.data;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pCloud_global = *input;
}

void pixel_cb(geometry_msgs::Point pixel_point)
{
  // Read pixel values
  int u = pixel_point.x;
  int v = pixel_point.y;

// ROS_WARN("Pixel points %d, %d", u, v);

  // Call method to convert pixel values to xyz in base link frame
  geometry_msgs::PointStamped pixelXYZ;
  sensor_msgs::PointCloud2 pCloud = getPcloud();
  pixelTo3DPoint(pCloud, u, v, pixelXYZ);
}

void camera_info_cb(const sensor_msgs::CameraInfo input){

  int x, y;
  for (int idx = 0; idx < 9; idx++){
    x = idx / 3;
    y = idx - (3 * x);
    cameraMatrix.at<double>(x, y) = input.K[idx];
  }

  for (int idx = 0; idx < 5; idx++){
    distCoeffs.at<double>(idx, 0) = input.D[idx];
  }

// std::cout << " Camera matrix : " << cameraMatrix << std::endl;
// std::cout << " Dist coeffs : " << distCoeffs << std::endl;

  return;
}
