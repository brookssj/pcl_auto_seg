#include <ros/ros.h>

#include <vector>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <brics_actuator/JointPositions.h>


ros::Publisher arm_state_pub;
int controllerState = 0;


void joint_state_cb(const brics_actuator::JointPositions state);
//std_msgs::Float32MultiArray array_msg;
//array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

void current_state_cb(const std_msgs::Int32& state);
//std::vector<double> GetArmPosition(); 



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "arm_joint_state");
  ros::NodeHandle nh;

  // Create a publisher for arm joint state
  arm_state_pub = nh.advertise<std_msgs::Float32MultiArray>(
    "/current_arm_state", 1);

  //Create a ROS subscriber for all joint states
  ros::Subscriber joint_state_sub = nh.subscribe (
    "/arm_1/arm_controller/position_command", 1, joint_state_cb);

  //Create a ROS subscriber for the current state
  /*ros::Subscriber current_state = nh.subscribe (
    "control_current_state", 1, current_state_cb);*/

  // Spin
  ros::spin();
}

void joint_state_cb(const brics_actuator::JointPositions state)
{
	//if (controllerState == 1)
	//{
  std_msgs::Float32MultiArray array_msg;
	array_msg.data.resize(5);
	array_msg.data[0] = float(state.positions[0].value);
	array_msg.data[1] = float(state.positions[1].value);
	array_msg.data[2] = float(state.positions[2].value);
	array_msg.data[3] = float(state.positions[3].value);
	array_msg.data[4] = float(state.positions[4].value);
	arm_state_pub.publish(array_msg);
	for( int i = 0; i < state.positions.size(); ++i )
	{
		std::cout << "Joint " << i+1 << ":  " << state.positions[i].value << std::endl;
	}
  std::cout << " " << std::endl;
	//}
}

/*std::vector<double> GetArmPosition() 
{
  return arm_state;
  std::cout << "arm joint 1:" << arm_state[0] << std::endl;
}*/

/*void current_state_cb( const std_msgs::Int32& state )
{
  controllerState = state.data;
}*/