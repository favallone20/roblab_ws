#include <ros/ros.h>
#include <kinematics_service_msgs/GetFKSolution.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fk_client");
  ros::NodeHandle nh;
  ros::ServiceClient client =  nh.serviceClient<kinematics_service_msgs::GetFKSolution>("fk_service");
  kinematics_service_msgs::GetFKSolution service;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  moveit::core::RobotState robot_state(kinematic_model);
  moveit_msgs::RobotState robot_state_msgs; 


  const std::vector<double> joint_positions = {0, 0.21, 0.05, -1.75, -1.65, 0};
  robot_state.setJointGroupPositions("fanuc", joint_positions);
  robot_state.updateLinkTransforms();

  moveit::core::robotStateToRobotStateMsg(robot_state, robot_state_msgs);

  service.request.robot_state = robot_state_msgs;

  if(client.call(service)){
      ROS_INFO_STREAM(service.response.end_effector);
  }
  else{
      ROS_ERROR("Failed to call service fk_service");
  }
  return 0;
}
