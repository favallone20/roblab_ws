/* -------------------------------------------------------------------
 *
 *
 * Title: fk_client.cpp
 * Author:  Francesco Avallone
 *
 * This module implements a service client to compute FK solution. The solution provided by fk_server module
 * can be compared with the solution provided by ros service compute_fk. 
 *
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_service_msgs/GetFKSolution.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPositionFK.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fk_client");
  ros::NodeHandle nh;
  ros::ServiceClient client =  nh.serviceClient<kinematics_service_msgs::GetFKSolution>("fk_service");
  ros::ServiceClient clientComputeFk =  nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
  kinematics_service_msgs::GetFKSolution service;
  moveit_msgs::GetPositionFK service_msgs_compute_fk;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  moveit::core::RobotState robot_state(kinematic_model);
  moveit_msgs::RobotState robot_state_msgs; 


  const std::vector<double> joint_positions = {0, 0.21, 0.05, -1.75, -1.65, 0};
  robot_state.setJointGroupPositions("fanuc", joint_positions);
  robot_state.updateLinkTransforms();

  moveit::core::robotStateToRobotStateMsg(robot_state, robot_state_msgs);

  service.request.robot_state = robot_state_msgs;

  service_msgs_compute_fk.request.fk_link_names = (kinematic_model->getJointModelGroup("fanuc"))->getLinkModelNames();
  service_msgs_compute_fk.request.header.frame_id = service_msgs_compute_fk.request.fk_link_names[0];
  service_msgs_compute_fk.request.robot_state = robot_state_msgs;

  ROS_INFO_STREAM("Solution with fk_server");
  if(client.call(service)){
      ROS_INFO_STREAM(service.response.end_effector);
  }
  else{
      ROS_ERROR("Failed to call service fk_service");
  }

  ROS_INFO_STREAM("Solution with compute_fk");
  if (clientComputeFk.call(service_msgs_compute_fk)){
     ROS_INFO_STREAM(service_msgs_compute_fk.response.pose_stamped[service_msgs_compute_fk.response.pose_stamped.size()-1]);
  }
  else{
    ROS_ERROR("Failed to call service compute_fk");
  }
  return 0;
}
