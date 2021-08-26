/* -------------------------------------------------------------------
 *
 *
 * Title: IK_client.cpp
 * Author:  Francesco Avallone
 *
 * This module implements an action client to compute all the inverse kinematics solutions
 * for a given end effector pose. Moreover, moveit_visual_tools class is used to show the
 * IK solutions on Rviz. 
 *
 * -------------------------------------------------------------------
 */

#include <actionlib/client/simple_action_client.h>
#include <kinematics_action_msgs/GetIKSolutionsAction.h>
#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>

#include <thread>

typedef actionlib::SimpleActionClient<kinematics_action_msgs::GetIKSolutionsAction> Client;

std::atomic<bool> exit_thread_flag{ false };


void publishJointState(ros::Publisher joint_state_publisher, sensor_msgs::JointState joint_state)
{
  while (!exit_thread_flag)
  {
    joint_state_publisher.publish(joint_state);
  }
}

void showPose(moveit_visual_tools::MoveItVisualTools visual_tools, ros::Publisher joint_state_publisher,
              sensor_msgs::JointState joint_state)
{
  exit_thread_flag = false;
  std::thread t1 = std::thread(publishJointState, joint_state_publisher, joint_state);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualize another solution");
  t1.detach();
  exit_thread_flag = true;
}

void printNewPoses(robot_state::RobotState robot_state)
{
  ROS_INFO_STREAM("---------------------------------- New Poses ----------------------------------");
  std::vector<std::string> links;
  links.push_back("base_link");
  links.push_back("link1");
  links.push_back("link2");
  links.push_back("link3");
  links.push_back("link4");
  links.push_back("link5");
  links.push_back("link6");
  links.push_back("flange");

  for (int i = 0; i < links.size(); i++)
  {
    const Eigen::Affine3d& end_effector_state = robot_state.getGlobalLinkTransform(links[i]);
    geometry_msgs::Pose pose = tf2::toMsg(end_effector_state);
    ROS_INFO_STREAM(links[i] << ": " << pose);
  }

  ROS_INFO_STREAM("-----------------------------------------------------------------------------");
}

void doneIK_CB(const actionlib::SimpleClientGoalState& state,
               const kinematics_action_msgs::GetIKSolutionsResultConstPtr& result)
{
  ros::NodeHandle nh;
  ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
  sensor_msgs::JointState joint_state;
  ros::Rate loopRate(10);

  ROS_INFO_STREAM("-----------------------------------------------------------------------------");
  ROS_INFO_STREAM("Solutions: "
                  << "\n");

  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualize the first solution");

  for (int i = 0; i < result->ik_solutions.size(); i++)
  {
    // ROS_INFO_STREAM(result->ik_solutions[i]<<" \n"); qui funziona, restituisce quello che deve
    joint_state = result->ik_solutions[i].joint_state;
  }
  for (int i = 0; i < result->ik_solutions.size(); i++)
  {
    showPose(visual_tools, joint_state_publisher, result->ik_solutions[i].joint_state);
  }

  ROS_INFO_STREAM(result->ik_solutions.size());
  ROS_INFO_STREAM("-----------------------------------------------------------------------------");
}

void activeServer_CB()
{
  ROS_INFO_STREAM("Goal just went active \n");
}

void feedback_CB(const kinematics_action_msgs::GetIKSolutionsFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Feedback: " << feedback->ik_solution << " \n");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "IK_client");
  Client aclient("IKActionServer", true);
  kinematics_action_msgs::GetIKSolutionsGoal goal;

  aclient.waitForServer();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotState robot_state(kinematic_model);
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("fanuc");

  const Eigen::Affine3d& end_effector_state = robot_state.getGlobalLinkTransform("flange");
  geometry_msgs::Pose pose = tf2::toMsg(end_effector_state);
  ROS_INFO_STREAM("Pose end_effector:" << pose << "\n");

  robot_state.setToRandomPositions();
  printNewPoses(robot_state);

  robot_state::robotStateToRobotStateMsg(robot_state, goal.robot_state);

  goal.pose_end_effector_goal = pose;

  aclient.sendGoal(goal, &doneIK_CB, &activeServer_CB, &feedback_CB);
  aclient.waitForResult();

  return 0;
}
