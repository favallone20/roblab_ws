/* -------------------------------------------------------------------
 *
 *
 * Title: IK_server.cpp
 * Author:  Francesco Avallone
 *
 * This module implements an action server to compute all the inverse kinematics solutions
 * for a given end effector pose.
 *
 * -------------------------------------------------------------------
 */

#include <actionlib/server/simple_action_server.h>
#include <kinematics_action_msgs/GetIKSolutionsAction.h>
#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.h>

class IKActionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kinematics_action_msgs::GetIKSolutionsAction> as_;
  std::string action_name_;

  kinematics_action_msgs::GetIKSolutionsFeedback feedback;
  kinematics_action_msgs::GetIKSolutionsResult result;

public:
  IKActionServer(std::string name)
    : as_(nh_, name, boost::bind(&IKActionServer::executeCB, this, _1), false), action_name_(name)
  {
    as_.start();
  }

  ~IKActionServer(void)
  {
  }

  bool checkNewSolution(const std::vector<std::vector<double>> &solutions, const std::vector<double> &solution)
  {
    for (int i = 0; i < solutions.size(); i++)
    {
      for (int j = 0; j < solutions[i].size(); j++)
      {
        if (solutions[i][j] == solution[j])
        {
          return false;
        }
      }
    }
    return true;
  }

  void getSolutions(const kinematics_action_msgs::GetIKSolutionsGoalConstPtr &goal,
                                                robot_state::RobotState robot_state,
                                                 const  moveit::core::JointModelGroup *joint_model_group)
  {
    double timeout = 10;
    std::vector<double> solution;
    std::vector<std::vector<double>> solutions;
    const std::vector<double>& position = solution; 
    moveit_msgs::MoveItErrorCodes error_code;
    const kinematics::KinematicsQueryOptions opt;

    std::vector<double> joint_position = goal->robot_state.joint_state.position;
    const geometry_msgs::Pose &pose = goal->pose_end_effector_goal;
    const kinematics::KinematicsBaseConstPtr solver = joint_model_group->getSolverInstance();

    int cnt = 0;
    while (cnt < 10000)
    {
      solver->searchPositionIK(pose, joint_position, timeout, solution, error_code, opt);
      if (checkNewSolution(solutions, solution) == true && error_code.val == error_code.SUCCESS)
      {
        solutions.push_back(solution);
        robot_state.setJointGroupPositions(joint_model_group, solution);
        moveit_msgs::RobotState robot_state_msgs;
        robot_state::robotStateToRobotStateMsg(robot_state, robot_state_msgs);
        feedback.ik_solution = robot_state_msgs;
        as_.publishFeedback(feedback);
        result.ik_solutions.push_back(robot_state_msgs);
      }

      if (result.ik_solutions.size() >= 8)
      {
        cnt = 10000;
      }

      cnt++;
      robot_state.setToRandomPositions(joint_model_group);
      robot_state.copyJointGroupPositions(joint_model_group, joint_position);
    }
  }

  void executeCB(const kinematics_action_msgs::GetIKSolutionsGoalConstPtr &goal)
  {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(kinematic_model);
    std::vector<std::vector<double>> solutions;
    robot_state::robotStateMsgToRobotState(goal->robot_state, robot_state);

    const  moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("fanuc");
    getSolutions(goal, robot_state, joint_model_group);

    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result);
    ROS_INFO_STREAM(solutions.size());
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "IKActionServer");
  IKActionServer ik_action_server("IKActionServer");
  ros::spin();
  return 0;
}
