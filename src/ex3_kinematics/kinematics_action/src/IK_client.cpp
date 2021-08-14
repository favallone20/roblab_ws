#include <actionlib/client/simple_action_client.h>
#include <kinematics_action_msgs/GetIKSolutionsAction.h>
#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2_eigen/tf2_eigen.h>

typedef actionlib::SimpleActionClient<kinematics_action_msgs::GetIKSolutionsAction> Client;


void printNewPoses(robot_state::RobotState robot_state)
{
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
}

void doneIK_CB(const actionlib::SimpleClientGoalState& state, const kinematics_action_msgs::GetIKSolutionsResultConstPtr& result)
{
    ROS_INFO_STREAM("Solutions: "<<"\n");
    for(int i=0; i<result->ik_solutions.size();i++){
        ROS_INFO_STREAM(result->ik_solutions[i]<<" \n");
    }
}

void activeServer_CB()
{
  ROS_INFO_STREAM("Goal just went active \n");
}

void feedback_CB(const kinematics_action_msgs::GetIKSolutionsFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("Feedback: "<<feedback->ik_solution<<" \n");
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "IK_client");
  Client aclient("IK_client", true);
  kinematics_action_msgs::GetIKSolutionsGoal goal;

  aclient.waitForServer();

  robot_model_loader::RobotModelLoader robot_model_loader("fanuc_description");
  robot_model::RobotModelConstPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotState robot_state(kinematic_model);
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("fanuc");

  const Eigen::Affine3d& end_effector_state = robot_state.getGlobalLinkTransform("flange");
  geometry_msgs::Pose pose = tf2::toMsg(end_effector_state);
  ROS_INFO_STREAM("Pose end_effector:" << pose<<  "\n");

  robot_state.setToRandomPositions();
  printNewPoses(robot_state);

  robot_state::robotStateToRobotStateMsg(robot_state, goal.robot_state);

  goal.pose_end_effector_goal = pose;

  aclient.sendGoal(goal, &doneIK_CB, &activeServer_CB, &feedback_CB);
  aclient.waitForResult();

  return 0;
}
