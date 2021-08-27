/* -------------------------------------------------------------------
 *
 *
 * Title: fk_server.cpp
 * Author:  Francesco Avallone
 *
 * This module implements a service server to compute FK solution.
 *
 * -------------------------------------------------------------------
 */

#include <kinematics_service_msgs/GetFKSolution.h>
#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

bool computeFk(kinematics_service_msgs::GetFKSolution::Request &req,
               kinematics_service_msgs::GetFKSolution::Response &res)
{
    
    // Load the robot model
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader.getModel();

    // Create default robot state from kinematic model
    moveit::core::RobotState robot_state(kinematic_model);

    // Convert robot state message to RobotState
    moveit::core::robotStateMsgToRobotState(req.robot_state, robot_state);

    robot_state.updateLinkTransforms();

    // Get the planning group
    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("fanuc");

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    // Compute FK and prepare the response message
    Eigen::Isometry3d forward_kinematics = robot_state.getGlobalLinkTransform(link_names.back());

    tf::poseEigenToMsg(forward_kinematics, res.end_effector);

    return true;

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fk_server");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("fk_service", computeFk);
  ros::spin();
  return 0;
}
