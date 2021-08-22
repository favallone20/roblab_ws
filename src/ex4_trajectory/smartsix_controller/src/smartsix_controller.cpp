#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>

#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>

#include <signal.h>

void startTrajectoryController();
void stopTrajectoryController();
void sigintCallback(int sig);

ros::ServiceClient switch_controller_client;
ros::ServiceClient load_controller_client;
ros::ServiceClient unload_controller_client;

int main(int argc, char **argv)
{
  // Initializing the node and the move_group interface
  ros::init(argc, argv, "smartsix_controller");
  ros::NodeHandle node_handle;
  std::string name = "yz_circular";

  std::string switch_controller_srv_name = "/smartsix/controller_manager/switch_controller";
  std::string load_controller_srv_name = "/smartsix/controller_manager/load_controller";
  std::string unload_controller_srv_name = "/smartsix/controller_manager/unload_controller";
  ros::service::waitForService(switch_controller_srv_name);
  ros::service::waitForService(load_controller_srv_name);
  ros::service::waitForService(unload_controller_srv_name);
  switch_controller_client =
      node_handle.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_srv_name);
  load_controller_client = node_handle.serviceClient<controller_manager_msgs::LoadController>(load_controller_srv_name);
  unload_controller_client =
      node_handle.serviceClient<controller_manager_msgs::UnloadController>(unload_controller_srv_name);

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, sigintCallback);

  startTrajectoryController();

  std::string topic;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (!(ros::param::get("~topic_trajectory", topic)))
  {
    ROS_ERROR_NAMED("smartsix_controller demo", "Could not find parameter topic_trajectory");
  }

  // Instantiate publisher to publish the trajectory to joint trajectory controllers
  ros::Publisher trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>(topic, 1000);

  /* Setup
   * MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
   * the `JointModelGroup`.
   */
  static const std::string PLANNING_GROUP = "smartsix";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // The :planning_scene_interface:`PlanningSceneInterface`
  // class is used to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* Visualization
   * ^^^^^^^^^^^^^
   * The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
   * and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
   */
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "smartsix_controller demo", rvt::WHITE, rvt::XXLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("smartsix_controller demo", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("smartsix_controller demo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("smartsix_controller demo", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start planning
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

  // Cartesian path
  // with the cartesian path we follow this path:
  // x(λ)--> interpolation --> IK --> time parametrization and after visualization on Rviz.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0;
  const double eef_step = 0.01;  // distance between one point and another (in this case 1cm)

  std::string package_path = ros::package::getPath("smartsix_controller");

  moveit_dp_redundancy_resolution::WorkspaceTrajectory wst("smartsix", package_path + "/data/circular.traj");

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  std::string path = "/home/francesco/Projects/roblab_ws/src/ex4_trajectory/data/circular.traj";
  moveit_dp_redundancy_resolution::WorkspaceTrajectory workspace_trajectory(name, path);
  moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory workspace_trajectory_msgs;
  workspace_trajectory.getWorkspaceTrajectoryMsg(workspace_trajectory_msgs);
  std::vector<geometry_msgs::Pose> waypoints = workspace_trajectory_msgs.waypoints;

  // percentage of the path where is respected the jump_threshold
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("smartsix_controller demo", "Visualizing path (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "", rvt::SMALL);
  visual_tools.trigger();

  // Publish the trajectory towards ROS control
  trajectory_publisher.publish(trajectory.joint_trajectory);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the joint space solution");

  // Prepare display trajectory message and publish it

  ros::Publisher plot_trajectory_publisher =
      node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_trajectory", 10000, true);
  std::vector<trajectory_msgs::JointTrajectoryPoint> pointsToPlot = trajectory.joint_trajectory.points;

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plot on rqt_multiplot");

  for (int i = 0; i < pointsToPlot.size(); i++)
  {
    plot_trajectory_publisher.publish(pointsToPlot[i]);
  }
  // Show status text in RViz
  visual_tools.publishText(text_pose, "rqt_plotted", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();

  // END
  spinner.stop();
  stopTrajectoryController();
  ros::shutdown();

  return 0;
}

void startTrajectoryController()
{
  // First, we must load the trajectory controller before starting it
  ROS_INFO_NAMED("smartsix_controller demo", "Loading Trajectory controller...\n");
  controller_manager_msgs::LoadController load_controller;
  load_controller.request.name = "trajectory_controller";
  load_controller_client.call(load_controller);

  // The controllers are switched by source code in order to avoid conflicts with the execution of demo_gazebo.launch
  ROS_INFO_NAMED("smartsix_controller demo", "Stopping joint position controllers...\nStarting trajectory "
                                             "controller...\n");
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.clear();
  switch_controller.request.stop_controllers.clear();
  switch_controller.request.start_controllers.push_back("trajectory_controller");
  switch_controller.request.stop_controllers.push_back("joint1_position_controller");
  switch_controller.request.stop_controllers.push_back("joint2_position_controller");
  switch_controller.request.stop_controllers.push_back("joint3_position_controller");
  switch_controller.request.stop_controllers.push_back("joint4_position_controller");
  switch_controller.request.stop_controllers.push_back("joint5_position_controller");
  switch_controller.request.stop_controllers.push_back("joint6_position_controller");
  switch_controller.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
  switch_controller_client.call(switch_controller);
}

void stopTrajectoryController()
{
  // The controllers are switched by source code in order to avoid conflicts with the execution of demo_gazebo.launch
  ROS_INFO_NAMED("smartsix_controller demo", "Stopping trajectory controller...\nStarting joint position "
                                             "controllers...\n");
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.clear();
  switch_controller.request.stop_controllers.clear();
  switch_controller.request.start_controllers.push_back("joint1_position_controller");
  switch_controller.request.start_controllers.push_back("joint2_position_controller");
  switch_controller.request.start_controllers.push_back("joint3_position_controller");
  switch_controller.request.start_controllers.push_back("joint4_position_controller");
  switch_controller.request.start_controllers.push_back("joint5_position_controller");
  switch_controller.request.start_controllers.push_back("joint6_position_controller");
  switch_controller.request.stop_controllers.push_back("trajectory_controller");
  switch_controller.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
  switch_controller_client.call(switch_controller);

  // Unloading the trajectory controller restores the previous setting
  ROS_INFO_NAMED("smartsix_controller demo", "Unloading trajectory controller...\n");
  controller_manager_msgs::UnloadController unload_controller;
  unload_controller.request.name = "trajectory_controller";
  unload_controller_client.call(unload_controller);
}

void sigintCallback(int sig)
{
  // All the default sigint handler does is call shutdown()
  ROS_INFO_NAMED("smartsix_controller demo", "SIGINT detected!\n");
  stopTrajectoryController();
  ros::shutdown();
}
