/* -------------------------------------------------------------------
 *
 * This module has been developed for the Control Systems Design
 * class of the University of Salerno, Italy.
 *
 * Title:   smartsix_controller.cpp
 * Author:  Giuseppe Cirillo, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    June 20, 2020
 *
 * This file implements a ROS node to control the Smart-Six robot by 
 * Comau, whose configuration is included in the ROS module 
 * smartsix_moveit_config. It follows the model of the 
 * move_group_interface_tutorial.
 * 
 * -------------------------------------------------------------------
 * 
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>

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

    std::string switch_controller_srv_name = "/smartsix/controller_manager/switch_controller";
    std::string load_controller_srv_name = "/smartsix/controller_manager/load_controller";
    std::string unload_controller_srv_name = "/smartsix/controller_manager/unload_controller";
    ros::service::waitForService(switch_controller_srv_name);
    ros::service::waitForService(load_controller_srv_name);
    ros::service::waitForService(unload_controller_srv_name);
    switch_controller_client = node_handle.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_srv_name);
    load_controller_client = node_handle.serviceClient<controller_manager_msgs::LoadController>(load_controller_srv_name);
    unload_controller_client = node_handle.serviceClient<controller_manager_msgs::UnloadController>(unload_controller_srv_name);
    
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

    //Planning to pose1 goal 
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = 1;
    target_pose1.position.z = 1;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "Pose 1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XXLARGE);

    // Trajectory lines are generated for the end-effector
    // If the planning group does not have an end-effector, 
    // we need to specify the reference frame for which to compute inverse kinematics
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link6"), joint_model_group,rvt::LIME_GREEN);
    
    visual_tools.trigger();

    // Publish the trajectory towards ROS control
    trajectory_publisher.publish(my_plan.trajectory_.joint_trajectory);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
   
    /* Cartesian Path1
     * ^^^^^^^^^^^^^^^
     * We plan a Cartesian path by giving some waypoints,
     * then ask the planner to interpolate between them to have a higher number of waypoints that we can use for planning.
     * In particular, we give one waypoint.
     */
    geometry_msgs::Pose target_pose2 = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    target_pose2.position.z += 0.15;
    target_pose2.position.x += 0.21;
    waypoints.push_back(target_pose2);//up and forward

    // This pose corresponds to coming back to the starting position
    target_pose2.position.z -= 0.15;
    target_pose2.position.x -= 0.21;
    waypoints.push_back(target_pose2);//down and back
    
    // message to specify trajectories to joint_trajectory_controller
    moveit_msgs::RobotTrajectory trajectory;

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing path (%.2f%% achieved)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian path 1", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
    visual_tools.trigger();

    // Publish the trajectory towards ROS control
    trajectory_publisher.publish(trajectory.joint_trajectory);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Cartesian Path2
     * ^^^^^^^^^^^^^^^
     * We plan a Cartesian path by giving some waypoints,
     * then ask the planner to interpolate between them to have a higher number of waypoints that we can use for planning.
     * In particular, we give one waypoint. 
     */
    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
   
    target_pose3.position.z += -0.55;
    target_pose3.position.x += 0.215;
    waypoints.push_back(target_pose3); //down and forward

    // This pose corresponds to coming back to the starting position
    target_pose3.position.z -= -0.55;
    target_pose3.position.x -= 0.215;
    waypoints.push_back(target_pose3); //up and back

    //We want the Cartesian path to be interpolated at a resolution of 1 cm 
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing path (%.2f%% achieved)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian path 1 + Cartesian path 2", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);

    visual_tools.trigger();

    // Publish the trajectory towards ROS control
    trajectory_publisher.publish(trajectory.joint_trajectory);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add a collision object");

    // Adding  Object
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";
    double height_surface = 0.50; //meters
    double width_surface = 0.50;
    double lenght_surface = 0.50;

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = height_surface;
    primitive.dimensions[1] = width_surface;
    primitive.dimensions[2] = lenght_surface;

    // Define a pose for the box.
    //box1's position 
    geometry_msgs::Pose box_pose;
    //box1's orientation
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.40;
    box_pose.position.y = -0.80;
    box_pose.position.z = 0.90;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Add the collision object into the world
    ROS_INFO_NAMED("smartsix_controller demo", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show status text in RViz 
    visual_tools.publishText(text_pose, "Added object", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

    // Now, when we plan a trajectory, it will avoid the obstacle
    geometry_msgs::Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = -0.5;
    another_pose.position.y = -0.6;
    another_pose.position.z = 0.9;
    move_group.setPoseTarget(another_pose);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing plan 5 (pose goal, move around cuboid) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(another_pose, "Goal pose");
    visual_tools.publishText(text_pose, "Goal with obstacle", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link6"), joint_model_group,rvt::LIME_GREEN);
    visual_tools.trigger();

    // Publish the trajectory towards ROS control
    trajectory_publisher.publish(my_plan.trajectory_.joint_trajectory);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove the collision object");

    // Now, let's remove the collision object from the world.
    ROS_INFO_NAMED("smartsix_controller demo", "Removing the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show status text in RViz
    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to close the demo");

    // Show status text in RViz
    visual_tools.publishText(text_pose, "Demo terminated", rvt::WHITE, rvt::XXLARGE);
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
    ROS_INFO_NAMED("smartsix_controller demo", "Stopping joint position controllers...\nStarting trajectory controller...\n");    
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
    ROS_INFO_NAMED("smartsix_controller demo", "Stopping trajectory controller...\nStarting joint position controllers...\n");
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
