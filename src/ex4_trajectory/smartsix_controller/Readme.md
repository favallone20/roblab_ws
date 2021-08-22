# smartsix_controller

The `smartsix_controller` is the package launching the ROS node where the workspace trajectory is defined and is sent to the topic on which a selected controller is listening. In this package, the selected controller is a joint trajectory controller (<http://wiki.ros.org/joint_trajectory_controller>) from the ROS controllers family.

The `smartsix_controller` is the MoveIt! controller that plans trajectories according to certain criteria, i.e., planning to a pose in task space and avoid obstacles, which are shown in `smartsix_controller.cpp`. The MoveIt! controller must not be confused with a ros-control controller (<http://wiki.ros.org/ros_control#Controllers>) that really executes the trajectory. So, in short, the MoveIt! controller is a trajectory producer, while the ros-control controller is a trajectory consumer.

## Dependencies

This demo assumes that you installed ROS Melodic, that is at the following link: <http://wiki.ros.org/melodic/Installation/Ubuntu>.

Other dependencies are listed in `package.xml` and can be installed with `rosdep`:

```bash
cd <your_workspace_folder>/src
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
```

## Run the demo

Start the simulation of the Comau Smart-Six robot:

```bash
roslaunch smartsix_moveit_config demo_gazebo.launch
```

This launches both RViz and Gazebo and loads on the Parameter Server the parameters needed to control both views.

Execute the `smartsix_controller` node through its launch file and replace the ros-control controllers with those needed to execute off-line trajectories:

```bash
roslaunch smartsix_controller smartsix_controller.launch
```

You can step through the demo by using the `RVizVisualToolsGui`.

## Remarks

* Do not use the `setMaxVelocityScalingFactor` function to scale down the trajectory velocity when using the `computeCartesianPath` service, it is not exactly a planner and, thus, does not use such scaling factors. Using it will not affect the final result. Look at [this thread](https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/) for additional info.

## Known issues

* The following error might show up in the RViz/Gazebo console:

```
    [ERROR] [1593531122.642604224]: Could not find the planner configuration 'None' on the param server.
```

This occurs because `smartsix_moveit_config` is configured without a group planner, however one is loaded by default (RRT-Connect).

* When planning the point-to-point trajectory that avoids the obstacle, the planner might return a path that collides with the floor. This is expected and it is actually nominal if the floor is not defined as an obstacle in the planning scene. In Gazebo, the floor represents a collision object, thus trajectories that require the robot to collide with the floor will break the robot.
