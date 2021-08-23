# smartsix_controller

The `smartsix_controller` is the package launching the ROS node where the workspace trajectory is defined and is sent to the topic on which a selected controller is listening. In this package, the selected controller is a joint trajectory controller (<http://wiki.ros.org/joint_trajectory_controller>) from the ROS controllers family.

## Dependencies

This demo assumes that you installed ROS Melodic, that is at the following link: <http://wiki.ros.org/melodic/Installation/Ubuntu>.

This demo is based on the package moveit_dp_redundancy_resolution which permits to import a circular trajectory, defined in matlab, in our code.
https://github.com/unisa-acg/moveit_dp_redundancy_resolution/tree/master/moveit_dp_redundancy_resolution

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
Moreover, in smartsix_controller.launch is included also the code for running rqt_multiplot node (the .xml file is provided in the package folder)

You can step through the demo by using the `RVizVisualToolsGui`.
