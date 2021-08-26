# transform_listener

This package implements an example of trasform listener to compute orientation and position of fanuc robot's end effector in all reference frames.

## Run the Example
### Setup 
An example launch file trasform_listener.launch is provided which launchs three nodes:

roslaunch trasform_listener trasform_listener.launch 

* rosrun trasform_listener trasform_listener
* roslaunch fanuc_moveit_config demo.launch
* rosrun joint_state_publisher_gui joint_state_publisher_gui

The user can use joint_state_publisher_gui node to see orientation and position changes of the end effector.