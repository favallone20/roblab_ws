# kinematics_action
This package implements an example of computation of inverse kinematics with ros action

## Setup
An example launch file IKAction.launch is provided which launches three nodes:

roslaunch kinematics_action IKAction.launch

* roslaunch fanuc_moveit_config demo.launch
* rosrun kinematics_action IK_server_node
* rosrun kinematics_action IK_client_node

You can step through the demo by using the `RVizVisualToolsGui`.
