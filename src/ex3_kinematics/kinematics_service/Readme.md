# kinematics_service
This package implements an example of computation of inverse kinematics with ros service

## Setup
An example launch file fk_service.launch is provided which launches three nodes:

roslaunch kinematics_service fk_service.launch

* roslaunch fanuc_moveit_config demo.launch
* rosrun kinematics_service fk_server_node
* rosrun kinematics_service fk_client_node
