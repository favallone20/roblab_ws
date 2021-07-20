1) Setup the Husky simulation

2) Launch the simulation and inspect the created nodes and their topics using
	rosnode list
	rostopic list
	rostopic echo [TOPIC]
	rostopic hz [TOPIC]
	rqt_graph

3)Command a desired velocity to the robot from the terminal (rostopic pub [TOPIC])

example: rostopic pub -r 1  /joy_teleop/cmd_vel geometry_msgs/Twist "linear:
  x: -10.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 


4)Use ​teleop_twist_keyboard ​to control your robot using the keyboard. Find it online 
and compile it from source! Use ​git clone​ to clone the repository to the folder 
~/git.

command used:rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
