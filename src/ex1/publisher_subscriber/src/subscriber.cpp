
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void jointStatesCallback(const sensor_msgs::JointState jointState)
{
  ROS_INFO_STREAM("message received: " << jointState);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber subscriber = nh.subscribe("joint_states_ex1", 10, jointStatesCallback);
  ros::spin();
  ros::shutdown();
  return 0;
}