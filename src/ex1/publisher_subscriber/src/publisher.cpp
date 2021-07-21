#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh; //The node handle is the access point for communications with the ROS system
    ros::Publisher publisher = nh.advertise<sensor_msgs::JointState>("chatter", 1);
    ros::Rate loopRate(10);

    while(ros::ok()){
        sensor_msgs:: JointState jt;
         jt.name.resize(6);   
        for (int i = 0; i < 6; i++)
        {
             jt.name[i]="joint_"+ std::to_string(i);
        }

        for (int i = 0; i < 6; i++)
        { 
            jt.position.resize(5);
            for (int j = 0; j < 5; j++)
            {
                jt.position[j] = rand();
            }
        }

        publisher.publish(jt);
        ros::spinOnce();
        loopRate.sleep();
    }
    ros::shutdown();
    return 0;
}
