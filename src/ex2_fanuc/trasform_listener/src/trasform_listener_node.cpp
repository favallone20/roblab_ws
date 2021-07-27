#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void printTrasformedStamped(tf2_ros::Buffer &tf_buffer, std::string target, std::string source){
    geometry_msgs::TransformStamped transformed_stamped;
    transformed_stamped = tf_buffer.lookupTransform(target, source, ros::Time(0));
    tf2::Quaternion quaternion;
    tf2::fromMsg(transformed_stamped.transform.rotation, quaternion);
    std::ostringstream ss;

    ss << std::endl << "------- Translation -------" << std::endl;
    ss << transformed_stamped.transform.translation << std::endl;

    tf2::Vector3 rotation_axis = quaternion.getAxis();
    ss << "------- Axis/angle -------" << std::endl;
    ss << "Axis = [" << rotation_axis.getX() << ", " << rotation_axis.getY() << ", " << rotation_axis.getZ() << "]" << std::endl;
    ss << "Angle = " << quaternion.getAngle() << std::endl;

    tf2::Matrix3x3 matrix(quaternion);
    ss << std::endl << "------- Rotation matrix -------" << std::endl;
    ss << "[ " << matrix[0][0] << ", " << matrix[0][1] << ", " << matrix[0][2] << " ]" << std::endl;
    ss << "[ " << matrix[1][0] << ", " << matrix[1][1] << ", " << matrix[1][2] << " ]" << std::endl;
    ss << "[ " << matrix[2][0] << ", " << matrix[2][1] << ", " << matrix[2][2] << " ]" << std::endl;

    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    ss << std::endl << "------- Euler angles (RPY) -------" << std::endl;
            ss << "[ " << roll << ", " << pitch << ", " << yaw << " ]" << std::endl;

    ROS_INFO_STREAM(ss.str());

    ROS_INFO_STREAM("-------------------------------------------");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener_tf");
    ros::NodeHandle nh;
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Rate rate(10);
    //21
    int num_link = 8; 
    std::string link_name[num_link];
    link_name[0] = "base_link"; link_name[1] = "link1"; link_name[2] = "link2";
    link_name[3] = "link3"; link_name[4] = "link4"; link_name[5] = "link5";
    link_name[6] = "link6"; link_name[7] = "flange";


    while(ros::ok()){
        try{
            for(int i= 0; i< num_link-1; i++){
                printTrasformedStamped(tf_buffer, link_name[i], link_name[7]);
            }  
        }
        catch (tf2::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            ros::Duration(1.0).sleep();
            continue;   
        }
        rate.sleep();
    }
    return 0;
}
