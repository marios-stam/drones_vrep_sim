#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    //euler from quaternion
    tf::Quaternion q;
    
    ros::NodeHandle node;

  
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/world", "/drone_0",
                               ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        tf::Vector3 pos = transform.getOrigin();
        ROS_INFO("x: %f, y: %f, z: %f", pos.x(), pos.y(), pos.z());

        rate.sleep();
    }
  return 0;
};