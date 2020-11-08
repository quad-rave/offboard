#include "ros/ros.h"
#include "std_msgs/String.h"

void staterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Iris Drone at state %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    ROS_INFO("Welcome to State Observer. . . ");
    ros::Subscriber sub;


    sub = nh.subscribe("state_publisher", 1000, staterCallback);
    ros::spin();

    
    return 0;
}