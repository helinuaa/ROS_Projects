#include"ros/ros.h"
#include "std_msgs/Int32.h"
//#include "std_msgs/String.h"
#include <iostream>
#include "demo/demo_msg.h"

void number_callback(const demo::demo_msg& msg)
{
    ROS_INFO("Received %s: %d",msg.greeting.c_str(), msg.number);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_listenner");
    ros::NodeHandle node_obj;
    ros::Subscriber number_subscriber = node_obj.subscribe("demo_msg",10,number_callback);
    ros::spin();
    return 0;
}