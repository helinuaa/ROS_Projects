#include "ros/ros.h"
#include "std_msgs/Int32.h"
//#include "std_msgs/String.h"
#include <iostream>
#include "demo/demo_msg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_publisher");
    ros::NodeHandle node_obj;
    ros::Publisher number_publisher = node_obj.advertise<demo::demo_msg>("demo_msg",10);
    ros::Rate loop_rate(10);
    int number_count = 0;
    while(ros::ok())
    {
        demo::demo_msg msg_test;
        msg_test.greeting = "helloword!";
        msg_test.number = number_count;

        ROS_INFO("%s: %d",msg_test.greeting.c_str(),msg_test.number);
        number_publisher.publish(msg_test);
        ros::spinOnce();
        loop_rate.sleep();
        ++number_count;

    }
    return 0;
}