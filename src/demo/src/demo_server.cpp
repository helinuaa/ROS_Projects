#include "ros/ros.h"
#include "demo/demo_srv.h"
#include <iostream>
#include <sstream>

using namespace std;

bool demo_service_callback(demo::demo_srv::Request& req,demo::demo_srv::Response& res)
{
    std::stringstream ss;
    ss<<"Received Here";
    res.out = ss.str();
    ROS_INFO("from client[%s],server says[%s]",req.in.c_str(), res.out.c_str());
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("demo_service",demo_service_callback);
    ROS_INFO("ready to recieve from client");
    ros::spin();
    return 0;
}