#include "ros/ros.h"
#include <iostream>
#include "demo/demo_srv.h"
#include <sstream>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv,"demo_client");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::ServiceClient client = n.serviceClient<demo::demo_srv>("demo_service");

    while(ros::ok())
    {
        demo::demo_srv srv;
        std::stringstream ss;
        ss << "sending from Here";
        srv.request.in = ss.str();
        if(client.call(srv))
        {
            ROS_INFO("fromclient[%s], server say[%s]",srv.request.in.c_str(), srv.response.out.c_str());
        }else
        {
            ROS_INFO("failed to call service");
            return 1;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}