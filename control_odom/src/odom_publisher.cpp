

#include <ros/ros.h>
#include <serial/serial.h>
#include <modbus.h>
#include <math.h>
//#include <cmath.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
 
#include <user_msgs/SickMagneticMsg.h>
#include <user_msgs/AgvStatusMsg.h>
#include <user_msgs/Odometry.h>
#include <user_srvs/AgvControlSrv.h>
#include <user_srvs/AgvControlSrvRequest.h>
#include <user_srvs/AgvControlSrvResponse.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <iomanip>


struct timeval timeout;
uint16_t regsValue[16];

nav_msgs::Odometry odom;

double gyo_z = 0;


class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        pub_ = n_.advertise<nav_msgs::Odometry>("odom", 50);


        //Topic you want to subscribe
        sub_ = n_.subscribe("/agv_status_data", 10, &SubscribeAndPublish::callback, this);
        //注意这里，和平时使用回调函数不一样了。


    }

void callback(const user_msgs::AgvStatusMsg::ConstPtr& input)
    {


        using namespace std;

        static tf::TransformBroadcaster odom_broadcaster;

        left_position1=input->position_left;
        right_position1=input->position_right;
//
        current_time = ros::Time::now();


        double dright = (right_position1 - position_right1) * M_PI * D / (16384 * 22);
        //dleft is left boot positon
        double dleft = (left_position1 - position_left1) * M_PI * D / (16384 * 22);


       ROS_INFO("Publish Position Info: left:%d  right:%d",
                left_position1 , right_position1);


        double dt = (current_time - last_time).toSec();


         ROS_INFO("Publish Position Info: time:%0.10f" ,
                  dt);

        double dxy_ave = (dright + dleft) / 2.0;

       

//    ROS_INFO("Publish Position Info: velocity:%0.5f" ,
//                 vxy);

        double dth = (dright - dleft) / wheel_track;
        th += dth;
        double theta=th/M_PI*180;
        double vth = dth / dt;
       ROS_INFO("Publish theta1 Info: theta:%0.5f" , theta);

        double vxy = dxy_ave / dt;     
        double dx = cos(dth) * dxy_ave;
        double dy = -sin(dth) * dxy_ave;
        x += (cos(th) * dx - sin(th) * dy);
        y += (sin(th) * dx + cos(th) * dy);
        ROS_INFO("Publish theta1 Info: x:%0.5f" , x);
        ROS_INFO("Publish theta1 Info: y:%0.5f" , y);

//        double dth1 = gyo_z*dt;
//        ROS_INFO("Publish gyo_z Info:gyo_z:%0.5f" , gyo_z);
//        th1 += dth1;
//        double theta1=th1/M_PI*180;
//        ROS_INFO("Publish theta Info:theta1:%0.5f" , theta1);



        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
//        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vxy;
        odom.twist.twist.linear.y = 0;
        // ROS_INFO("Publish Position Info: gyo_z:%0.4f" , gyo_z);
        odom.twist.twist.angular.z = vth;

        pub_.publish(odom);

        last_time = current_time;
        position_left1=left_position1;
        position_right1=right_position1;

    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;



    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();



    int32_t position_left1=0 ;
    int32_t position_right1=0;
    int32_t left_position1 ;
    int32_t right_position1 ;

    double wheel_track=0.445;

    double x = 0.0;
    double y = 0.0;
    double th = 0 ;
    double th1 = 0;


    double pi=3.1415926;
    double D=0.125;

};


//void imu_callback(const sensor_msgs::Imu imu_data);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");

    SubscribeAndPublish SAPObject;
    ros::NodeHandle node;

    //start asynchronous spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //stop asynchronous spinner
    

//    ros::Subscriber Imu_sub = node.subscribe("/imu_data", 10, imu_callback);


    // ros::spin();

    ros::Rate loop_rate(500);
    while (ros::ok()){
        ros::spinOnce(); 
        loop_rate.sleep();
        }

    spinner.stop();

    return 0;
}


void imu_callback(const sensor_msgs::Imu imu_data)
{
    gyo_z = imu_data.angular_velocity.z;
    // ROS_INFO("Publish Position Info: gyo_z:%0.4f" , gyo_z);
}



