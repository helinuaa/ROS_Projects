#include <ros/ros.h>
#include <serial/serial.h>
#include <modbus.h>
#include <math.h>
#include <sensor_msgs/Imu.h>

#define AX	0x34
#define AY	0x35
#define AZ	0x36

#define GX	0x37
#define GY	0x38
#define GZ	0x39

#define Roll	0x3d
#define Pitch	0x3e
#define Yaw		0x3f

#define Quat		0x51



// signed short sReg[REGSIZE]={0};

modbus_t *ctx; 

struct timeval timeout;
uint16_t regsValue[16];
char ModbusReadReg(char ComPort,char Addr, unsigned short usReg, unsigned short usRegNum,short sRegs[]);



// //回调函数 
// void write_callback(const std_msgs::String::ConstPtr& msg) 
// { 
//     ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
//     ser.write(msg->data);   //发送串口数据 
// } 

int main(int argc, char **argv)
{
   	//初始化节点 
    ros::init(argc, argv, "imu_publisher_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
 
    //发布主题, 消息格式使用sensor_msg::Imu标准格式（topic名称，队列长度）
    ros::Publisher IMU_read_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1000); 

    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    modbus_set_slave(ctx, 0x50);
	timeout.tv_sec = 0;
	timeout.tv_usec = 20000;
	modbus_set_response_timeout(ctx, &timeout);
	if(modbus_connect(ctx) == -1)
	{
		modbus_free(ctx);
	}

	
	signed short ll;
    //消息发布频率
	ros::Rate loop_rate(50);
    while (ros::ok()){

    	int32_t position;
    	int16_t readRegs;
    	// signed short sReg[REGSIZE]={0};

    	readRegs = modbus_read_registers(ctx, AX, 1, regsValue );
    	double ax=(double)((int16_t)regsValue[0])/32768*16*9.8;
    	// ROS_INFO("Publish Position Info: ax:%0.4f" , ax);

    	readRegs = modbus_read_registers(ctx, AY, 1, regsValue );
    	double ay=(double)((int16_t)regsValue[0])/32768*16*9.8;
    	// ROS_INFO("Publish Position Info: ay:%0.4f" , ay);

    	readRegs = modbus_read_registers(ctx, AZ, 1, regsValue );
    	double az=(double)((int16_t)regsValue[0])/32768*16*9.8;
    	// ROS_INFO("Publish Position Info: az:%0.4f" , az);

		readRegs = modbus_read_registers(ctx, GX, 3, regsValue );
    	double gx=(double)((int16_t)regsValue[0])/32768*2000/180*M_PI;
    	// ROS_INFO("Publish Position Info: gx:%0.4f" , gx);

    	// readRegs = modbus_read_registers(ctx, AY, 1, regsValue );
    	double gy=(double)((int16_t)regsValue[1])/32768*2000/180*M_PI;
    	// ROS_INFO("Publish Position Info: gy:%0.4f" , gy);

    	// readRegs = modbus_read_registers(ctx, AZ, 1, regsValue );
    	double gz=(double)((int16_t)regsValue[2])/32768*2000/180*M_PI;
    	// ROS_INFO("Publish Position Info: gz:%0.4f" , gz);


    	readRegs = modbus_read_registers(ctx, Roll, 3, regsValue );
    	double roll=(double)((int16_t)regsValue[0])/32768*180;
    	// ROS_INFO("Publish Position Info: roll:%0.4f" , roll);

    	// readRegs = modbus_read_registers(ctx, AY, 1, regsValue );
    	double pitch=(double)((int16_t)regsValue[1])/32768*180;
    	// ROS_INFO("Publish Position Info: pitch:%0.4f" , pitch);

    	// readRegs = modbus_read_registers(ctx, AZ, 1, regsValue );
    	double yaw=(double)((int16_t)regsValue[2])/32768*180;
    	// ROS_INFO("Publish Position Info: yaw:%0.4f" , yaw);


    	readRegs = modbus_read_registers(ctx, Quat, 4, regsValue );
    	double o1=(double)((int16_t)regsValue[0])/32768;
    	//ROS_INFO("Publish Position Info: o1:%0.4f" , o1);

    	//readRegs = modbus_read_registers(ctx, Quat, 4, regsValue );
    	double o2=(double)((int16_t)regsValue[1])/32768;
    	//ROS_INFO("Publish Position Info: o2:%0.4f" , o2);

    	//readRegs = modbus_read_registers(ctx, Quat, 4, regsValue );
    	double o3=(double)((int16_t)regsValue[2])/32768;
    	//ROS_INFO("Publish Position Info: o3:%0.4f" , o3);

    	//readRegs = modbus_read_registers(ctx, Quat, 4, regsValue );
    	double o4=(double)((int16_t)regsValue[3])/32768;



           //打包IMU数据
            sensor_msgs::Imu imu_data;
            //tf::Quaternion quat;
            
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";
            imu_data.orientation.x = o2;
            imu_data.orientation.y = o3;
            imu_data.orientation.z = o4;
            imu_data.orientation.w = o1;
       
           imu_data.angular_velocity.x = gx;
           imu_data.angular_velocity.y = gy;
           imu_data.angular_velocity.z = gz;
 
           imu_data.linear_acceleration.x = ax;
           imu_data.linear_acceleration.y = ay;
           imu_data.linear_acceleration.z = az;
           
 
           //发布topic
           IMU_read_pub.publish(imu_data);

    	ros::spinOnce(); 
        loop_rate.sleep();
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数 
         
  
  	return 0;
 }
