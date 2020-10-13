/***** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******source file****
* File name          : agv_control_node.cpp
* Author             : Jinliang Yang
* Brief              : agv control of node
********************************************************************************
* modify
* Version   Date                Author          	Described
* V1.00     2020/05/12          Jinliang Yang       Created
*******************************************************************************/
#include <ros/ros.h>  
#include <serial/serial.h>
#include <modbus.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <user_msgs/SickMagneticMsg.h>
#include <user_msgs/AgvStatusMsg.h>
#include <user_srvs/AgvControlSrv.h>
#include <user_srvs/AgvControlSrvRequest.h>
#include <user_srvs/AgvControlSrvResponse.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <iomanip>




enum{
	CTRL_ERROR_CODE1 = 200, //驱动器报警信息
	CTRL_CLEAR_ERROR_FLAG = 52, //清除驱动器报警信息,1清除自动恢复为0。
	CTRL_OPERATING_MODE = 36, //操作模式 0:网络模式 1：脉冲/方向 2：模拟信号 3：PWM操作模式
	CTRL_CONTROL_MODE = 37, //0:电流控制模式 1：速度控制模式 2：位置控制模式
	CTRL_ENABLE_FLAG = 40,//0:电机释放 1：电机使能
	CTRL_QUICK_SOFT_STOP = 42,//急停停止方式  0：立即停止  1：减速停止  2：释放电机
	CTRL_EMERGENCY_STOP = 44, //急停命令 1：电机执行急停命令，此后自动恢复到0
	CTRL_POS_ACTUAL = 172, //位置实际值
	CTRL_VEL_SET_ACTUAL = 174,
	CTRL_VEL_ACTUAL = 175, //速度实际值
	CTRL_ENCODER_RESET_FLAG =51 //设置为 1 编码器清零,此后自动恢复到 0
}MotorRegMap;

class MotorControl{
   	typedef void (MotorControl::*pFunStateMachine)(int32_t value);
	typedef void (MotorControl::*pFunControlMode)(void);

public:
	MotorControl();
	~MotorControl();



	void motorInit(void);
	void otherTask(void);
	void agvNavigationControl(void);
	void agvHandleContol(void);
 	void agvRemoteControl(void);

 	void stateROS_SHUT_DOWN(int32_t value);
 	void stateROS_PARKING(int32_t value);
 	void stateROS_NORMAL_CONTROL(int32_t value);
 	void stateROS_TASKING(int32_t value);
 	void stateROS_ROTATION(int32_t value);
 	void enterROS_SHUT_DOWN(void);
 	void enterROS_PARKING(uint16_t distance);
 	void enterROS_NORMAL_CONTROL(void);
 	void enterROS_TASKING(void);
 	void enterROS_ROTATION(void);

    ros::Publisher  pubStatus;
    modbus_t *ctx;

	enum{
		ROS_NORMAL = 0,
		ROS_TASKING = 1,
		ROS_SHUT_DOWN = 2,
		ROS_TURN_LEFT = 3,
		ROS_TURN_RIGHT = 4,
		ROS_GO_AHEAD = 7,
		ROS_GO_BACK = 8,
		ROS_PARKING = 9,
		ROS_ROTATION = 10,
		ROS_PARKING_ROTATION = 11
	};

	enum{
		SICK_NO_LINE = 0,
		SICK_ONE_LINE = 2,
		SICK_TWO_LINE_RIGHT = 3,
		SICK_TWO_LINE_LEFT = 6,
		SICK_THREE_LINE = 7
	};

	pFunControlMode agvControlMode = NULL;
	pFunStateMachine agvStateMachine = NULL;


private:
	bool agvControlSrvCallback(user_srvs::AgvControlSrv::Request &req, user_srvs::AgvControlSrv::Response &res);
	void sickMagneticDataCallback(const user_msgs::SickMagneticMsg::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void cmdCallback(const geometry_msgs::Twist& twist);

	void speedOutIgnoreMagnetic(int16_t left, int16_t right);
	void speedOut(int16_t left, int16_t right);
	int16_t turnPositionPD(float bias);
	int16_t distancePositionPD(float bias);

	float sickMagneticBias;
	uint8_t sickMagneticLines;
	float carLinear, carAngular;
	uint8_t carCmd;
	int16_t joyLinear, joyAngular;
	bool joyLock = true;
	int32_t userData;

	uint32_t targetPosition;
	uint16_t maxParkingLinear;


	ros::NodeHandle node;
	ros::Subscriber subMagnetic;
	ros::Subscriber subJoy;
    ros::Subscriber subCmd;

	ros::ServiceServer serverControl;

	struct timeval timeout;
	uint16_t regsValue[16];
};

MotorControl::MotorControl()
{
	subMagnetic = node.subscribe("/sick_magnetic_data", 10, &MotorControl::sickMagneticDataCallback, this);
	subJoy = node.subscribe("/joy", 10, &MotorControl::joyCallback, this);
	pubStatus = node.advertise<user_msgs::AgvStatusMsg>("/agv_status_data", 10);

	subCmd = node.subscribe("/cmd_vel", 10, &MotorControl::cmdCallback, this);



	serverControl = node.advertiseService("/agvControlService", &MotorControl::agvControlSrvCallback, this);

	motorInit();

	agvControlMode = &MotorControl::agvHandleContol;
	agvStateMachine = &MotorControl::stateROS_NORMAL_CONTROL;

	carLinear = 1800;
}

MotorControl::~MotorControl()
    {
	stateROS_SHUT_DOWN(0);
	modbus_free(ctx);
    }

bool MotorControl::agvControlSrvCallback(user_srvs::AgvControlSrv::Request &req, user_srvs::AgvControlSrv::Response &res)
{
	switch(req.cmd){
		case ROS_PARKING:
			agvStateMachine = &MotorControl::stateROS_PARKING;
			break;
		case ROS_PARKING_ROTATION:
			break;
		case ROS_SHUT_DOWN:
			agvStateMachine = &MotorControl::stateROS_SHUT_DOWN;
			break;
		case ROS_TASKING:
			agvStateMachine = &MotorControl::stateROS_PARKING;
			break;
		case ROS_TURN_RIGHT:
			break;
		case ROS_GO_AHEAD:
			break;
		case ROS_ROTATION:
			agvStateMachine = &MotorControl::stateROS_ROTATION;
			break;
		case ROS_NORMAL:
			agvStateMachine = &MotorControl::stateROS_NORMAL_CONTROL;
			break;
	}

	return true;
}

void MotorControl::sickMagneticDataCallback(const user_msgs::SickMagneticMsg::ConstPtr &msg)
{
	sickMagneticLines = msg->lines;

	switch(msg->lines){
	case SICK_NO_LINE:
		sickMagneticBias = 0;
		break;
	case SICK_ONE_LINE:
		sickMagneticBias = msg->line2_value;
		break;
	case SICK_TWO_LINE_LEFT://6
		if (carCmd == ROS_TURN_LEFT)
			sickMagneticBias = msg->line1_value;
		else
			sickMagneticBias = msg->line2_value;
		break;
	case SICK_TWO_LINE_RIGHT://3
		if (carCmd == ROS_TURN_RIGHT)
			sickMagneticBias = msg->line3_value;
		else
			sickMagneticBias = msg->line2_value;
		break;
	case SICK_THREE_LINE://7
		if (carCmd == ROS_TURN_LEFT)
			sickMagneticBias = msg->line3_value;
		else if (carCmd == ROS_TURN_RIGHT)
			sickMagneticBias = msg->line1_value;
		else
		 	sickMagneticBias = msg->line2_value;
		break;
	}
}

void MotorControl::cmdCallback(const geometry_msgs::Twist& twist)
{

    int16_t left, right;
    double wheel_track=0.451 ;
    double D = 0.125;
    static double linear_x = 0.0;
    static double angular_z = 0.0;

    // if(((linear_x < 0.0) && (twist.linear.x > 0.0))||((linear_x > 0.0) && (twist.linear.x < 0.0)))
    //    linear_x = 0.0;
    // else
    //    linear_x = twist.linear.x;

    // if(((angular_z < 0.0) && (twist.angular.z > 0.0))||((angular_z > 0.0) && (twist.angular.z < 0.0)))
    //    angular_z = 0.0;
    // else
    //    angular_z = twist.angular.z;

    linear_x = twist.linear.x;
	angular_z = twist.angular.z;
    double vel_left = (linear_x - angular_z * wheel_track / 2)*0.5;
    double vel_right = (linear_x + angular_z * wheel_track / 2)*0.5;

    left = vel_left*60*22/M_PI/D;
    right = vel_right*60*22/M_PI/D;


	//ROS_INFO("Publish left Info:left:%d" , left);
   // ROS_INFO("Publish right Info:right:%d" , right);

    speedOutIgnoreMagnetic(left, right);


}

void MotorControl::joyCallback(const sensor_msgs::Joy::ConstPtr &joyMsg)
{
	static uint8_t key_press = 0;     //解锁按键按下检测
	static uint8_t key_unpress = 0;   //解锁按键按下后，放开检测
	if(joyMsg->axes[4]<0 && joyMsg->axes[5]<0)
        key_press = 1;
	else if(key_press)
		key_unpress =1;

    if(key_press && key_unpress)
	{
		joyLock = !joyLock;
        key_press = 0;
		key_unpress = 0;
	}
		
	if(joyLock){
		agvControlMode = &MotorControl::agvNavigationControl;
		joyLinear = 0;
		joyAngular = 0;
	} else {
		agvControlMode = &MotorControl::agvHandleContol;
		joyLinear = 2400 * joyMsg->axes[3];
		joyAngular = 600 * joyMsg->axes[0];
	}
}
// void MotorControl::joyCallback(const sensor_msgs::Joy::ConstPtr &joyMsg)
// {
// 	static uint8_t modeSwitch = 0;
// 	if(joyMsg->buttons[4] && joyMsg->buttons[5])
// 		joyLock = !joyLock;

// 	if(joyMsg->buttons[2])
// 	{
// 		if(++modeSwitch == 2)
// 			modeSwitch = 0;
// 		switch(modeSwitch){
// 		case 0:
// 			agvControlMode = &MotorControl::agvHandleContol;
// 			break;
// 		case 1:
// 			agvControlMode = &MotorControl::agvNavigationControl;
// 			break;
// 		default:
// 			modeSwitch = 0;
// 			break;
// 		}
// 	}

// 	if(joyLock){
// 		joyLinear = 0;
// 		joyAngular = 0;
// 	} else {
// 		joyLinear = 2400 * joyMsg->axes[1];
// 		joyAngular = 600 * joyMsg->axes[2];
// 	}
// }

void MotorControl::speedOutIgnoreMagnetic(int16_t left, int16_t right)
{
	int res;

	if(left > 3000) 	left = 3000;
	if(left < -3000) 	left = -3000;
	if(right > 3000) 	right = 3000;
	if(right < -3000)   right = -3000;

	modbus_set_slave(ctx, 1);
	res =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, -left);	
	modbus_set_slave(ctx, 2);
	res =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, right);	
}

void MotorControl::speedOut(int16_t left, int16_t right)
{
	if(left > 3000) 	left = 3000;
	if(left < -3000) 	left = -3000;
	if(right > 3000) 	right = 3000;
	if(right < -3000)   right = -3000;

	if(sickMagneticLines == SICK_NO_LINE)
	{
		modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
		modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);	
	} else {
		modbus_set_slave(ctx, 1);
		modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, -left);	
		modbus_set_slave(ctx, 2);
		modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, right);		
	}
}

float turnP = 63;
float turnD = 10;
int16_t MotorControl::turnPositionPD(float bias)
{
	static float lastBias = 0;
	int16_t velocity;

	velocity = turnP * bias + turnD *(bias - lastBias);
	lastBias = bias;

	return velocity;
}

float diatanceP = 0.006;
float distanceD = 0.002;
int16_t MotorControl::distancePositionPD(float bias)
{
	static float lastBias = 0;
	int16_t velocity;

	velocity = diatanceP * bias + distanceD *(bias - lastBias);
	lastBias = bias;

	return velocity;
}

void MotorControl::motorInit(void)
{
	ctx = modbus_new_rtu("/dev/ttyS3", 115200, 'N', 8, 1);
	timeout.tv_sec = 0;
	timeout.tv_usec = 5000;
	modbus_set_response_timeout(ctx, &timeout);
	if(modbus_connect(ctx) == -1)
	{
		modbus_free(ctx);
	}

	int readRegs;
	modbus_set_slave(ctx, 1);
	readRegs = modbus_read_registers(ctx, CTRL_ERROR_CODE1, 1, regsValue );
    ROS_INFO("Motor Error:%d %d",readRegs, regsValue[0]); 
	modbus_set_slave(ctx, 2);
	readRegs = modbus_read_registers(ctx, CTRL_ERROR_CODE1, 1, regsValue );
	ROS_INFO("Motor Error:%d %d",readRegs, regsValue[0]); 

	//清除驱动器报警信息
	modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
	readRegs =  modbus_write_register(ctx, CTRL_CLEAR_ERROR_FLAG, 1);
	readRegs =  modbus_write_register(ctx, CTRL_OPERATING_MODE, 0);
	readRegs =  modbus_write_register(ctx, CTRL_CONTROL_MODE, 1);
	readRegs =  modbus_write_register(ctx, CTRL_ENABLE_FLAG, 1);
	readRegs =  modbus_write_register(ctx, CTRL_ENCODER_RESET_FLAG, 1);

	modbus_set_slave(ctx, 1);
	readRegs =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);
	modbus_set_slave(ctx, 2);
	readRegs =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);

	ros::Duration(1).sleep();

	modbus_set_slave(ctx, 1);
	readRegs =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);
	modbus_set_slave(ctx, 2);
	readRegs =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);

	ros::Duration(1).sleep();

	modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
	readRegs =  modbus_write_register(ctx, CTRL_VEL_SET_ACTUAL, 0);	
}

void MotorControl::otherTask(void)
{

	static uint16_t delayCount = 0 ;
		int32_t position_left ;
		int32_t position_right;
	using namespace std;

	//1000ms task
	if(delayCount == 4)
	{
		//subVoicePub.publish(std_msgs.msg.String("巡检中，请注意！"))
		uint8_t readRegs;
		modbus_set_slave(ctx, 1);
		readRegs = modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
		position_left = abs((int32_t)regsValue[0]<<16 | regsValue[1]);

		//ROS_INFO("---------------:%d %d",readRegs, abs((int16_t)regsValue[0]));
		modbus_set_slave(ctx, 2);
		readRegs = modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
		position_right= abs((int32_t)regsValue[0]<<16 |regsValue[1]);
		//ROS_INFO("---------------:%d %d",readRegs, abs((int16_t)regsValue[0]));

//		readRegs = modbus_read_registers(ctx, CTRL_VEL_ACTUAL, 1, regsValue );

//                cout << position_left << '\t' << position_right << endl;



		delayCount =0;
	}

	else
		delayCount +=1;


}

void MotorControl::agvNavigationControl(void)
{
//	(this->*agvStateMachine)(this->userData);
}

void MotorControl::agvHandleContol(void)
{
	int16_t left, right;

	left = joyLinear - joyAngular;
	right = joyLinear + joyAngular;

	speedOutIgnoreMagnetic(left, right);
}



//void MotorControl::agvcmdvelContol(void)
//{
////    double wheel_track=0.392 ;
////
////    double left = twist.linear.x - twist.angular.z * robot_track / 2;
////    double right = twist.linear.x + twist.angular.z * robot_track / 2;
////
////    speedOutIgnoreMagnetic(left, right);
//}

void MotorControl::agvRemoteControl(void)
{

}

void MotorControl::stateROS_SHUT_DOWN(int32_t value)
{
	ROS_INFO("stateROS_SHUT_DOWN"); 
	modbus_set_slave(ctx, 1);
	modbus_write_register(ctx, CTRL_EMERGENCY_STOP, 1);	
	modbus_set_slave(ctx, 2);
	modbus_write_register(ctx, CTRL_EMERGENCY_STOP, 1);	
}

void MotorControl::enterROS_PARKING(uint16_t distance)
{
	//读取编码器
	uint64_t position;
	modbus_set_slave(ctx, 1);
	modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
	position = abs((int16_t)regsValue[0])<<16 | abs((int16_t)regsValue[1]);
	//ROS_INFO("---------------:%d %d",readRegs, abs((int16_t)regsValue[0])); 
	modbus_set_slave(ctx, 2);
	modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
	position += abs((int16_t)regsValue[0])<<16 | abs((int16_t)regsValue[1]);
	position>>1;

	targetPosition = distance + position;

	//读取两轮当前速度取均值
	uint8_t readRegs;
	modbus_set_slave(ctx, 1);
	readRegs = modbus_read_registers(ctx, CTRL_VEL_ACTUAL, 1, regsValue );
	maxParkingLinear = abs((int16_t)regsValue[0]);
	//ROS_INFO("---------------:%d %d",readRegs, abs((int16_t)regsValue[0])); 
	modbus_set_slave(ctx, 2);
	readRegs = modbus_read_registers(ctx, CTRL_VEL_ACTUAL, 1, regsValue );
	maxParkingLinear += abs((int16_t)regsValue[0]);
	maxParkingLinear>>1;
	//print(self.parking_max_linear )
}

void MotorControl::stateROS_PARKING(int32_t value)
{
	int16_t left, right;
	int32_t positionBias;
	uint64_t currentPosition;
	//读取编码器

	modbus_set_slave(ctx, 1);
	modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
	currentPosition = abs((int16_t)regsValue[0])<<16 | abs((int16_t)regsValue[1]);
	//ROS_INFO("---------------:%d %d",readRegs, abs((int16_t)regsValue[0])); 
	modbus_set_slave(ctx, 2);
	modbus_read_registers(ctx, CTRL_POS_ACTUAL, 2, regsValue );
	currentPosition += abs((int16_t)regsValue[0])<<16 | abs((int16_t)regsValue[1]);
	currentPosition>>1;

	//计算偏差
	positionBias = targetPosition - currentPosition;
	//转换为线速度和角速度
	carLinear = distancePositionPD(positionBias);
	if(carLinear > maxParkingLinear)
	 	carLinear = maxParkingLinear;
	carAngular = turnPositionPD(sickMagneticBias);
	//转换为左右轮速度
	left = carLinear - carAngular;
	right = carLinear + carAngular;
	//给电机发送速度控制指令
	speedOut(left, right);
}

void MotorControl::stateROS_NORMAL_CONTROL(int32_t value)
{
	int16_t left, right;

	carAngular = turnPositionPD(sickMagneticBias);
	left = carLinear - carAngular;
	right = carLinear + carAngular;

	speedOut(left, right);
}

void MotorControl::stateROS_TASKING(int32_t value)
{

}

void MotorControl::stateROS_ROTATION(int32_t value)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agv_control_node");

	MotorControl control;

	ros::Rate loop_rate(50);

    uint16_t regsValue[16];

    static uint16_t delayCount = 0 ;
    int32_t position_left ;
    int32_t position_left1 ;
    int32_t position_right;
	int32_t position_left_last = 0 ;
    int32_t position_right_last = 0;
    using namespace std;


    user_msgs::AgvStatusMsg agv_status_data;
    int count = 0;

	while(ros::ok())
	{
		(control.*control.agvControlMode)();

		//control.otherTask();

        int8_t readRegs;
        modbus_set_slave(control.ctx, 1);
        readRegs = modbus_read_registers(control.ctx, CTRL_POS_ACTUAL, 2, regsValue );
//        ROS_INFO(":%d ",readRegs);
        if (readRegs==2)
        {
        position_left = (int32_t)regsValue[0]<<16 | regsValue[1];
        position_left1 = position_left*(-1);
//        ROS_INFO("left:%d %d",readRegs, abs((int16_t)regsValue[0]));
//         ROS_INFO("left:%d %d ",readRegs, position_left1);
        }

        modbus_set_slave(control.ctx, 2);
        readRegs = modbus_read_registers(control.ctx, CTRL_POS_ACTUAL, 2, regsValue );
        if (readRegs==2)
        {
        position_right= (int32_t)regsValue[0]<<16 |regsValue[1];
//        ROS_INFO("right:%d %d",readRegs, position_right);
        }

		//速度小于1m/s的数据才有效
        if((abs(position_left1 - position_left_last) < 21000) && (abs(position_right - position_right_last) < 21000))
		{
			position_left_last = position_left1;
			position_right_last = position_right;

        	agv_status_data.position_left=position_left1;
        	agv_status_data.position_right=position_right;
        	control.pubStatus.publish(agv_status_data);
		}

       ROS_INFO("Publish Position Info: left:%d  right:%d",
                agv_status_data.position_left , agv_status_data.position_right);



		ros::spinOnce(); 
		loop_rate.sleep(); 
	}

	return 0;
}

/******** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******** End *****/
