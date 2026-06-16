
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <skyee_msg/RobotStatusInfo.h>
#include <skyee_msg/ControlCommand.h>
#include <skyee_msg/clean_system.h>
#include <sensor_msgs/Range.h>
using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

// rx[1] chassis status byte bit layout (MCU -> ROS)
// bit0: 1=charging, 0=not charging
// bit1: 1=emergency stop pressed, 0=released
#define CHASSIS_STATUS_BIT_BATTERY_STATE    0x01
#define CHASSIS_STATUS_BIT_EMERGENCY_STOP   0x02



//tx[1] (ROS -> MCU)
#define CHASSIS_CMD_BIT_CLEAN_MODEL      0x01 //清洁模式
#define CHASSIS_CMD_BIT_SWITCH_CONTROL   0x02 //清扫系统总开关
#define CHASSIS_CMD_BIT_BESIDE_BRUSH     0x04 //边刷
#define CHASSIS_CMD_BIT_MIDDLE_BRUSH     0x08 //中刷
#define CHASSIS_CMD_BIT_PUSHBEAM         0x10 //推杆
#define CHASSIS_CMD_BIT_FAN              0x20 //风机
#define CHASSIS_CMD_BIT_WATER_PUMP       0x40 //水泵
#define CHASSIS_CMD_BIT_VIBRATING_DUST   0x80 //振尘

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	

extern sensor_msgs::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;
}RECEIVE_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
	public:
		turn_on_robot();  //Constructor //构造函数
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象 
	private:
		ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄
		ros::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

		ros::Subscriber Cmd_Vel_Sub;
		ros::Subscriber gpio_write_sub_;
		ros::Subscriber Cliff_Sub;
		void Cmd_Vel_Callback(const skyee_msg::ControlCommand &twist_aux);
		void gpioWriteCallback(const skyee_msg::clean_system::ConstPtr &msg);
		void Cliff_Callback(const sensor_msgs::Range &distance);

		ros::Publisher odom_publisher, imu_publisher, voltage_publisher;
		ros::Publisher battery_percent_publisher, battery_state_publisher;
		ros::Publisher scram_button_publisher;
		ros::Publisher eva_robot_status;
		ros::Publisher clean_system_status_publisher_;
		void Publish_Odom();
		void Publish_ImuSensor();
		void Publish_Voltage();
		void Publish_Battery();
		void PublishCleanSystemAndroid();
		void ApplyCleanSystemCommand(const skyee_msg::clean_system &cmd);
		void ApplyCleanSystemHardware();
		uint8_t BuildChassisCommandTxByte() const;
		void WriteMotionFrameToMcu(float linear_x, float linear_y, float angular_z);
		skyee_msg::RobotStatusInfo BuildEvaRobotStatusInfo() const;
		int VoltageToPercent(float voltage) const;
		int GetBatteryStateFromChassis() const;//从底盘获取电池状态
		bool IsEmergencyStopPressed() const;//从底盘获取急停状态

        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();   
		bool Get_Sensor_Data_New();
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables //定义相关变量
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;
		float cliff_threshold;
		bool cliff_switch;
		bool cliff_detected;
		bool robot_clean_model_;
		skyee_msg::clean_system clean_system_state_;
};
#endif
