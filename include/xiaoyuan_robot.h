
#ifndef __XIAOYUAN_ROBOT_H_
#define __XIAOYUAN_ROBOT_H_
// sudo apt-get install ros-melodic-serial
// rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
// sudo usermod -aG　dialout wsh

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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"

#define header_data 		0xa0b0c0d0
#define RECIVER_DATA_HEADER 0xd0c0b0a0
#define PROTOBUFLENGHT		24

using namespace std;

//#define ROS_INFO_OK		/* Log */

#pragma pack(1)
typedef union _Upload_speed_   
{
	unsigned char buffer[PROTOBUFLENGHT];
	struct _Speed_data_
	{
		unsigned int Header;
		float X_speed, Y_speed, Z_speed;

		unsigned char Charger_Status;
		unsigned char Ahand_Obstacle;
		unsigned char Behand_Obstacle;
		unsigned char US[3];
		unsigned char Stop_Status;
		unsigned char check_sum;

	}Upload_Speed;
}Struct_Union;
#pragma pack(4)


class keysi_start_object
{
	public:
		keysi_start_object();
		~keysi_start_object();

		/* /cmd_val topic call function */
		void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
		void follow_cmd_velCallback(const geometry_msgs::Twist &twist_aux);
		void movebase_cmd_velCallback(const geometry_msgs::Twist &twist_aux);
		void cmd_heart_statusCallback(const geometry_msgs::Twist &twist_aux);
		void odomPose_velCallback(const geometry_msgs::Pose &Pose_msg);
		void robot_mode_callback(const std_msgs::String::ConstPtr& robot_mode);
		void get_model_callback(const std_msgs::String::ConstPtr& get_model);
		void stop_flag_callback(const std_msgs::Bool::ConstPtr& stop);


		/* Read/Write data from ttyUSB */
		bool ReadFormUart();
		bool WriteToUart(unsigned char*){}
		bool ReadAndWriteLoopProcess();

		/* This node Publisher topic and tf */
		void PublisherOdom();
		void SendHeartbeat2Gujin(const ros::TimerEvent& e);

		serial::Serial Robot_Serial; //声明串口对象 
		bool hear_status;

	private:
		/* ROS param value */
		int uart_fd, read_size, write_size, baud_data;
		bool use_imu_provied_Z;
		string usart_port, robot_frame_id, smoother_cmd_vel, m_str_robot_mode;
		bool m_bstop_flag;
		float filter_Vx_match,filter_Vth_match;
 

		/* Ros node define*/
		ros::NodeHandle n;
		ros::Time current_time, last_time;
		ros::Timer heartbeatTimer;

		ros::Subscriber cmd_vel_sub, Imu_velocity_sub, follow_cmd_vel_sub, movebase_cmd_vel_sub, robot_mode_sub, stop_flag_sub, get_model_sub;
		ros::Publisher odom_pub,follow_pub, get_model_pub, move_base_cancel_pub, m_pstop_flag_pub;
		ros::Subscriber cmd_heart_status;

		//Defines the message type to be transmitted geometry_msgs
		geometry_msgs::Quaternion odom_quat;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::Pose Current_Pose;

		Struct_Union Reciver_data, SendSpeed_data;         

		/* Odom and tf value*/
		double x, y, th, vx, vy, vth, dt;
};


#endif


