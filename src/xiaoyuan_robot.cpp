#include "xiaoyuan_robot.h"


/***
 @ Description	-> 
 @ Param		-> 6D 
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> keysi_start_object()
***/

keysi_start_object::keysi_start_object()
{
	memset(&Reciver_data, 0, sizeof(Struct_Union));
	memset(&SendSpeed_data, 0, sizeof(Struct_Union));
	x = y = th = vx = vy = vth = dt = 0.0;
	hear_status = false;
	m_str_robot_mode = "none";
	m_bstop_flag = false;

	this->SendSpeed_data.Upload_Speed.Header = RECIVER_DATA_HEADER;
	this->SendSpeed_data.Upload_Speed.check_sum = 0xdd;

	/* Get Luncher file define value */
	ros::NodeHandle nh_private("~");
	nh_private.param<bool>("use_imu_provied_Z", this->use_imu_provied_Z, true); 
	nh_private.param<std::string>("usart_port", this->usart_port, "/dev/ttyUSB0"); 
   	nh_private.param<int>("baud_data", this->baud_data, 115200); 
   	nh_private.param<std::string>("robot_frame_id", this->robot_frame_id, "base_link");
	nh_private.param<std::string>("smoother_cmd_vel", this->smoother_cmd_vel, "/cmd_vel");

	nh_private.param<float>("filter_Vx_match", this->filter_Vx_match, 1.045f); 
	nh_private.param<float>("filter_Vth_match", this->filter_Vth_match, 1.02f); 

	/* Create a boot node for the underlying driver layer of the robot base_controller */
	this->cmd_vel_sub = n.subscribe(smoother_cmd_vel, 100, &keysi_start_object::cmd_velCallback, this);
	this->follow_cmd_vel_sub = n.subscribe("/follow_cmd_vel", 100, &keysi_start_object::follow_cmd_velCallback, this);
	this->movebase_cmd_vel_sub = n.subscribe("/movebase_cmd_vel", 100, &keysi_start_object::movebase_cmd_velCallback, this);
	this->cmd_heart_status = n.subscribe("/cmd_heart_status", 100, &keysi_start_object::cmd_heart_statusCallback, this);
	this->Imu_velocity_sub = n.subscribe("/imu_velocity_z", 100, &keysi_start_object::odomPose_velCallback, this);
	this->robot_mode_sub = n.subscribe("/robot_mode", 1000, &keysi_start_object::robot_mode_callback, this);
	this->stop_flag_sub = n.subscribe("/stop_flag", 1000, &keysi_start_object::stop_flag_callback, this);
	this->get_model_sub = n.subscribe("/get_model", 1000, &keysi_start_object::get_model_callback, this);

	this->odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	this->follow_pub = n.advertise<std_msgs::Int32MultiArray>("/follow_msg",1000);
	this->move_base_cancel_pub = n.advertise<std_msgs::String>("/move_base_cancel",1000);
	this->get_model_pub = n.advertise<std_msgs::String>("/get_model",1000);
	// heartbeatTimer =  n.createTimer(ros::Duration(0.1), &keysi_start_object::SendHeartbeat2Gujin,this);

	/**open seril device**/
	try{
         Robot_Serial.setPort(this->usart_port);
         Robot_Serial.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(2000);
         Robot_Serial.setTimeout(to);
         Robot_Serial.open();
    }
	catch (serial::IOException& e){
		 ROS_ERROR_STREAM("Unable to open port ");
	}
	if(Robot_Serial.isOpen()){
	 	ROS_INFO_STREAM("Serial Port opened");
	}else{
	}
}

void keysi_start_object::stop_flag_callback(const std_msgs::Bool::ConstPtr& stop)
{
	m_bstop_flag = stop->data;
	if (m_bstop_flag)
	{
		ROS_INFO("keysi_start_object:: stop !");
		if (m_str_robot_mode == "follow" || m_str_robot_mode == "navi")
		{
			this->SendSpeed_data.Upload_Speed.X_speed = 0;
			this->SendSpeed_data.Upload_Speed.Y_speed = 0;
			this->SendSpeed_data.Upload_Speed.Z_speed = 0;
	
		//ROS_INFO("sendx = %f, senz = %f",twist_aux.linear.x,twist_aux.angular.z);
			Robot_Serial.write(SendSpeed_data.buffer, sizeof(SendSpeed_data.buffer));
		}
	}else
	{
		ROS_INFO("keysi_start_object:: release !");
	}
}
void keysi_start_object::get_model_callback(const std_msgs::String::ConstPtr& robot_mode)
{
	if ("get_model"== robot_mode->data)
	{
		ROS_INFO("keysi_start_object:: get_model_callback");
		std_msgs::String send_msg;
		send_msg.data = m_str_robot_mode;
		get_model_pub.publish(send_msg);
	}
}
void keysi_start_object::robot_mode_callback(const std_msgs::String::ConstPtr& robot_mode)
{
	if (m_str_robot_mode != robot_mode->data)
	{
		ROS_INFO("keysi_start_object:: start cancel goal");
		std_msgs::String send_msg;
		move_base_cancel_pub.publish(send_msg);
	}
	m_str_robot_mode = robot_mode->data;
	ROS_INFO("keysi_start_object::robot_mode is : [%s]", m_str_robot_mode.c_str());
}
keysi_start_object::~keysi_start_object()
{
	close(uart_fd);
}
/***
 @ Description	-> cmd_vel Callback function
 @ Param		-> const geometry_msgs::Twist &twist_aux 
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> keysi_start_object::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
***/
void keysi_start_object::follow_cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;
	if (m_str_robot_mode == "follow" && !m_bstop_flag)
	{
	
		this->SendSpeed_data.Upload_Speed.X_speed = -twist_aux.linear.x;
		this->SendSpeed_data.Upload_Speed.Y_speed = twist_aux.linear.y;
		this->SendSpeed_data.Upload_Speed.Z_speed = -twist_aux.angular.z;
		
		//ROS_INFO("sendx = %f, senz = %f",twist_aux.linear.x,twist_aux.angular.z);
		Robot_Serial.write(SendSpeed_data.buffer, sizeof(SendSpeed_data.buffer));
	}
}
void keysi_start_object::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	if (m_str_robot_mode == "none")
	{
		geometry_msgs::Twist twist = twist_aux;

		this->SendSpeed_data.Upload_Speed.X_speed = -twist_aux.linear.x;
		this->SendSpeed_data.Upload_Speed.Y_speed = twist_aux.linear.y;
		this->SendSpeed_data.Upload_Speed.Z_speed = -twist_aux.angular.z;
	
		//ROS_INFO("sendx = %f, senz = %f",twist_aux.linear.x,twist_aux.angular.z);
		Robot_Serial.write(SendSpeed_data.buffer, sizeof(SendSpeed_data.buffer));
	}
}
void keysi_start_object::movebase_cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	if (m_str_robot_mode == "navi" && !m_bstop_flag)
	{
		geometry_msgs::Twist twist = twist_aux;

		this->SendSpeed_data.Upload_Speed.X_speed = -twist_aux.linear.x;
		this->SendSpeed_data.Upload_Speed.Y_speed = twist_aux.linear.y;
		this->SendSpeed_data.Upload_Speed.Z_speed = -twist_aux.angular.z;
		
		//ROS_INFO("sendx = %f, senz = %f",twist_aux.linear.x,twist_aux.angular.z);
		Robot_Serial.write(SendSpeed_data.buffer, sizeof(SendSpeed_data.buffer));
	}
}
void keysi_start_object::cmd_heart_statusCallback(const geometry_msgs::Twist &twist_aux)
{
	/*
	if (twist_aux.linear.x < 0)
	{
		hear_status = false;		
	}else
	{
		hear_status = true;
	}
	*/
}

void keysi_start_object::SendHeartbeat2Gujin(const ros::TimerEvent& e)
{
	Struct_Union gujin_data;
	memset(&gujin_data, 0, sizeof(Struct_Union));
	gujin_data.Upload_Speed.Charger_Status = 0x55;
	gujin_data.Upload_Speed.Header = RECIVER_DATA_HEADER;
	gujin_data.Upload_Speed.check_sum = 0xdd;
	//if (hear_status == true)
	{
		Robot_Serial.write(gujin_data.buffer, sizeof(gujin_data.buffer));
		// ROS_INFO("hahahahamh ");	
	}
	
}

/***
 @ Description	-> odom velocity Z form imu 
 @ Param		-> const std_msgs::Float64 &data
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> void odomZ_velCallback(const std_msgs::Float64 &data)
***/
void keysi_start_object::odomPose_velCallback(const geometry_msgs::Pose &Pose_msg)
{
		//this->vth =Pose_msg.position.z ;//* filter_Vth_match;
		Current_Pose = Pose_msg;
}

/***
 @ Description	-> Publisher Odom
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> void keysi_start_object::PublisherOdom()
***/
void keysi_start_object::PublisherOdom()
{
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//ROS_INFO("th = %f", th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = this->robot_frame_id;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = odom_quat;
    // odom_trans.transform.rotation.x = 0.0;
    // odom_trans.transform.rotation.y = 0.0;
    // odom_trans.transform.rotation.z = Current_Pose.orientation.z;
    // odom_trans.transform.rotation.w = Current_Pose.orientation.w;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = odom_quat;
    // odom.pose.pose.orientation.x = 0.0;
    // odom.pose.pose.orientation.y = 0.0;
    // odom.pose.pose.orientation.z = Current_Pose.orientation.z;
    // odom.pose.pose.orientation.w = Current_Pose.orientation.w;

    //set the velocity
    odom.child_frame_id = this->robot_frame_id;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
}

/***
 @ Description	-> send and get stm32 board data
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> bool keysi_start_object::ReadFormUart()
 @ return 		-> status
***/
bool keysi_start_object::ReadFormUart()
{	
	unsigned char LocalCheckSum = 0xdd;
	unsigned char CheckSumBuffer[1];
	std_msgs::Int32MultiArray  follow_msg;
	follow_msg.data.clear();
	//memset(&follow_msg,0,sizeof(beginner_tutorials::follow_msg));

	Robot_Serial.read(Reciver_data.buffer,sizeof(Reciver_data.buffer));
	if (Reciver_data.Upload_Speed.Header == RECIVER_DATA_HEADER)
	{
		if (Reciver_data.Upload_Speed.check_sum == LocalCheckSum)
		{
			/* Get robot speed value*/
			//this->vx  = Reciver_data.Upload_Speed.X_speed * filter_Vx_match;
			this->vy  = Reciver_data.Upload_Speed.Y_speed;
			//this->vth = -Reciver_data.Upload_Speed.Z_speed * filter_Vth_match; 	// this value (Vth) from imu_node topic publish msgs
			this->vx  = -Reciver_data.Upload_Speed.X_speed * filter_Vx_match;
			this->vth = Reciver_data.Upload_Speed.Z_speed * filter_Vth_match;
		    //ROS_INFO("left = %f, mid = %f, right = %d,stop = %d"
			// ,Reciver_data.Upload_Speed.US[0],Reciver_data.Upload_Speed.US[1],Reciver_data.Upload_Speed.US[2],Reciver_data.Upload_Speed.Stop_Status);
			//ROS_INFO("vx = %f,vth= %f", vx, vth);

			follow_msg.data.push_back(Reciver_data.Upload_Speed.US[0]);
			follow_msg.data.push_back(Reciver_data.Upload_Speed.US[1]);
			follow_msg.data.push_back(Reciver_data.Upload_Speed.US[2]);
			follow_msg.data.push_back(Reciver_data.Upload_Speed.Stop_Status);
			follow_msg.data.push_back(this->vx * 1000);
			follow_msg.data.push_back(this->vy * 1000);
			follow_msg.data.push_back(this->vth * 1000);

			follow_pub.publish(follow_msg);
			return true;
		}
	} 
	Robot_Serial.read(CheckSumBuffer,sizeof(CheckSumBuffer));
	//ROS_INFO("[ZHOUXUEWEI] Get base controller data error!");
	return false;
}
/***
 @ Description	-> loop 
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> bool keysi_start_object::ReadAndWriteLoopProcess()
 @ return 		-> status
***/
bool keysi_start_object::ReadAndWriteLoopProcess()
{
	this->last_time = ros::Time::now();
	while(ros::ok())
	{
		this->current_time = ros::Time::now();
		this->dt = (current_time - last_time).toSec();

		if (true == ReadFormUart()) 	/* Get npu data include robot move speed and action status information*/
		{
			/* Calculation tf and odom */
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;
			x += delta_x;
			y += delta_y;
			th += delta_th;			
			
			PublisherOdom();				/* Publisher odom topic */
		}

		this->last_time = current_time;
		ros::spinOnce();
	}
}

int main(int argc, char** argv)
{
	/* Voltage thread fb*/

	ros::init(argc, argv, "base_controller");
	ROS_INFO("[ZHOUXUEWEI] base controller node start! ");

	keysi_start_object Robot_Control; 
	Robot_Control.ReadAndWriteLoopProcess();
	
	return 0;
}


