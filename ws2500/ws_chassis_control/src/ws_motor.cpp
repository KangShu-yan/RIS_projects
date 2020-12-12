#include <ws_motor/ws_motor.h>
namespace ws_motor
{
WSMotor::WSMotor(ros::NodeHandle nh) : nh_(nh),loop_rate_(1000)	//construct
{
  	// Declare variables that can be modified by launch file or command line.
	double rate = 200.0;
	double state_pub_rate=125.;
	double framerate = 1000.0;
	ros::NodeHandle pnh("~");
  	pnh.param("rate", rate, rate);
  	pnh.param("framerate", framerate_, framerate);
	pnh.param("state_pub_rate", state_pub_rate, state_pub_rate);
	
//	loop_rate_=framerate_;	//以hz设置循环频率
	
	chassis_client_.create_client("192.168.1.10",4001,1);//async = 1
//	Create timer.
  	state_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / state_pub_rate), &WSMotor::statePubTimerCallback, this);	//sec nsec 8ms
	odom_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &WSMotor::odomPubTimerCallback, this);	//5ms 
	//发布话题
	state_pub_ = nh_.advertise<ws_motor::wsMotorData>("/wsMotor/motorState", 10);	//缓冲队列为10发布话题
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/wsMotor/odom", 50);
	//订阅话题
	twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("/ws2500_chassis/cmd_vel",20,&WSMotor::cmdVelCallback,this);
	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/IMU/data_raw",20,&WSMotor::imuCallback,this);
}
void WSMotor::init_motor()
{
	while(chassis_client_.Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
	get_CAN_counter_=0;
	send_CAN_counter_=0;
	control_mode_=1;
	yaw_angle_ = 0;
	for(int idx=0;idx<6;idx++)
	{
		set_motor_rpm_[idx]=0;
		set_motor_Nm_[idx]=0;
		motor_rpm_[idx]=0;
		motor_Nm_[idx]=0;
		motor_tempoc_[idx]=0;
		motor_break_code_[idx]=0;
		motor_odom_er_[idx]=0;
		mdriver_tempoc_[idx]=0;
		send_motor_cmd(idx,control_mode_);	// 电机上电
	}
	
	control_mode_=2;
	ROS_INFO("START");
	
	current_time_ = ros::Time::now();
  	last_time_ = ros::Time::now();
	motor_spin_once();	
}
void WSMotor::stop_motor()
{
	for(int16_t motor_id= 1;motor_id<=6;motor_id++)
	{
		send_motor_cmd(motor_id,4);	//电机下电
	}
  	odom_pub_.shutdown();	//Shutdown publisher 
  	state_pub_.shutdown();
//  	chassis_client_.Disconnect();
}
//Input an array and its length,no output
void WSMotor::printf_frame(const unsigned char *frame,const int size)	
{
	for(int index=0;index<size;index++)
	{
		printf("%.2x ",(unsigned char)frame[index]);	
	}
	printf("\n");
}

void WSMotor::analyse_motor_data(unsigned char *frame,int size)
{
	//08 00 00 00 72 00 00 00 00 00 00 00 00 
//	std::lock_guard<mutex> lock(m_mutex_);
	static int32_t motor_rpm[6]={0},motor_Nm[6]={0};
	static int32_t last_motor_rpm[6],last_motor_Nm[6]={0};
	static uint32_t motor_break_code[6]={0};
	static uint16_t motor_odom_er[6]={0};
	static uint8_t motor_tempoc[6]={0},mdriver_tempoc[6]={0};
	static uint16_t get_flag=0;
	if(frame[0]==0x08&&frame[1]==0x00)
	{
		if(frame[4]==0x72)		//轮1
		{
			get_flag = get_flag|0x01;
		  	motor_rpm[0] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[0] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		  	ROS_INFO("motor1_rpm = %d,motor1_Nm = %d",motor_rpm[0],motor_Nm[0]);
		  	
		}
		else if(frame[4]==0x73)
		{
			get_flag = get_flag|(0x01<<1);
			motor_rpm[1] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[1] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		}
		else if(frame[4]==0x74)
		{
			get_flag = get_flag|(0x01<<2);
		 	motor_rpm[2] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[2] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		}
		else if(frame[4]==0x75)
		{
			get_flag = get_flag|(0x01<<3);
		 	motor_rpm[3] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[3] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		}
		else if(frame[4]==0x76)
		{
			get_flag = get_flag|(0x01<<4);
		 	motor_rpm[4] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[4] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		}
		else if(frame[4]==0x77)
		{
			get_flag = get_flag|(0x01<<5);
		 	motor_rpm[5] = (int32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
		  	motor_Nm[5] =(int32_t)frame[9]|(frame[10]<<8)|(frame[11]<<16)|(frame[12]<<24);
		}
		else if(frame[4]==0x52)		//
		{
			get_flag = get_flag|(0x01<<6);
			motor_break_code[0]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[0]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[0]=frame[10];
			motor_odom_er[0]=(uint16_t)frame[11]|(frame[12]<<8);
				
	//		printf("\nframe[5] = %.2x frame[6] = %.2x motor_tempoc= %.2x \n",frame[5],frame[6],motor_tempoc[0]);
			ROS_INFO("\033[34mmotor1_tempoc = %d,mdriver_tempoc = %d ,motor1_odom_er = %d\033[0m",motor_tempoc[0],mdriver_tempoc[0],motor_odom_er[0]);
		}
		else if(frame[4]==0x53)
		{
			get_flag = get_flag|(0x01<<7);
			motor_break_code[1]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[1]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[1]=frame[10];
			motor_odom_er[1]=(uint16_t)frame[11]|(frame[12]<<8);
		}
		else if(frame[4]==0x54)
		{
			get_flag = get_flag|(0x01<<8);
			motor_break_code[2]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[2]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[2]=frame[10];
			motor_odom_er[2]=(uint16_t)frame[11]|(frame[12]<<8);
		}
		else if(frame[4]==0x55)
		{
			get_flag = get_flag|(0x01<<9);
			motor_break_code[3]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[3]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[3]=frame[10];
			motor_odom_er[3]=(uint16_t)frame[11]|(frame[12]<<8);
		}
		else if(frame[0]==0x56)
		{
			get_flag = get_flag|(0x01<<11);
			motor_break_code[4]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[4]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[4]=frame[10];
			motor_odom_er[4]=(uint16_t)frame[11]|(frame[12]<<8);
		}
		else if(frame[0]==0x57)
		{
			get_flag = get_flag|(0x01<<11);
			motor_break_code[5]= (uint32_t)frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
			motor_tempoc[5]=frame[9];	//frame[6]  mdriver_tempoc;
			mdriver_tempoc[5]=frame[10];
			motor_odom_er[5]=(uint16_t)frame[11]|(frame[12]<<8);
		}
		else;
	}
	if(get_flag == 0x0fff)		//
	{
		get_flag=0;
		m_mutex_.lock();
		for(int idx=0;idx<6;idx++)
		{
			if(abs(motor_rpm[idx]/65536-last_motor_rpm[idx])<500)	//sign also should be considered 
			{
				motor_rpm_[idx]=motor_rpm[idx]/65536;
				last_motor_rpm[idx]=motor_rpm[idx]/65536;
			}
			else motor_rpm_[idx]=last_motor_rpm[idx]/65536.0;
			motor_Nm_[idx]=motor_Nm[idx]/65536.0;		//65536
			motor_break_code_[idx]=motor_break_code[idx];
			motor_tempoc_[idx]=motor_tempoc[idx]-50;
			mdriver_tempoc_[idx]=mdriver_tempoc[idx]-50;
			motor_odom_er_[idx]=motor_odom_er[idx]/32.0;	//32为电机每转一圈的电圈数
		}	
		get_twist();
		m_mutex_.unlock();
	}
	ROS_INFO("got \033[32m %ldth CAN\033[0m frame with length %d bytes",++get_CAN_counter_,size);
	printf_frame(frame,size);
}
void WSMotor::get_twist()
{
	double WHEEL_PI = 3.1415926,WHEEL_D = 1.3; 
	double v1 = (motor_rpm_[0])/60.0*WHEEL_D *WHEEL_PI;
  	double v2 = (motor_rpm_[1])/60.0*WHEEL_D *WHEEL_PI;
  	double v3 = (motor_rpm_[2])/60.0*WHEEL_D *WHEEL_PI;
  	double v4 = (motor_rpm_[3])/60.0*WHEEL_D *WHEEL_PI;
  	double v5 = (motor_rpm_[4])/60.0*WHEEL_D *WHEEL_PI;
  	double v6 = (motor_rpm_[5])/60.0*WHEEL_D *WHEEL_PI;
  	vel_x_ = (v1-v2+v3-v4+v5-v6)/6;
// 	vel_y_ = acc_y_*dt+vel_y_;
  	vel_y_ =  0;		//理想状态，不考虑测滑
  	vel_th_radps_ = vel_th_radps_;
}


//发布电机与驱动器状态信息，包括电机转速，电机温度，里程，驱动器工作状态
void WSMotor::statePubTimerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
	ws_motor::wsMotorData msg;
	int temp_array[6]={0};
//	temp_array[0]=12;
	m_mutex_.lock();
	std::vector<double> m_rpm(motor_rpm_,motor_rpm_+6);
	std::vector<double> m_Nm(motor_Nm_,motor_Nm_+6);
	std::vector<int64_t> m_break(motor_break_code_,motor_break_code_+6);
	std::vector<int16_t> m_temp(motor_tempoc_,motor_tempoc_+6);
	std::vector<int16_t> md_temp(mdriver_tempoc_,mdriver_tempoc_+6);
	std::vector<int32_t> m_odom(motor_odom_er_,motor_odom_er_+6);
	std::vector<int32_t> set_rpm(set_motor_rpm_,set_motor_rpm_+6);
	std::vector<int32_t> set_Nm(set_motor_Nm_,set_motor_Nm_+6);
	msg.send_CAN_counter=send_CAN_counter_;
	msg.get_CAN_counter=get_CAN_counter_;
	m_mutex_.unlock();

	msg.set_motor_rpm = set_rpm;
	msg.set_motor_Nm =set_Nm;
	msg.set_motor_rpm.resize(6);
	msg.set_motor_Nm.resize(6);
	
	msg.motor_rpm=m_rpm;
	msg.motor_rpm.resize(6);	//设定容器尺寸
	msg.motor_Nm=m_Nm;
	msg.motor_Nm.resize(6);	//设定容器尺寸
	msg.motor_break_code=m_break;
	msg.motor_break_code.resize(6);	//设定容器尺
	msg.motor_tempoC=m_temp;
	msg.motor_tempoC.resize(6);	//设定容器尺寸
	msg.mdriver_tempoC=md_temp;
	msg.mdriver_tempoC.resize(6);	//设定容器尺寸
	msg.motor_odom_er=m_odom;
	msg.motor_odom_er.resize(6);	//设定容器尺寸
	state_pub_.publish(msg);
//	ROS_INFO("published state msg");

}
//发布里程计
void WSMotor::odomPubTimerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
	tf::TransformBroadcaster odom_broadcaster;
	m_mutex_.lock();
	double x = pos_x_;
  	double y = pos_y_;
  	double th = th_rad_;
  	double vx = vel_x_;
  	double vy = vel_y_;
  	double vth = vel_th_radps_;
  	m_mutex_.unlock();
  	current_time_ = ros::Time::now();
	//compute odometry in a typical way given the velocities of the robot
    double dt = (current_time_ - last_time_).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    //publish the message
    odom_pub_.publish(odom);
	last_time_ = current_time_;
	m_mutex_.lock();
	pos_x_=x;
    pos_y_=y;
  	th_rad_ = th;
  	m_mutex_.unlock();
//	ROS_INFO("published odom msg");
}
void WSMotor::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	Eigen::Matrix3d rotation_matrix;	//
	Eigen::Quaterniond quaternion;
	quaternion.w() = msg->orientation.w;
	quaternion.x() = msg->orientation.x;
	quaternion.y() = msg->orientation.y;
	quaternion.z() = msg->orientation.z;
	rotation_matrix = quaternion.matrix();
	Eigen::Vector3d  eluer_angle= rotation_matrix.eulerAngles(0,1,2);	//x-y-z
	m_mutex_.lock();
	yaw_angle_ = eluer_angle[2];	//航向角
	vel_th_radps_ = msg->angular_velocity.z;	//角速度
	acc_y_ = msg->linear_acceleration.y;
	m_mutex_.unlock();
}
void WSMotor::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	static int count=0;
	double B=1.604;		//
	double wheel_r=0.65;//0.15
//	double vx=0,wz=0; 
//	vx = msg->linear.x;
//	wz = msg->angular.z;
	m_mutex_.lock();
	control_mode_ = msg->linear.y;	//
	if(control_mode_==2)
	{
		set_motor_rpm_[0] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 1 
		set_motor_rpm_[2] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 3
		set_motor_rpm_[4] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 5
	
		set_motor_rpm_[1] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 2
		set_motor_rpm_[3] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 4
		set_motor_rpm_[5] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 6
	}else if(control_mode_==3)	// Fd
	{
		set_motor_Nm_[0] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 1 
		set_motor_Nm_[2] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 3
		set_motor_Nm_[4] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 5
	
		set_motor_Nm_[1] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 2
		set_motor_Nm_[3] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 4
		set_motor_Nm_[5] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 6
	}
	else;
//	for(int idx=0;idx<6;idx++)
//	{
//		printf("set_rpm[%d] = %d , ",idx,set_motor_rpm_[idx]);
//	}
	m_mutex_.unlock();
	
//	printf("\n");
//	ROS_INFO("I got twist.linear.x = %lf , twist.angular.z = %lf ,control_mode = %lf",msg->linear.x, msg->angular.z,msg->linear.y);
}
void WSMotor::syschronic_frame(void)
{
	unsigned char s_data[13]={0};
	int size=0;
	s_data[0]=0x08;
	s_data[4]=0x33;
	//size = ros_ser_.write(s_data,13);	//
	chassis_client_.Send(s_data,13);
//	printf_frame(s_data,13);
//	ROS_INFO("sent \033[32m syschronic_frame on %ldth\033[0m CAN frame.",++send_CAN_counter_);
}
//
void WSMotor::send_motor_cmd(int motor_id ,int16_t control_mode)
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	int size=0;
	union int32_uchar setSpeedRpm;
	union int32_uchar setTorqueNm;
	m_mutex_.lock();
	setSpeedRpm.val = set_motor_rpm_[motor_id-1]*65536.;
	setTorqueNm.val = set_motor_Nm_[motor_id-1]*65536.;
	m_mutex_.unlock();
	s_data[s_length++]=0x08;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x44+motor_id;
	
	if(control_mode==2)	//速度控制模式
	{
		s_data[s_length++]=setSpeedRpm.buf[0];	//低位
		s_data[s_length++]=setSpeedRpm.buf[1];
		s_data[s_length++]=setSpeedRpm.buf[2];
		s_data[s_length++]=setSpeedRpm.buf[3];
		s_length+=2;
		s_data[s_length++]=0x55;
		if(chassis_client_.Send(s_data,13)==0){ROS_INFO("send ok");}
		printf_frame(s_data,13);
		ROS_INFO("sent \033[32mspeed mode %ldth\033[0m CAN frame.",++send_CAN_counter_);
		syschronic_frame();
		
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		
	}
	else if(control_mode==3)	//力矩控制模式
	{
		s_data[s_length++]=setTorqueNm.buf[0];	//低位
		s_data[s_length++]=setTorqueNm.buf[1];
		s_data[s_length++]=setTorqueNm.buf[2];
		s_data[s_length++]=setTorqueNm.buf[3];
		s_length=s_length+2;
		s_data[s_length++]=0xaa;
//		size = ros_ser_.write(s_data,10);
		chassis_client_.Send(s_data,13);
//		printf_frame(s_data,13);

//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		syschronic_frame();
		
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		// control_mode_=0;
	}
	
	else if(control_mode==4)	//motor power off
	{
		s_length+=6;
		s_data[s_length++]=0x5a;
		chassis_client_.Send(s_data,13);
//		printf_frame(s_data,13);
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		syschronic_frame();
		
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		// control_mode_=0;
	}
	else
	{
		s_length+=6;
		s_data[s_length++]=0xa5;
		chassis_client_.Send(s_data,13);
//		printf_frame(s_data,13);
		ROS_INFO("sent \033[32m power on mode %ldth\033[0m CAN frame.",++send_CAN_counter_);
//		syschronic_frame();
		
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
		// syschronic_frame();
	}		
}
//motor control 
void WSMotor::control_motor()
{
	for(int motor_idx=1;motor_idx<=6;motor_idx++)
	{
		send_motor_cmd(motor_idx,control_mode_);	//速度模式
	}
//	ROS_INFO("wrote data");
}
// As third thread, process motor control and pub_msg
void WSMotor::motor_spin_once()
{
	int counter=0;
	int byte_len=0;
	unsigned char feedback_data[20]={0};
	while (nh_.ok())
    {
     	counter++;
//		if(counter%10==0)
//		{
//		 	pub_msg();
//		}
		if(chassis_client_.Receive(feedback_data, 100, 0, &byte_len, 0)==1)
			analyse_motor_data(feedback_data,byte_len);	
		 else
			ROS_WARN_STREAM_ONCE(" Received wrong!");
//		if(counter%3==0)
//		{
			control_motor();	//tcp-CAN test constant acceleration
			syschronic_frame();
//		}
		ROS_INFO("control_mode = %d",control_mode_);
//		if(counter>1000) counter=0;
		ros::spinOnce();
		loop_rate_.sleep();
	}
//	stop();
}
}

