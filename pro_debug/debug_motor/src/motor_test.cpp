#include <motor_test/motor_test.h>
namespace debug_motor
{
DebugMotor::DebugMotor(ros::NodeHandle nh) : nh_(nh)
{
  	// Set up a dynamic reconfigure server.
  	// Do this before parameter server, else some of the parameter server values can be overwritten.
   	dynamic_reconfigure::Server<debug_motor::motorTestConfig>::CallbackType cb;
  	cb = boost::bind(&DebugMotor::configCallback, this, _1, _2);
  	dr_srv_.setCallback(cb);
  	// Declare variables that can be modified by launch file or command line.
	double rate = 200.0;
	double framerate = 1000.;
	double a = 1000.;
  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
	ros::NodeHandle pnh("~");
  	pnh.param("a", a_, a);
  	pnh.param("rate", rate, rate);
  	pnh.param("framerate", framerate_, framerate);
	chassis_client_.create_client("192.168.1.10",4001,1);
  
	for(int idx=0;idx<6;idx++)
	{
		motor_rpm_[idx]=0;
		motor_Nm_[idx]=0;
		motor_tempoc_[idx]=0;
		motor_break_code_[idx]=0;
		motor_odom_er_[idx]=0;
		mdriver_tempoc_[idx]=0;
	}
	for(int idx=0;idx<1024;idx++)
		for(int sub_idx=0;sub_idx<8;sub_idx++)
			params_[idx][sub_idx]=0;
	params_index_=0;
	send_CAN_counter_=0;
	get_CAN_counter_=0;
  	// Create timer.  5ms 发布一次 
  	timer_ = nh_.createTimer(ros::Duration(1.0/rate), &DebugMotor::timerCallback, this);
  	//15ms
  	control_timer_ = nh_.createTimer(ros::Duration(1.0/100.), &DebugMotor::contorlTimerCallback, this);
  	
}
void DebugMotor::start()
{
	while(chassis_client_.Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
	
	set_speed_rpm_=0;	//Initializes essential variables used to control motor with CAN protocol  
	set_torque_Nm_=0;
	control_mode_=0;
	ROS_INFO("START");
	spin();
    
}
void DebugMotor::stop()
{
  	pub_.shutdown();	//Shutdown publisher
}
//Input an array and its length,output sum,practicing checksum
void DebugMotor::crc(const unsigned char* frame,const int length,unsigned char* sum)
{
	for(int index=0;index<length;index++)	
	{
		*sum+=frame[index];
	}
}
//Input an array and its length,no output
void DebugMotor::printf_frame(const unsigned char *frame,const int size)	
{
	printf("printf_frame size = %d :",size);
	for(int index=0;index<size;index++)
	{
		printf("%.2x ",(unsigned char)frame[index]);	
	}
	printf("\n");
}
void DebugMotor::analyse_CAN_data(unsigned char *frame,int size)
{
	//08 00 00 00 72 00 00 00 00 00 00 00 00 
//	std::lock_guard<mutex> lock(m_mutex);
	static int32_t motor_rpm[6]={0},motor_Nm[6]={0};
	static int32_t last_motor_rpm[6],last_motor_Nm[6]={0};
	static uint32_t motor_break_code[6]={0};
	static uint16_t motor_odom_er[6]={0};
	static uint8_t motor_tempoc[6]={0},mdriver_tempoc[6]={0};
	static int32_t short_frame=0;
	unsigned char one_frame_flag =0;
	
	if(size<13){ROS_INFO("got \033[33m %dth short frame",++short_frame);return;}
	//
	for(int16_t idx = 0;idx<size;)
	{
		if(frame[idx+0]==0x08&&frame[idx+1]==0x00)
		{
			if(frame[idx+4]==0x72)
			{
			  	motor_rpm[0] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[0] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
	//		  	ROS_INFO("motor1_rpm = %d,motor1_Nm = %d",motor_rpm[0],motor_Nm[0]);
				one_frame_flag=1;
			}
			else if(frame[idx+4]==0x73)
			{
				motor_rpm[1] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[1] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
			  	one_frame_flag = 1;
			}
			else if(frame[idx+4]==0x74)
			{
			 	motor_rpm[2] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[2] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
			  	one_frame_flag = 1;
			}
			else if(frame[idx+4]==0x75)
			{
			 	motor_rpm[3] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[3] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
			  	one_frame_flag = 1;
			}
			else if(frame[idx+4]==0x76)
			{
			 	motor_rpm[4] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[4] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
			  	one_frame_flag = 1;
			}
			else if(frame[idx+4]==0x77)
			{
			 	motor_rpm[5] = (int32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
			  	motor_Nm[5] =(int32_t)frame[idx+9]|(frame[idx+10]<<8)|(frame[idx+11]<<16)|(frame[idx+12]<<24);
			  	one_frame_flag = 1;
			}
			else if(frame[idx+4]==0x52)		//
			{
				motor_break_code[0]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[0]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[0]=frame[idx+10];
				motor_odom_er[0]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
		//		printf("\nframe[5] = %.2x frame[6] = %.2x motor_tempoc= %.2x \n",frame[5],frame[6],motor_tempoc[0]);
	//			ROS_INFO("\033[34mmotor1_tempoc = %d,mdriver_tempoc = %d ,motor1_odom_er = %d\033[0m",motor_tempoc[0],mdriver_tempoc[0],motor_odom_er[0]);
			}
			else if(frame[4]==0x53)
			{
				motor_break_code[1]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[1]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[1]=frame[idx+10];
				motor_odom_er[1]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
			}
			else if(frame[4]==0x54)
			{
				motor_break_code[2]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[2]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[2]=frame[idx+10];
				motor_odom_er[2]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
			}
			else if(frame[4]==0x55)
			{
				motor_break_code[3]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[3]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[3]=frame[idx+10];
				motor_odom_er[3]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
			}
			else if(frame[0]==0x56)
			{
				motor_break_code[4]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[4]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[4]=frame[idx+10];
				motor_odom_er[4]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
			}
			else if(frame[0]==0x57)
			{
				motor_break_code[5]= (uint32_t)frame[idx+5]|(frame[idx+6]<<8)|(frame[idx+7]<<16)|(frame[idx+8]<<24);
				motor_tempoc[5]=frame[idx+9];	//frame[6]  mdriver_tempoc;
				mdriver_tempoc[5]=frame[idx+10];
				motor_odom_er[5]=(uint16_t)frame[idx+11]|(frame[idx+12]<<8);
				one_frame_flag = 1;
			}
			else;
		}
		
		if(one_frame_flag==1)
		{
			one_frame_flag=0;
			idx=idx+13;
			ROS_INFO("got \033[32m %ldth CAN\033[0m frame with length %d",++get_CAN_counter_,idx);
		}
		else
		{
			idx=idx+1;
		}
	}
	m_mutex.lock();
	for(int idx=0;idx<6;idx++)
	{
		if(abs(motor_rpm[idx]/65536-last_motor_rpm[idx])<500)	//sign also should be considered 
		{
			motor_rpm_[idx]=motor_rpm[idx]/65536;
			last_motor_rpm[idx]=motor_rpm[idx]/65536;
		}
		else motor_rpm_[idx]=last_motor_rpm[idx]/65536.0;
		
		motor_Nm_[idx]=motor_Nm[idx]/65536;		//65536
		motor_break_code_[idx]=motor_break_code[idx];
		motor_tempoc_[idx]=motor_tempoc[idx]-50;
		mdriver_tempoc_[idx]=mdriver_tempoc[idx]-50;
		motor_odom_er_[idx]=motor_odom_er[idx]/32.0;	//32为电机每转一圈的电圈数
	}	
	m_mutex.unlock();
	ROS_INFO("got \033[32m %ldth CAN\033[0m frame with length %d",++get_CAN_counter_,size);
//	printf_frame(frame,size);
}
//
void DebugMotor::send_data()
{
	
}
void DebugMotor::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
	pub_msg();
//	ROS_INFO(" pub_msg");
}
void DebugMotor::contorlTimerCallback(const ros::TimerEvent &event)
{
	//	ROS_INFO(" control_callback");
	if(send_CAN_)
	{	
		for(int idx=1;idx<=6;idx++)
		{
			control_motor(idx);	
					
	//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",send_CAN_counter_++);
		}
		syschronic_frame();
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",send_CAN_counter_++);	
					
	}
}
//Input configured parameters,no output ,updating parameters of calibration as well as that of motor control 
void DebugMotor::configCallback(debug_motor::motorTestConfig &config, uint32_t level __attribute__((unused)))
{
  //parameters used to motor control with CAN protocol 
  m_mutex.lock();
  set_motor_rpm_[0]=config.set_rpm_1;
  set_motor_Nm_[0]=config.set_current_1;
  set_motor_rpm_[1]=config.set_rpm_2;
  set_motor_Nm_[1]=config.set_current_2;
  set_motor_rpm_[2]=config.set_rpm_3;
  set_motor_Nm_[2]=config.set_current_3;
  set_motor_rpm_[3]=config.set_rpm_4;
  set_motor_Nm_[3]=config.set_current_4;
  set_motor_rpm_[4]=config.set_rpm_5;
  set_motor_Nm_[4]=config.set_current_5;
  set_motor_rpm_[5]=config.set_rpm_6;
  set_motor_Nm_[5]=config.set_current_6;
  
  limited_speed_=config.limited_speed;
  limited_torque_=config.limited_current;
  control_mode_=config.control_mode;
  m_mutex.unlock();
  send_CAN_=config.send_CAN;
//  ROS_INFO("config callback");
  
}
//publish necessary variables
void DebugMotor::pub_msg()
{
	debug_motor::debugMotorData msg;
	int temp_array[6]={0};
//	temp_array[0]=12;
	m_mutex.lock();
	std::vector<double> m_rpm(motor_rpm_,motor_rpm_+6);
	std::vector<double> m_Nm(motor_Nm_,motor_Nm_+6);
	std::vector<int64_t> m_break(motor_break_code_,motor_break_code_+6);
	std::vector<int16_t> m_temp(motor_tempoc_,motor_tempoc_+6);
	std::vector<int16_t> md_temp(mdriver_tempoc_,mdriver_tempoc_+6);
	std::vector<int32_t> m_odom(motor_odom_er_,motor_odom_er_+6);
	std::vector<int32_t> set_rpm(set_motor_rpm_,set_motor_rpm_+6);
	std::vector<int32_t> set_Nm(set_motor_Nm_,set_motor_Nm_+6);
	m_mutex.unlock();
	msg.send_CAN_counter=send_CAN_counter_;
	msg.get_CAN_counter=get_CAN_counter_;
	msg.set_motor_rpm = set_rpm;
	msg.set_motor_Nm =set_Nm;
	msg.set_motor_rpm.resize(6);
	msg.set_motor_Nm.resize(6);
	msg.set_speed_rpm=set_speed_rpm_;
	msg.set_torque_Nm=set_torque_Nm_;
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
	
	pub_.publish(msg);
//	ROS_INFO("published msg");
}
void DebugMotor::syschronic_frame(void)
{
	unsigned char s_data[13]={0};
	int size=0;
	s_data[0]=0x08;
	s_data[4]=0x33;
	chassis_client_.Send(s_data,13);
//	++send_CAN_counter_;
//	printf_frame(s_data,13);
	ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
}
//motor control 
void DebugMotor::control_motor(int motor_id)
{
//	ROS_INFO("CONTORL_MOTOR");
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	
	int size=0;
	union int32_uchar setSpeedRpm;
	union int32_uchar setTorqueNm;
	m_mutex.lock();
	setSpeedRpm.val = set_motor_rpm_[motor_id-1]*65536;
	setTorqueNm.val = set_motor_Nm_[motor_id-1]*65536.;	//
	m_mutex.unlock();
	s_data[s_length++]=0x08;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x44+motor_id;
	if(control_mode_==1)	//速度控制模式
	{
		s_data[s_length++]=setSpeedRpm.buf[0];	//低位
		s_data[s_length++]=setSpeedRpm.buf[1];
		s_data[s_length++]=setSpeedRpm.buf[2];
		s_data[s_length++]=setSpeedRpm.buf[3];
		s_length+=2;
		s_data[s_length++]=0x55;
		chassis_client_.Send(s_data,13);
//		usleep(a_);
//		printf_frame(s_data,13);
		ROS_INFO("sent \033[32m speed cmd motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//		++send_CAN_counter_;
//		syschronic_frame();
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
//		control_mode_=0;
	}
	else if(control_mode_==2)	//力矩控制模式
	{
//		s_length+=2;
//		s_data[s_length++]=(setSpeedNm>>8)&0xff;
//		s_data[s_length++]=(unsigned char)setSpeedNm&0xff;
		s_data[s_length++]=setTorqueNm.buf[0];	//低位
		s_data[s_length++]=setTorqueNm.buf[1];
		s_data[s_length++]=setTorqueNm.buf[2];
		s_data[s_length++]=setTorqueNm.buf[3];
		s_length=s_length+2;
		s_data[s_length++]=0xaa;
		chassis_client_.Send(s_data,13);
//		++send_CAN_counter_;
//		printf_frame(s_data,13);
		// send_CAN_counter_++;
		ROS_INFO("sent \033[32mtorque cmd motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//		syschronic_frame();
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
//		usleep(a_);
//		control_mode_=0;
	}
	else if(control_mode_==3)	//motor power on
	{
		s_length+=6;
		s_data[s_length++]=0xa5;
		chassis_client_.Send(s_data,13);
//		printf_frame(s_data,13);
//		++send_CAN_counter_;
		ROS_INFO("sent \033[32m power on motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//		usleep(a_);
//		syschronic_frame();
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
//		control_mode_=0;
	}	
	else if(control_mode_==6)	//constant accelerate for speed
	{
//		ROS_INFO("control_mode_ = %d",control_mode_);
		static float temp_speed_rpm=0;
		static uint8_t temp_rpm_flag=0;
		if(temp_rpm_flag==0)
		{	
//			ROS_INFO("acceleration>>>>>>>>>>>>>>>.");
			if(temp_speed_rpm==limited_speed_)temp_rpm_flag=1;
			
			temp_speed_rpm+=0.5;
//			ROS_INFO("\033[32mtemp_speed_rpm = %f \033[0m",temp_speed_rpm);
			set_speed_rpm_=temp_speed_rpm;
			setSpeedRpm.val=temp_speed_rpm*65536;
			s_data[s_length++]=setSpeedRpm.buf[0];	//低位
			s_data[s_length++]=setSpeedRpm.buf[1];
			s_data[s_length++]=setSpeedRpm.buf[2];
			s_data[s_length++]=setSpeedRpm.buf[3];
			s_length+=2;
			s_data[s_length++]=0x55;
			chassis_client_.Send(s_data,s_length);
//			printf_frame(s_data,13);
			ROS_INFO("sent \033[32m motor_id = %d, %ldth\033[0m CAN frame.",motor_id ,++send_CAN_counter_);
//			++send_CAN_counter_;
//			syschronic_frame();
//			ROS_INFO("sent \033[32m %ldth\033[0m CAN frame.",++send_CAN_counter_);
//			usleep(a_);	//10ms
		}
		else if(temp_rpm_flag==1)
		{
//			ROS_INFO("deceleration>>>>>>>>>>>.");
			if(temp_speed_rpm==-limited_speed_)temp_rpm_flag=0;
			temp_speed_rpm-=1;
//			ROS_INFO("\033[32m temp_speed_rpm = %f \033[0m",temp_speed_rpm);
			set_speed_rpm_=temp_speed_rpm;
			setSpeedRpm.val=temp_speed_rpm*65536;
			s_data[s_length++]=setSpeedRpm.buf[0];	//低位
			s_data[s_length++]=setSpeedRpm.buf[1];
			s_data[s_length++]=setSpeedRpm.buf[2];
			s_data[s_length++]=setSpeedRpm.buf[3];
			s_length+=2;
			s_data[s_length++]=0x55;
			chassis_client_.Send(s_data,13);
//			++send_CAN_counter_;
			ROS_INFO("sent \033[32mmotor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//			syschronic_frame();
//			ROS_INFO("sent \033[32m %ldth\033[0m CAN frame.",++send_CAN_counter_);
//			usleep(a_);	//10ms
		}
		else;
//		control_mode_=0;	
	}
	else if(control_mode_==7)	//constant accelerate for torque  
	{
		static float temp_torque_Nm=0;
		static uint8_t temp_Nm_flag=0;
		if(temp_Nm_flag==0)	
		{
			if(temp_torque_Nm==limited_torque_)temp_Nm_flag=1;	//A
			temp_torque_Nm+=0.1;
		}
		else if(temp_Nm_flag==1)
		{
			if(temp_torque_Nm==-limited_torque_)temp_Nm_flag=0;
			temp_torque_Nm-=0.1;
		}else;
		if(temp_Nm_flag==1||temp_Nm_flag==0)
		{
			set_torque_Nm_=temp_torque_Nm;
			setTorqueNm.val=temp_torque_Nm*65536;
			s_data[s_length++]=setTorqueNm.buf[0];	//低位
			s_data[s_length++]=setTorqueNm.buf[1];
			s_data[s_length++]=setTorqueNm.buf[2];
			s_data[s_length++]=setTorqueNm.buf[3];
			s_length+=2;
			s_data[s_length++]=0xaa;
			chassis_client_.Send(s_data,13);
//			++send_CAN_counter_;
//			printf_frame(s_data,13);
			ROS_INFO("sent \033[32m motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//			syschronic_frame();
//			ROS_INFO("sent \033[32m %ldth\033[0m CAN frame.",++send_CAN_counter_);
//			usleep(a_);	//10ms		
		}
	}
	else if(control_mode_==8)	//sine_speed_mode
	{
		static int64_t  cnt = 0;
		set_motor_rpm_[motor_id-1] = limited_speed_*sin(cnt*0.0010);
		setSpeedRpm.val=set_motor_rpm_[motor_id-1]*65536;
		
		s_data[s_length++]=setSpeedRpm.buf[0];	//低位
		s_data[s_length++]=setSpeedRpm.buf[1];
		s_data[s_length++]=setSpeedRpm.buf[2];
		s_data[s_length++]=setSpeedRpm.buf[3];	
		s_length+=2;
		s_data[s_length++]=0x55;
		chassis_client_.Send(s_data,13);
//		++send_CAN_counter_;
//		printf_frame(s_data,13);
		ROS_INFO("sent \033[32m motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
		cnt++;
		usleep(3000);	
	}
	else	//motor power off
	{
		s_length+=6;
		s_data[s_length++]=0x5a;
		chassis_client_.Send(s_data,13);
//		++send_CAN_counter_;
//		printf_frame(s_data,13);
		ROS_INFO("sent \033[32m power off motor_id = %d, %ldth\033[0m CAN frame.",motor_id,++send_CAN_counter_);
//		usleep(a_);	//
//		syschronic_frame();
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
//		control_mode_=0;
	}
	//s_length++;
	
//	ROS_INFO("wrote data");
}
// As third thread, process motor control and pub_msg
void DebugMotor::spin()
{
	int counter=0,byte_len=0;
	unsigned char feedback_data[200]={0};
	ros::Rate loop_rate(500);	//以hz设置循环频率
	pub_ = nh_.advertise<debug_motor::debugMotorData>("/motor_test/example", 10);	//缓冲队列为10发布话题
	while (nh_.ok())
    {
     	counter++;
		if(chassis_client_.Receive(feedback_data, 200, 0, &byte_len, 0)==1)
			analyse_CAN_data(feedback_data,byte_len);	
		else
			ROS_WARN_STREAM_ONCE(" Received wrong!");
//		if(counter%5==0)	//5ms
//		{
			
//		}
//		syschronic_frame();
//		ROS_INFO("control_mode_ = %d",control_mode_);
//		ROS_INFO("loop \033[32m%dth\033[0m times.",counter);	
		if(counter>1000) counter=0;
		ros::spinOnce();
		loop_rate.sleep();
//		usleep(1000);
	}
	stop();
}
}

