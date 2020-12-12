#include <ws_suspension/ws_suspension.h>
namespace ws_suspension
{
WSSuspension::WSSuspension(ros::NodeHandle nh) : nh_(nh)	//construct
{
  	// Declare variables that can be modified by launch file or command line.
	double rate = 1.0;
	double framerate = 1000.0;
	ros::NodeHandle pnh("~");
  	pnh.param("rate", rate, rate);
  	pnh.param("framerate", framerate_, framerate);
	
	suspension_client_.create_client("192.168.1.41",5001,0);
  	// Create timer.
  	state_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &WSSuspension::statePubTimerCallback, this);
	control_timer_ = nh_.createTimer(ros::Duration(1.0/100.), &WSSuspension::contorlTimerCallback, this);
}
void WSSuspension::init_suspension()
{
	for(int idx=0;idx<6;idx++)
	{
		set_angle_[idx]=0;
	}
	while(suspension_client_.Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
	control_mode_=0;
	ROS_INFO("START");
	current_time_ = ros::Time::now();
  	last_time_ = ros::Time::now();
	spin();	
}
void WSSuspension::stop()
{
  	 state_pub_.shutdown();
}
//Input an array and its length,no output
void WSSuspension::printf_frame(const unsigned char *frame,const int size)	
{
	for(int index=0;index<size;index++)
	{
		printf("%.2x ",(unsigned char)frame[index]);	
	}
	printf("\n");
}

void WSSuspension::analyse_data(unsigned char *frame,int size)
{
	//08 00 00 00 72 00 00 00 00 00 00 00 00 
//	std::lock_guard<mutex> lock(m_mutex_);
	
	static int32_t get_angle[6]={0},get_pressure[6]={0};
	static int16_t last_get_angle[6]={0};
	static int32_t short_frame=0;
	unsigned char one_frame_flag =0;
	unsigned char brake_frame_flag=0;
	unsigned char idx_self_add_flag =0;
	bool emergency_flag=0;
	static int16_t get_origin_angle[6][10]={0};
	int64_t temp_angle=0;
	union int16_uchar temp_int16;
	if(size<4)return;
	for(int16_t idx = 0;idx<size;)
	{
		if(frame[idx+0]==0xAE&&frame[idx+1]==0xEA)
		{
			if(frame[idx+2]==0x01&&frame[idx+3]==0x10)	//angle 
			{
				for(int16_t arr_idx=0;arr_idx<6;)
				{
			  		get_angle[arr_idx]=(int16_t)frame[idx+arr_idx+4]|(frame[idx+arr_idx+5]<<16);
			  		arr_idx=arr_idx+2;
				}
				one_frame_flag=1;
				if(get_angle_counter_<=10)++get_angle_counter_;
//				ROS_INFO("get angle ");
			}
			else if(frame[idx+2]==0x02&&frame[idx+3]==0x20)	//pressure
			{
				for(int16_t arr_idx=0;arr_idx<6;)
				{
			  		get_pressure[arr_idx]=(int16_t)frame[idx+arr_idx+4]|(frame[idx+arr_idx+5]<<16);
			  		arr_idx=arr_idx+2;
				}
			  	one_frame_flag = 1;
//			  	ROS_INFO("get pressure ");
			}
			else if(frame[idx+2]==0x03&&frame[idx+3]==0x30)	//emergency stop
			{
				emergency_stop_enable_=static_cast<bool>(frame[idx+4]);
				
			  	emergency_flag=1;
			}
			else if(frame[idx+2]==0x04&&frame[idx+3]==0x40)	//brake position limitation
			{
//				brake_pos_limit_[0] =static_cast<bool>(frame[idx+4]);
//				brake_pos_limit_[1]	= static_cast<bool>(frame[idx+6]);
				brake_pos_limit_[0] =frame[idx+4];
				brake_pos_limit_[1]	=frame[idx+6];
				brake_frame_flag = 1;
			}
			else;
		}
		if(one_frame_flag==1)
		{
			one_frame_flag=0;idx=idx+16;
//			ROS_INFO("idx = \033[32m %d \033[0m ",idx);	
			ROS_INFO("got \033[32m %ldth \033[0m frame with length 16 ",++get_counter_);	
//			printf_frame(frame+idx,16);
			idx_self_add_flag=1;
		}
//		else {ROS_INFO("idx = \033[32m %d \033[0m ",idx);	}
		if(brake_frame_flag==1)
		{
			brake_frame_flag=0;idx=idx+8;
			idx_self_add_flag=1;
			ROS_INFO("got \033[32m %ldth brake pos limit \033[0m frame with length 8",++get_counter_);	
//			printf_frame(frame+idx,8);
		}
		if(emergency_flag==1)	//
		{
			emergency_flag=0;idx=idx+6;
			idx_self_add_flag=1;
			ROS_INFO("got \033[32m %ldth emergency stop \033[0m frame with length 6 ",++get_counter_);
//			printf_frame(frame+idx,6);
		}
		if(!idx_self_add_flag){idx=idx+1;}
		//original angle 
		if(get_angle_counter_<10){get_origin_angle[idx][get_counter_]=get_angle[idx];}	
		else if(get_angle_counter_==10)
		{
			for(int i=0;i<6;i++)
			{	
				for(int j=0;j<10;j++)
				{
					temp_angle=temp_angle+get_origin_angle[i][j];
				}
				get_angle_origin_[i]=temp_angle/10.;
				temp_angle=0;
			}
		}
		else;
	}
	m_mutex_.lock();
	for(int idx=0;idx<6;idx++)
	{
		if(abs(get_angle[idx]-last_get_angle[idx])<1000)	//sign also should be considered 
		{
			get_angle_[idx]=get_angle[idx];
			last_get_angle[idx]=get_angle[idx];
		}
		else get_angle_[idx]=last_get_angle[idx];
		get_pressure_[idx]=get_pressure[idx];		//65
	}	
	m_mutex_.unlock();
	
//	if(size<13)ROS_INFO("got \033[33m %dth short frame",++short_frame);
//	printf_frame(frame,size);
}
//
void WSSuspension::send_suspension_data(unsigned char power_state)
{
	ROS_INFO("CONTORL_plc");
//	if(get_angle_counter_<=10) return;
//	ROS_INFO("control_plc start");	//到control_plc end 用时 0.1ms
	unsigned char s_data[50]={0};
	unsigned char s_length=0;
	int size=0;
	union int16_uchar setCurrent[6];
	union int16_uchar setAngle[6];
	
	m_mutex_.lock();
	if(plc_control_mode_==2||plc_control_mode_==1)	//依据角度
	{
		for(int suspension_id=0;suspension_id<6;suspension_id++)
		{
			setCurrent[suspension_id].val = set_current_[suspension_id];
			setAngle[suspension_id].val = set_angle_[suspension_id]+get_angle_origin_[suspension_id];	//初始值待确定
	//		setAngle[suspension_id].val = set_angle_[suspension_id]+get_angle_origin_[suspension_id];	//
		}
	}
	else if(plc_control_mode_==3)	//依据位置
	{
		for(int suspension_id=0;suspension_id<6;suspension_id++)
		{
			setCurrent[suspension_id].val = set_current_[suspension_id-1];
			
	//		setAngle[suspension_id].val = set_angle_[suspension_id]+get_angle_origin_[suspension_id];	//
		}
		setAngle[0].val = 15278.87*pos2angle(10,10,set_pos_[0]+get_pos_origin_[0]);	//初始值待确定 16000/60*180/pi ~ 15278.87
		setAngle[1].val = 15278.87*pos2angle(10,10,set_pos_[1]+get_pos_origin_[1]);	//初始值待确定 16000/60 ~ 267.67
		setAngle[2].val = set_pos_[2]+get_pos_origin_[2];	//初始值待确定 计算关系待确定
		setAngle[3].val = set_pos_[3]+get_pos_origin_[3];	//初始值待确定
		setAngle[4].val = 15278.87*pos2angle(10,10,set_pos_[4]+get_pos_origin_[4]);	//初始值待确定 16000/60 ~ 267.67
		setAngle[5].val = 15278.87*pos2angle(10,10,set_pos_[5]+get_pos_origin_[5]);	//初始值待确定 16000/60 ~ 267.67	
	}	
	m_mutex_.unlock();
	s_data[s_length++]=0xEA;
	s_data[s_length++]=0xAE;
	if(plc_control_mode_==2||plc_control_mode_==1)	//依据角度控制
	{
		if(plc_control_mode_==2)
		{
			s_data[s_length++]=0x01;
			s_data[s_length++]=0x01;
		}
		else
		{
			s_data[s_length++]=0x06;
			s_data[s_length++]=0x06;
		}
		
		for(int idx=0;idx<6;idx++)
		{
			s_data[s_length++]=setAngle[idx].buf[0];	//低位
			s_data[s_length++]=setAngle[idx].buf[1];
		}
		for(int idx=0;idx<6;idx++)
		{
			s_data[s_length++]=setCurrent[idx].buf[0];	//低位
			s_data[s_length++]=setCurrent[idx].buf[1];
		}
		suspension_client_.Send(s_data,s_length);
//		usleep(a_);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32m pos cmd , %ldth\033[0m frame.",++send_counter_);
//		control_mode_=0;
	}
	else if(plc_control_mode_==3)	//依据位置控制
	{
		s_data[s_length++]=0x02;
		s_data[s_length++]=0x02;
		for(int idx=0;idx<6;idx++)
		{
			s_data[s_length++]=setAngle[idx].buf[0];	//低位
			s_data[s_length++]=setAngle[idx].buf[1];
		}
		for(int idx=0;idx<6;idx++)
		{
			s_data[s_length++]=setCurrent[idx].buf[0];	//低位
			s_data[s_length++]=setCurrent[idx].buf[1];
		}
		
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32mpos cmd %ldth\033[0m frame.",++send_counter_);
//		control_mode_=0;
	}
	else if(plc_control_mode_==4)	//依据压力控制
	{
		s_data[s_length++]=0x03;
		s_data[s_length++]=0x03;
		for(int idx=0;idx<6;idx++)
		{
			s_data[s_length++]=setAngle[idx].buf[0];	//低位
			s_data[s_length++]=setAngle[idx].buf[1];
		}
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32mtorque cmd %ldth\033[0m frame.",++send_counter_);
	
	}
	else if(plc_control_mode_==5)	// motor control power on
	{
		s_data[s_length++]=0x04;
		s_data[s_length++]=0x04;
		s_data[s_length++]=0x01;
		s_data[s_length++]=0x00;
		
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32m control power on %ldth\033[0m frame.",++send_counter_);
//		usleep(a_);
//		ROS_INFO("sent \033[32m%ldth\033[0m CAN frame.",++send_CAN_counter_);
	}
	else if(plc_control_mode_==6)	// motor control power off
	{
		s_data[s_length++]=0x04;
		s_data[s_length++]=0x04;
		s_data[s_length++]=0x00;
		s_data[s_length++]=0x00;
		
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32m control power off %ldth\033[0m frame.",++send_counter_);
	
	}	
	else if(plc_control_mode_==7)	//motor main power on
	{
		s_data[s_length++]=0x05;
		s_data[s_length++]=0x05;
		s_data[s_length++]=0x01;
		s_data[s_length++]=0x00;
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32m main power on %ldth\033[0m frame.",++send_counter_);
	}
	else if(plc_control_mode_==8)	//motor main power off
	{
		s_data[s_length++]=0x05;
		s_data[s_length++]=0x05;
		s_data[s_length++]=0x00;
		s_data[s_length++]=0x00;
		suspension_client_.Send(s_data,s_length);
		printf_frame(s_data,s_length);
		ROS_INFO("sent \033[32m main power off %ldth\033[0m frame.",++send_counter_);
	}
	else;
}

void WSSuspension::statePubTimerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
//  pub_.publish(msg);
	ws_motor::wsSuspensionData msg;
	m_mutex_.lock();
	std::vector<int16_t> temp_set_angle(set_angle_,set_angle_+6);
	std::vector<int16_t> temp_set_current(set_current_,set_current_+6);
	std::vector<int16_t> temp_set_pressure(set_pressure_,set_pressure_+6);
	std::vector<int16_t> temp_get_angle(get_angle_,get_angle_+6);
	std::vector<int16_t> temp_get_pressure(get_pressure_,get_pressure_+6);
	std::vector<unsigned char> temp_brake_pl(brake_pos_limit_,brake_pos_limit_+2);
	m_mutex_.unlock();
	
	msg.send_counter=send_counter_;
	msg.get_counter=get_counter_;
	msg.set_angle = temp_set_angle;
	msg.set_angle.resize(6);
	msg.set_current =temp_set_current;
	msg.set_current.resize(6);
	msg.set_pressure=temp_set_pressure;
	msg.set_pressure.resize(6);	//设定容器尺寸
	msg.get_angle=temp_get_angle;
	msg.get_angle.resize(6);	//设定容器尺寸
	msg.get_pressure=temp_get_pressure;
	msg.get_pressure.resize(6);	//设定容器尺
	msg.emergency_stop_enable=emergency_stop_enable_;
	msg.motor_control_power = motor_control_power_;
	msg.motor_main_power = motor_main_power_;
	msg.brake_pos_limit=temp_brake_pl;
	msg.brake_pos_limit.resize(2);

	state_pub_.publish(msg);
//	ROS_INFO("published msg");
	
	//	ROS_INFO("published msg: message_ = %s,a = %d,b = %d",shake_id_string_.c_str(),a_,b_);
}

void WSSuspension::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	static int count=0;
	control_mode_=msg->linear.y;
	m_mutex_.lock();	//mode
	if(control_mode_==1)
	{
		set_angle_[0] = msg->linear.y; //suspension 1
		set_angle_[2] = msg->linear.y; //suspension 3
		set_angle_[4] = msg->linear.y; //suspension 5
	
		set_angle_[1] = msg->linear.y; //suspension 2
		set_angle_[3] = msg->linear.y; //suspension 4
		set_angle_[5] = msg->linear.y; //suspension 6
	}
	m_mutex_.unlock();
	ROS_INFO("I got twist.linear.x = %lf , twist.angular.z = %lf",msg->linear.x, msg->angular.z);
}

void WSSuspension::contorlTimerCallback(const ros::TimerEvent &event)
{
	//	ROS_INFO(" control_callback");
	if(send_confirm_)
	{	
		send_suspension_data(0);		
		send_confirm_=0;				
	}
}
//, process motor control and pub_msg
void WSSuspension::spin()
{
	int counter=0,byte_len=0;
	unsigned char feedback_data[20]={0};
	ros::Rate loop_rate(framerate_);	//以hz设置循环频率
	state_pub_ = nh_.advertise<ws_motor::wsSuspensionData>("/wsSuspension/suspensionState", 10);	//缓冲队列为10发布话题
	while (nh_.ok())
    {
     	counter++;
//		if(counter%10==0)
//		{
//		 	pub_msg();
//		}
		if(suspension_client_.Receive(feedback_data, 100, 0, &byte_len, 0)==1)
			analyse_data(feedback_data,byte_len);	
		 else
			ROS_WARN_STREAM_ONCE(" Received wrong!");
//		if(counter%8==0)
//		{
//			control_motor();	//tcp-CAN test constant acceleration
//		}
		if(counter>1000) counter=0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	stop();
}
}

