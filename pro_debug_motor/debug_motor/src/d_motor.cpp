#include <debug_motor/d_motor.h>


namespace debug_motor
{
DebugMotor::DebugMotor(ros::NodeHandle nh) : nh_(nh), shake_id_string_("hello"), dev_("/dev/ttyUSB0"), baud_rate_(115200), framerate_(1000),read_write_(0)	//construct
{
  	// Set up a dynamic reconfigure server.
  	// Do this before parameter server, else some of the parameter server values can be overwritten.
   	dynamic_reconfigure::Server<debug_motor::debugMotorConfig>::CallbackType cb;
  	cb = boost::bind(&DebugMotor::configCallback, this, _1, _2);
  	dr_srv_.setCallback(cb);
  	// Declare variables that can be modified by launch file or command line.
	double rate = 1.0;
	int time_out = 1000;
  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
	ros::NodeHandle pnh("~");
  	pnh.param("a", a_, a_);
  	pnh.param("b", b_, b_);
  	pnh.param("message", shake_id_string_, shake_id_string_);
  	pnh.param("rate", rate, rate);
  	pnh.param("enable", read_write_, read_write_);
  	
	pnh.param("dev", dev_, dev_);
	pnh.param("buad_rate", baud_rate_, baud_rate_);
	pnh.param("time_out", time_out, time_out);
  	// 开启串口模块

	try
	{
	    ros_ser_.setPort(dev_);		//Sets the serial port identifier ,like 'COM1','/dev/ttyUSB0'
	    ros_ser_.setBaudrate(baud_rate_);	//Sets the baudrate for the serial port
	    serial::Timeout to = serial::Timeout::simpleTimeout(time_out);	//Defines timeout struct
	    ros_ser_.setTimeout(to);	//Sets the timeout for reads and writes using the Timeout struct
	    ros_ser_.open();	//Opens the serial port as long as the port is set and the port isn't already open
	    ros_ser_.flushInput();	 //Flush only the input buffer
	}
	catch (serial::IOException& e)	//Captures exception
	{
	  	ROS_ERROR_STREAM("Unable to open port ");     
	}
	char ch = getchar();
	ROS_INFO("%.2X",ch);
	while(!ros_ser_.isOpen())	//Gets the open stauts of the serial port
	{
	  	ROS_INFO_STREAM("Serial Port did't open");
	  	if(getchar()==0x27)
	  	{
	  		break;
	  	}
	}
	ros_ser_.flushInput(); 
	ROS_INFO_STREAM("Serial Port opened");

  	// Create timer.
  	//timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &DebugMotor::timerCallback, this);
}
void DebugMotor::start()
{
	set_speed_rpm_=0;	//Initializes essential variables used to control motor with CAN protocol  
	set_torque_Nm_=0;
	control_mode_=0;
	ROS_INFO("START");
	std::thread analyse_frame_thread(&DebugMotor::analyse_data,this);	//Create a thread used to analyse received data
	std::thread send_frame_thread(&DebugMotor::send_data,this);	//Creates a thread used to send calibration frame 
	std::thread main_process(&DebugMotor::spin,this);	//Creates a thread as used to control motor with CAN protocol and publish key message periodically  
	analyse_frame_thread.join();	//Joins thread of data analysation 
	send_frame_thread.join();	//Joins thread of data send 
    main_process.join();	//Joins thread of control and publish
    stop();
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
	for(int index=0;index<size;index++)
	{
		printf("%.2x ",(unsigned char)frame[index]);	
	}
	printf("\n");
}
//Input an array and its size written by serial ,no output 
void DebugMotor::confirm_send(const unsigned char *frame,const int size)
{
	if(size>0)	//if size larger than zero,there were bytes have been send successfully
	{
		ROS_INFO("%d bytes written to the serial port",size);
		for(int index=0;index<size;index++)		//
		{
			printf("%.2x ",(unsigned char)frame[index]);
		}
		printf("\n");
	}
	else
	{
		ROS_WARN_STREAM("No byte written!");
	}
}
//As first thread applied to analyser of read bytes from the serial port 
void DebugMotor::analyse_data()
{
	unsigned char sum=0,r_length=0;
	unsigned char r_data[100]={0};	
  	while(nh_.ok())
  	{
  		received_data_.data = ros_ser_.read(ros_ser_.available());	//serial::available() returns the number of characters in the buffer.
  																	//serial::read() read a given amount of bytes from the serial port 
  		ROS_INFO("\033[32manalyse_data with size: \033[0m%d",static_cast<int>(received_data_.data.size()));
  		r_length=0;	//Clears essential parameter before use it 
  		sum=0;
	  	for(int index=0;index<received_data_.data.size();index++)
	  	{
	  		r_data[r_length++]=received_data_.data.at(index);	//Transform an string to an array
	  	}
	  	if(r_length>0)
	  	{
	  		printf_frame(r_data,r_length);
	  	}
//  	usleep(10000);	//unit is us
		sleep(1);	//unit is s 
  	}
}
//As second thread assigned as sender of calibration
void DebugMotor::send_data()
{
	while(nh_.ok())
	{
		if(send_params_)
		{
			switch(oper_object_)	//Judge objection calibrated candidate 
			{
				case 1:		//握手帧
				{	
					send_data_.data="";
					send_data_.data+="wzengqc";	
					ROS_INFO("\033[34msend shake cmd :\033[0m");
					send_shake_id_cmd(send_data_);
	//				oper_object_=0;
					break;
				}
				case 2:		//识别帧
				{
					send_data_.data="";
					send_data_.data+="Zyfsbmv";	
					ROS_INFO("\033[34msend identify cmd :\033[0m");
					send_shake_id_cmd(send_data_);
					break;
				}
				case 3:		//保存指令
				{
					unsigned char s_data[8]={0};
					int size=0;
					ROS_INFO("\033[34msend save cmd :\033[0m");
					s_data[0]=0x53;
					s_data[7]=0x53;
					size = ros_ser_.write(s_data,8);
					confirm_send(s_data,size);
//					ROS_INFO("wrote");
					break;
				}
				case 4:		//通信模式切换指令
				{
					ROS_INFO("\033[34msend alter communication mode cmd :\033[0m");
					send_data_.data="";
					send_data_.data.push_back(0xa5);
					send_data_.data+="change";
					send_shake_id_cmd(send_data_);
					break;
				}
				case 5:		//operate bit
				{
					ROS_INFO("\033[34msend bit cmd\033[0m");
					send_bit_cmd(page_index_,pos_index_,set_val_,nth_bit_);
					break;
				}
				case 6:		//operate byte
				{
					ROS_INFO("\033[34msend byte cmd\033[0m");
					send_nbyte_cmd(0xaa,page_index_,pos_index_,set_val_);
					break;
				}
				case 7:		//operate word
				{
					ROS_INFO("\033[34msend word cmd\033[0m");
					send_nbyte_cmd(0xcc,page_index_,pos_index_,set_val_);
					break;
				}
				case 8:		//operate long word
				{
					ROS_INFO("\033[34msend long word cmd\033[0m");
					send_nbyte_cmd(0x33,page_index_,pos_index_,set_val_);
					break;
				}
				default:
				{
						ROS_INFO("No write or read operation!");
				}
			}
		}
//		usleep(1000000);	//us
		sleep(1);
	}
}
//Input string ,no output,sending an array of shake hand frame or an array of identification frame  
void DebugMotor::send_shake_id_cmd(std_msgs::String str)
{
	unsigned char sum=0,s_length=0;
	int size=0;;
	unsigned char s_data[20]={0};	
//	ROS_INFO("str = %s str.data.size() = %d",str.data.c_str(),static_cast<int>(str.data.size()));
	for(int index=0;index < str.data.size();index++)
	{
		sum+=str.data.at(index);
		s_data[s_length++]=str.data.at(index);
	}
	s_data[s_length++]=sum;
//	ROS_INFO("wrote data");
	size = ros_ser_.write(s_data,s_length);
	confirm_send(s_data,size);
	
}
//Input calibration parameters,no output ,sending an array of bit operation frame
void DebugMotor::send_bit_cmd(int page_index,int pos_index,int32_t set_val,int nth_bit)
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	unsigned char sum=0;
	int size=0;
	if(read_write_==1)
	{
		ROS_INFO("\033[32msend_read_cmd:\033[0m");
		s_data[s_length++]=0x55;
		s_data[s_length++]=0x80|pos_index;	
	}
	else if(read_write_==2)
	{
		ROS_INFO("\033[32msend_write_data:\033[0m");
		s_data[s_length++]=0x55;
		s_data[s_length++]=0x00|pos_index;
	}
	else;
	if(read_write_==1|read_write_==2)
	{
		s_data[s_length++]=page_index;
		//数据位
		s_data[s_length++]=0x01<<nth_bit;
		s_data[s_length++]=0x00;
		s_data[s_length++]=0x00;
		s_data[s_length++]=static_cast<unsigned char>(set_val)&0xff;
		crc(s_data,s_length,&sum);	//Gets checksum
		s_data[s_length++]=sum;
		size = ros_ser_.write(s_data,s_length);
		confirm_send(s_data,size);
//		ROS_INFO("wrote data");
	}
	else
	{
		ROS_INFO("No write and read");
	}
}
//Input calibration parameters,no output ,sending an array of n bytes operation frame
void DebugMotor::send_nbyte_cmd(unsigned char operated_object,int page_index,int pos_index,int32_t set_val)
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	unsigned char sum=0;
	int size=0;
	if(read_write_==1)
	{
		ROS_INFO("\033[32msend_read_cmd:\033[0m");
		s_data[s_length++]=operated_object;
		s_data[s_length++]=0x00+pos_index;
	}
	else if(read_write_==2)
	{
		ROS_INFO("\033[32msend_write_data:\033[0m");
		s_data[s_length++]=operated_object;
		s_data[s_length++]=0x80+pos_index;
	}
	else;
	if(read_write_==1|read_write_==2)
	{
		s_data[s_length++]=page_index;
		//数据位
		s_data[s_length++]=(set_val>>24)&0xff;
		s_data[s_length++]=(set_val>>16)&0xff;
		s_data[s_length++]=(set_val>>8)&0xff;
		s_data[s_length++]=static_cast<unsigned char>(set_val)&0xff;
		
		for(int counter=0;counter<s_length;counter++)
		{
			sum+=s_data[counter];
		}
		s_data[s_length++]=sum;
		
		size = ros_ser_.write(s_data,s_length);
		confirm_send(s_data,size);
//		ROS_INFO("wrote data");
	}
	else
	{
		ROS_INFO("No write and read");
	}	
}
void DebugMotor::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
//  debug_motor::debugMotorData msg;
//  msg.message = shake_id_string_;
//  msg.a = a_;
//  msg.b = b_;

//  pub_.publish(msg);
}
//Input configured parameters,no output ,updating parameters of calibration as well as that of motor control 
void DebugMotor::configCallback(debug_motor::debugMotorConfig &config, uint32_t level __attribute__((unused)))
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  oper_object_=config.operate_object;
  read_write_=config.read_write;
  page_index_=config.page_index;
  pos_index_=config.pos_index;
  set_val_=config.set_val;
  nth_bit_=config.nth_bit;
  send_params_=config.send_params;
  //parameters used to motor control with CAN protocol 
  set_speed_rpm_=config.set_speed_rpm;
  set_torque_Nm_=config.set_torque_Nm;
  control_mode_=config.control_mode;
  motor_id_=config.motor_id;
  send_CAN_=config.send_CAN;
  
}
//publish necessary variables
void DebugMotor::pub_msg()
{
	debug_motor::debugMotorData msg;
	msg.message = shake_id_string_;
	msg.a = a_;
	msg.b = b_;
	pub_.publish(msg);
//	ROS_INFO("published msg: message_ = %s,a = %d,b = %d",shake_id_string_.c_str(),a_,b_);
}
//motor control 
void DebugMotor::control_motor()
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	int size=0;
	s_data[s_length++]=0x44;
	s_length+=3;
	s_data[s_length++]=motor_id_;
	if(control_mode_==1)	//速度控制模式
	{
		s_data[s_length++]=(set_speed_rpm_>>8)&0xff;
		s_data[s_length++]=(unsigned char)set_speed_rpm_&0xff;
		s_length+=4;
		s_data[s_length++]=0x55;
	}
	else if(control_mode_==2)
	{
		s_length+=2;
		s_data[s_length++]=(set_torque_Nm_>>8)&0xff;
		s_data[s_length++]=(unsigned char)set_torque_Nm_&0xff;
		s_length+=2;
		s_data[s_length++]=0xaa;
	}
	else;
	s_length++;
	size = ros_ser_.write(s_data,s_length);
	confirm_send(s_data,size);
//	ROS_INFO("wrote data");
}
// As third thread, process motor control and pub_msg
void DebugMotor::spin()
{
	int counter=0;
	
	ros::Rate loop_rate(framerate_);	//以hz设置循环频率
	pub_ = nh_.advertise<debug_motor::debugMotorData>("example", 10);	//缓冲队列为10发布话题
	while (nh_.ok())
    {
     	counter++;
		if(counter>1000)
		{
		  	counter=0;
			pub_msg();
		}
		if(send_CAN_)
		{
			control_motor();
		}
//		ROS_INFO("spin : %d",counter);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

}
