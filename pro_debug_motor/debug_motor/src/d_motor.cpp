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
	
	for(int idx=0;idx<6;idx++)
	{
		motor_rpm_[idx]=0;
		motor_Nm_[idx]=0;
		motor_tempoc_[idx]=0;
		motor_break_code_[idx]=0;
		motor_odom_er_[idx]=0;
	}
	for(int idx=100;idx<100;idx++)
		for(int sub_idx=13;sub_idx<8;sub_idx++)
			params_[idx][sub_idx]=0;
	params_index_=0;
	send_CAN_counter_=0;
	get_CAN_counter_=0;
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
//	char ch = getchar();
//	ROS_INFO("%.2X",ch);
	while(!ros_ser_.isOpen())	//Gets the open stauts of the serial port
	{
	  	ROS_INFO_STREAM("Serial Port did't open");
//	  	if(getchar()==0x27)
//	  	{
//	  		break;
//	  	}
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
//	std::thread analyse_frame_thread(&DebugMotor::analyse_data,this);	//Create a thread used to analyse received data
	std::thread send_frame_thread(&DebugMotor::send_data,this);	//Creates a thread used to send calibration frame 
	std::thread main_process(&DebugMotor::spin,this);	//Creates a thread as used to control motor with CAN protocol and publish key message periodically  
//	analyse_frame_thread.join();	//Joins thread of data analysation 
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
bool DebugMotor::confirm_send(const unsigned char *frame,const int size)
{
	if(size>0)	//if size larger than zero,there were bytes have been send successfully
	{
		ROS_INFO("%d bytes written to the serial port : ",size);
		for(int index=0;index<size;index++)		//
		{
			printf("%.2x ",(unsigned char)frame[index]);
		}
		printf("\n");
		return 1;
	}
	else
	{
		ROS_WARN_STREAM("No byte written!");
		return 0;
	}
}
void DebugMotor::analyse_CAN_data(unsigned char *frame,int size)
{
	//08 00 00 00 72 00 00 00 00 00 00 00 00 
//	std::lock_guard<mutex> lock(m_mutex);
	int32_t motor_rpm[6]={0},motor_Nm[6]={0},motor_break_code[6]={0};
	int16_t motor_odom_er[6]={0},motor_tempoc[6]={0};
	ROS_INFO("got \033[32m %ldth CAN\033[0m frame>>>>>>>>",get_CAN_counter_++);
	if(frame[0]==0x72)
	{
	  	motor_rpm[0] = frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[0] =frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
	  	
	}
	else if(frame[0]==0x73)
	{
		motor_rpm[1] =frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[1] =frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);
	}
	else if(frame[0]==0x74)
	{
	 	motor_rpm[2] = frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[2] = frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24); 		
	}
	else if(frame[0]==0x75)
	{
	 	motor_rpm[3] = frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[3] = frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24); 			
	}
	else if(frame[0]==0x76)
	{
	 	motor_rpm[4] = frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[4] = frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);		
	}
	else if(frame[0]==0x77)
	{
	 	motor_rpm[5] = frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
	  	motor_Nm[5] =frame[5]|(frame[6]<<8)|(frame[7]<<16)|(frame[8]<<24);		
	}
	else if(frame[0]==0x52)
	{
		motor_break_code[0]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[0]=frame[5]|(frame[6]<<8);
		motor_odom_er[0]=frame[7]|(frame[8]<<8);	
	}
	else if(frame[0]==0x53)
	{
		motor_break_code[1]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[1]=frame[5]|(frame[6]<<8);
		motor_odom_er[1]=frame[7]|(frame[8]<<8);	
	}
	else if(frame[0]==0x54)
	{
		motor_break_code[2]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[2]=frame[5]|(frame[6]<<8);
		motor_odom_er[2]=frame[7]|(frame[8]<<8);	
	}
	else if(frame[0]==0x55)
	{
		motor_break_code[3]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[3]=frame[5]|(frame[6]<<8);
		motor_odom_er[3]=frame[7]|(frame[8]<<8);	
	}
	else if(frame[0]==0x56)
	{
		motor_break_code[4]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[4]=frame[5]|(frame[6]<<8);
		motor_odom_er[4]=frame[7]|(frame[8]<<8);	
	}
	else if(frame[0]==0x57)
	{
		motor_break_code[5]= frame[1]|(frame[2]<<8)|(frame[3]<<16)|(frame[4]<<24);
		motor_tempoc[5]=frame[5]|(frame[6]<<8);
		motor_odom_er[5]=frame[7]|(frame[8]<<8);	
	}
	else;
	m_mutex.lock();
	for(int idx=0;idx<6;idx++)
	{
		motor_rpm_[idx]=motor_rpm[idx]>>16;
		motor_Nm_[idx]=motor_Nm[idx]>>16;
		motor_break_code_[idx]=motor_break_code[idx];
		motor_tempoc_[idx]=motor_tempoc[idx]+50;
		motor_odom_er_[idx]=motor_odom_er[idx]/32;	//32为电机每转一圈的电圈数
	}	
	m_mutex.unlock();
	printf_frame(frame,size);
}

//As first thread applied to analyser of read bytes from the serial port 
char DebugMotor::analyse_data()
{
	unsigned char sum=0,r_length=0;
	unsigned char r_data[100]={0};	
//  	while(nh_.ok())
//  	{
  		received_data_.data = ros_ser_.read(ros_ser_.available());	//serial::available() returns the number of characters in the buffer.																	//serial::read() read a given amount of bytes from the serial port 
//  		ROS_INFO("\033[32manalyse_data with size: \033[0m%d",static_cast<int>(received_data_.data.size()));
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
	  	for(int idx=0;idx<r_length;idx++)
	  	{
	  		if(r_data[idx]==0x55)
	  		{
	  			ROS_INFO("got command with respect to \033[32mbit\033[0m value operation");	
	  			if(oper_object_==9)	//batch read
	  			{
	  				memcpy(*(params_+params_index_),r_data,r_length);	//存储batch read 后的数据
	  				params_index_++;
	  				ROS_INFO("[\033[32m%dth response in terms of batch write\033[0m]",params_index_);
	  			
	  			}	
	  			return 1;
	  		}
	  		else if(r_data[idx]==0xaa)
	  		{
	  			ROS_INFO("got command with respect to \033[32mbyte\033[0m value operation");
	  			if(oper_object_==9)	//batch read
	  			{
	  				memcpy(*(params_+params_index_),r_data,r_length);	//存储batch read 后的数据
	  				params_index_++;
	  				ROS_INFO("[\033[32m%dth response in terms of batch write\033[0m]",params_index_);
	  			}
	  			return 2;
	  		}
	  		else if(r_data[idx]==0xcc)
	  		{
	  			ROS_INFO("got command with respect to \033[32mword\033[0m value operation");
	  			if(oper_object_==9)	//batch read
	  			{
	  				memcpy(*(params_+params_index_),r_data,r_length);	//存储batch read 后的数据
	  				params_index_++;
	  				ROS_INFO("[\033[32m%dth response in terms of batch write\033[0m]",params_index_);
	  			}
	  		
	  			return 3;
	  		}
	  		else if(r_data[idx]==0x33)
	  		{
	  			ROS_INFO("got command with respect to \033[32mlong word\033[0m value operation");
	  			return 4;
	  		}
	  		//08 00 00 00 72 00 00 00 00 00 00 00 00 
	  		else if(((r_data[idx]&0xf0)==0x70)||((r_data[idx]&0xf0)==0x50))
	  		{
	  			analyse_CAN_data(r_data+idx,9);
	  			return 5;
	  		}
	  		else return -1;
	  	}
//  	usleep(10000);	//unit is us
//		sleep(1);	//unit is s 
//  	}
}
//As second thread assigned as sender of calibration
void DebugMotor::send_data()
{
	char batch_send_flag=0;
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
					oper_object_=0;
					break;
				}
				case 2:		//识别帧
				{
					send_data_.data="";
					send_data_.data+="Zyfsbmv";	
					ROS_INFO("\033[34msend identify cmd :\033[0m");
					send_shake_id_cmd(send_data_);
					oper_object_=0;
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
					oper_object_=0;
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
					oper_object_=0;
					break;
				}
				case 5:		//operate bit
				{
					ROS_INFO("\033[34msend bit cmd\033[0m");
					send_bit_cmd(page_index_,pos_index_,set_val_,nth_bit_);
					oper_object_=0;
					break;
				}
				case 6:		//operate byte
				{
					ROS_INFO("\033[34msend byte cmd\033[0m");
					send_nbyte_cmd(0xaa,page_index_,pos_index_,set_val_);
					oper_object_=0;
					break;
				}
				case 7:		//operate word
				{
					ROS_INFO("\033[34msend word cmd\033[0m");
					send_nbyte_cmd(0xcc,page_index_,pos_index_,set_val_);
					oper_object_=0;
					break;
				}
				case 8:		//operate long word
				{
					ROS_INFO("\033[34msend long word cmd\033[0m");
					send_nbyte_cmd(0x33,page_index_,pos_index_,set_val_);
					oper_object_=0;
					break;
				}
				case 9:
				{
					ROS_INFO("\033[34msend batch read cmd\033[0m");
					batch_send();
					oper_object_=0;	
					batch_send_flag=1;
					break;
				}
				default:
				{
					ROS_INFO("No write or read operation!");
				}
			}
			if(!batch_send_flag)
				analyse_data();
			else
				batch_send_flag=0;
				
		}//end if(send_params_)
//		usleep(1000000);	//us
		sleep(1);
	}
}
void DebugMotor::send_bit_with_response(int page_index,int pos_index,int32_t set_val,int nth_bit)
{
	for(int send_counter=0;send_counter<3;send_counter++)
	{
		send_bit_cmd(page_index,pos_index,0,nth_bit);	//0byte index
		usleep(30000);	//2ms
		if(analyse_data()==1)
		{
			ROS_INFO("got command \033[32m page_index %.2x pos_index %d nth_bit %d\033[0m value operation");
			break;
		}
		if(send_counter==2)ROS_INFO("send bit_idx page 0 byte 0 failure.");
	}
}
void DebugMotor::send_nbyte_with_response(unsigned char operated_object,int page_index,int pos_index,int32_t set_val,char target)
{
	int max_send_times=3;
	for(int send_counter=0;send_counter<max_send_times;send_counter++)
	{
		send_nbyte_cmd(operated_object,page_index,pos_index,0);
		usleep(20000);	//2ms
		if(analyse_data()==target)
		{	
			ROS_INFO("got command \033[32m operated_object %.2x  page_index %d pos_index %d target %d\033[0m value operation");
			break;
		}
		if(send_counter==2)ROS_INFO("send bit_idx page 0 byte 0 failure.");
	}
}
void DebugMotor::batch_send()
{
	char break_flag=0;
	params_index_=0;
	ROS_INFO("\033[34mbatch send command>>>>>>>>\033[0m");
	if(read_write_==1)	//batch_read
	{
		for(int bit_idx=0;bit_idx<8;bit_idx++)	//批量读/写取位		//32
		{
			for(int byte_num=0;byte_num<32;byte_num++)
				send_bit_with_response(0,byte_num,0,bit_idx);
		}
		ROS_INFO("\033[34mbatch send by bit ko >>>>>>>>\033[0m");
		for(int page_idx=1;page_idx<5;page_idx++)
			for(int byte_idx=0;byte_idx<32;byte_idx++)	//批量读/写字节	//128
			{
	//			send_nbyte_cmd(0xaa,page_index_,pos_index_,set_val_);
//				if(byte_idx!=3)
				send_nbyte_with_response(0xaa,page_idx,byte_idx,0,2);
			}
		ROS_INFO("\033[34mbatch send by byte ko >>>>>>>>\033[0m");
		for(int page_idx=5;page_idx<13;page_idx++)	//批量读/写字			//128*2=256
		{
			for(int word_idx=0;word_idx<16;word_idx++)
			{
	//			send_nbyte_cmd(0xcc,page_index_,pos_index_,set_val_);
				//send_nbyte_cmd(0xcc,page_idx,word_idx,0);
				send_nbyte_with_response(0xcc,page_idx,word_idx,0,3);
//				if(page_idx==7&&word_idx==13)
//				{
//					break_flag=1;
//					break;
//				}
			}
//			if(break_flag==1)
//				break;
		}
		ROS_INFO("\033[34mbatch send by word ko >>>>>>>>\033[0m");
		for(int page_idx=13;page_idx<32;page_idx++)
			for(int lword_idx=0;lword_idx<8;lword_idx++)	//批量读/写长字  156*4
			{
	//		send_nbyte_cmd(0x33,13,lword_idx,0);	
				send_nbyte_with_response(0x33,page_idx,lword_idx,0,4);
//				if(lword_idx<3)
					//send_nbyte_cmd(0x33,14,lword_idx,0);
//					send_nbyte_with_response(0x33,page_idx,lword_idx,0,4);
			}
		ROS_INFO("\033[34mbatch send by long word ko >>>>>>>>\033[0m");
		outfile.open("./data.txt");		//
		for(int idx=0;idx<1024;idx++)
			for(int sub_idx=0;sub_idx<8;sub_idx++)
				outfile<<params_[idx][sub_idx]<<" ";
		outfile.close();
	}
	else if(read_write_==2)		//batch write
	{
		ROS_INFO("ready to batch read>>>>(Y/N)");
		char ch= getchar();
		for(int idx=0;idx<params_index_;idx++)		//update 
		{
			unsigned char sum=0;
			if(params_index_<1024)
			{			
				if(ch==0x59||ch==0x79)
				{
					params_[idx][1]=params_[idx][1]+0x80;	//
//					params_[idx][7]=params_[idx][7]+0x80;
//					params_[436][6]=0x44+motor_id_;
//					crc(params_[436],8,&sum);	//Gets checksum
//					params_[436][7]=sum;
//				
//					params_[437][6]=0x71+motor_id_;
//					crc(params_[437],8,&sum);	//Gets checksum
//					params_[437][7]=sum;
//				
//					params_[437][6]=0x51+motor_id_;
//					crc(params_[437],8,&sum);	//Gets checksum
//					params_[437][7]=sum;
				}
			}
		}
//		infile.open("./data.txt");
//		for(int idx=0;idx<1024;idx++)
//			for(int sub_idx=0;sub_idx<8;sub_idx++)
//				infile>>params_[idx][sub_idx];
//		infile.close();
		for(int idx=0;idx<params_index_;idx++)
		{
			ros_ser_.write(*(params_+idx),8);
			usleep(20000);		//2ms
		}	
				//confirm_send(const unsigned char *frame,const int size);
	}
	else;
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
	if(confirm_send(s_data,size))
		ROS_INFO_STREAM("string "<<str<< " was sent to the driver");
	
}
//Input calibration parameters,no output ,sending an array of bit operation frame
void DebugMotor::send_bit_cmd(int page_index,int pos_index,int32_t set_val,int nth_bit)
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	unsigned char sum=0;
	int size=0;
	unsigned char rw_now=read_write_;
	if(read_write_==1)	//read
	{
		ROS_INFO("\033[32msend_read_cmd:\033[0m");
		s_data[s_length++]=0x55;
		s_data[s_length++]=0x00|pos_index;	
	}
	else if(read_write_==2)	//write
	{
		ROS_INFO("\033[32msend_write_data:\033[0m");
		s_data[s_length++]=0x55;
		s_data[s_length++]=0x80|pos_index;
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
		if(confirm_send(s_data,size))
			if(rw_now==1)
				ROS_INFO("page %d bit %d of byte %d with set value %d was sent to the driver for read ",page_index,nth_bit,pos_index,set_val);
			else if(rw_now==2)
				ROS_INFO("page %d bit %d of byte %d with set value %d was sent to the driver for write",page_index,nth_bit,pos_index,set_val);
			else;
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
	unsigned char rw_now=read_write_;
	if(rw_now==1)
	{
		ROS_INFO("\033[32msend_read_cmd:\033[0m");
		s_data[s_length++]=operated_object;
		s_data[s_length++]=0x00+pos_index;
	}
	else if(rw_now==2)
	{
		ROS_INFO("\033[32msend_write_data:\033[0m");
		s_data[s_length++]=operated_object;
		s_data[s_length++]=0x80+pos_index;
	}
	else;
	if(rw_now==1|rw_now==2)
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
		if(confirm_send(s_data,size))
		{
			if(operated_object==0xaa)
				if(rw_now==1)
					ROS_INFO("page %d byte %d with set value %d was sent to the driver for \033[32mread\033[0m",page_index,pos_index,set_val);
				else if(rw_now==2)
					ROS_INFO("page %d byte %d with set value %d was sent to the driver for \033[32mwrite\033[0m",page_index,pos_index,set_val);
				else;
			else if(operated_object==0xcc)
				if(rw_now==1)
					ROS_INFO("page %d word %d with set value %d was sent to the driver for \033[32mread\033[0m",page_index,pos_index,set_val);
				else if(rw_now==2)
					ROS_INFO("page %d word %d with set value %d was sent to the driver for \033[32mwrite\033[0m",page_index,pos_index,set_val);
				else;
			else if(operated_object==0x33)
				if(rw_now==1)
					ROS_INFO("page %d long word %d with set value %d was sent to the driver for \033[32mread\033[0m",page_index,pos_index,set_val);
				else if(rw_now==2)
					ROS_INFO("page %d long word %d with set value %d was sent to the driver for \033[32mwrite\033[0m,",page_index,pos_index,set_val);
				else;
			else;
		}
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
  m_mutex.lock();
  set_speed_rpm_=config.set_speed_rpm;
  set_torque_Nm_=config.set_torque_Nm;
  control_mode_=config.control_mode;
  m_mutex.unlock();
  motor_id_=config.motor_id;
  send_CAN_=config.send_CAN;
//  batch_read_ =config.batch_read;
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
	std::vector<double> m_break(motor_break_code_,motor_break_code_+6);
	std::vector<double> m_temp(motor_tempoc_,motor_tempoc_+6);
	std::vector<double> m_odom(motor_odom_er_,motor_odom_er_+6);
	m_mutex.unlock();
	msg.send_CAN_counter=send_CAN_counter_;
	msg.get_CAN_counter=get_CAN_counter_;
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
	msg.motor_odom_er=m_odom;
	msg.motor_odom_er.resize(6);	//设定容器尺寸
	msg.message = shake_id_string_;
	msg.a = a_;
	msg.b = b_;
	pub_.publish(msg);
//	ROS_INFO("published msg: message_ = %s,a = %d,b = %d",shake_id_string_.c_str(),a_,b_);
}
void DebugMotor::syschronic_frame(void)
{
	unsigned char s_data[9]={0};
	int size=0;
	s_data[1]=0x33;
	size = ros_ser_.write(s_data,9);	//
	confirm_send(s_data,size);
	
}
//motor control 
void DebugMotor::control_motor()
{
	unsigned char s_data[20]={0};
	unsigned char s_length=0;
	int size=0;
	union int32_uchar setSpeedRpm;
	union int32_uchar setTorqueNm;
	m_mutex.lock();
	setSpeedRpm.val = set_speed_rpm_*65536;
	setTorqueNm.val = set_torque_Nm_*65536;
	m_mutex.unlock();
	s_data[s_length++]=0x00;
	s_data[s_length++]=0x44+motor_id_;
	if(control_mode_==1)	//速度控制模式
	{
		s_data[s_length++]=setSpeedRpm.buf[0];	//低位
		s_data[s_length++]=setSpeedRpm.buf[1];
		s_data[s_length++]=setSpeedRpm.buf[2];
		s_data[s_length++]=setSpeedRpm.buf[3];
		s_length+=2;
		s_data[s_length++]=0x55;
		size = ros_ser_.write(s_data,10);
		confirm_send(s_data,size);
		send_CAN_counter_++;
		control_mode_=0;
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
		size = ros_ser_.write(s_data,10);
		confirm_send(s_data,size);
		send_CAN_counter_++;
		control_mode_=0;
	}
	else if(control_mode_==3)	//motor power on
	{
		s_length+=6;
		s_data[s_length++]=0xa5;
		size = ros_ser_.write(s_data,10);
		confirm_send(s_data,size);
		send_CAN_counter_++;
		control_mode_=0;
	}	
	else if(control_mode_==4)	//motor power off
	{
		s_length+=6;
		s_data[s_length++]=0x5a;
		size = ros_ser_.write(s_data,10);
		confirm_send(s_data,size);
		send_CAN_counter_++;
		control_mode_=0;
	}
	else if(control_mode_==5)
	{
		syschronic_frame();
		send_CAN_counter_++;
		control_mode_=0;
	}
	else if(control_mode_==6)	//constant accelerate for speed
	{
		int32_t temp_speed_rpm=0;
		do
		{	
			temp_speed_rpm+=2;
			set_speed_rpm_=temp_speed_rpm;
			setSpeedRpm.val=temp_speed_rpm<<16;
			s_data[2]=setSpeedRpm.buf[0];	//低位
			s_data[3]=setSpeedRpm.buf[1];
			s_data[4]=setSpeedRpm.buf[2];
			s_data[5]=setSpeedRpm.buf[3];
			s_data[8]=0x55;
			size = ros_ser_.write(s_data,10);
			confirm_send(s_data,size);
			send_CAN_counter_++;
			syschronic_frame();
			send_CAN_counter_++;
			usleep(10000);	//10ms
		}while(temp_speed_rpm<set_speed_rpm_);		
		temp_speed_rpm=set_speed_rpm_;
		do
		{
			temp_speed_rpm-=2;
			temp_speed_rpm=temp_speed_rpm;
			setSpeedRpm.val=temp_speed_rpm<<16;
			s_data[2]=setSpeedRpm.buf[0];	//低位
			s_data[3]=setSpeedRpm.buf[1];
			s_data[4]=setSpeedRpm.buf[2];
			s_data[5]=setSpeedRpm.buf[3];
			s_data[8]=0x55;
			size = ros_ser_.write(s_data,10);
			confirm_send(s_data,size);
			send_CAN_counter_++;
			syschronic_frame();
			send_CAN_counter_++;
			usleep(10000);	//10ms
		}while(temp_speed_rpm>0);	
		control_mode_=0;	
	}
	else if(control_mode_==7)	//constant accelerate for torque
	{
		int32_t temp_torque_Nm=0;
		do
		{	
			temp_torque_Nm+=2;
			set_torque_Nm_=temp_torque_Nm;
			setTorqueNm.val=temp_torque_Nm<<16;
			s_data[2]=setTorqueNm.buf[0];	//低位
			s_data[3]=setTorqueNm.buf[1];
			s_data[4]=setTorqueNm.buf[2];
			s_data[5]=setTorqueNm.buf[3];
			s_data[8]=0xaa;
			size = ros_ser_.write(s_data,10);
			confirm_send(s_data,size);
			send_CAN_counter_++;
			syschronic_frame();
			send_CAN_counter_++;
			usleep(10000);	//10ms
		}while(temp_torque_Nm<set_torque_Nm_);		
		do
		{
			temp_torque_Nm-=2;
			set_torque_Nm_=temp_torque_Nm;
			setTorqueNm.val=temp_torque_Nm<<16;
			s_data[2]=setTorqueNm.buf[0];	//低位
			s_data[3]=setTorqueNm.buf[1];
			s_data[4]=setTorqueNm.buf[2];
			s_data[5]=setTorqueNm.buf[3];
			s_data[8]=0xaa;
			size = ros_ser_.write(s_data,10);
			confirm_send(s_data,size);
			send_CAN_counter_++;
			syschronic_frame();
			send_CAN_counter_++;
			usleep(10000);	//10ms
		}while(temp_torque_Nm>0);
		control_mode_=0;		
	}
	
	else;
	//s_length++;
	
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
			control_motor();	//USB-CAN
			analyse_data();
		}
//		ROS_INFO("spin : %d",counter);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
}
