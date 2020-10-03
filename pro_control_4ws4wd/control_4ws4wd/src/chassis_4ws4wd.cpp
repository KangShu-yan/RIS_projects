//chassis_4ws4wd.cpp
#include "chassis_4ws4wd/chassis_4ws4wd.h"
int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in servaddr;
struct sockaddr_in  src_addr = { 0 };
socklen_t len = sizeof(src_addr);

chassis_info_check_feedback_ chassis_feedback_info = {0};
double l_scale_1=0.0, w_scale_1=0.0; 
double l_scale_2=0.0, w_scale_2=0.0; 

static char motion_state = 0;  
char vel_gear = 0;
ledParam led_control_para ={0};
chassis_motion_cmd motion_cmd_para={0};
ros::Publisher vel_pub_;
/**
 * \file
* @brief Source code for this .cpp that does cmd encode and decode 
*/

/**
  *@ @brief CRC16 function :  cylic redundancy check ,designed to check the frame data issued and uploaded.
  *
  *@ input : 
  *			buffer : frame data besides frame header 
  *			len : length of frame data besides frame header
  *@ output : 
  *			crc : ultimate cylic redundancy check value
**/
unsigned short int CRC16(const unsigned char *buffer, unsigned int len)
{

	unsigned short int crc = 0; ;
	while ((len--)) crc = ((crc >> 8)) ^ crc16_table[(crc ^ (*buffer++)) & 0xff];
	
	return crc;
}
/**
  *@brief 
  *@uchar_to_ushort function :  designed to transfer buf array with two byte unsigned char datas into one short data 
  *
  *@input : 
  *			buffer : an array which is acquired from frame data
  *			i : buffer index
  *@output : 
  *			ultrasonic_val : distance deteceted by ultrasonic,with unit (mm)  
**/
unsigned short int uchar_to_ushort(unsigned char *buf,unsigned char i)
{
	unsigned short int ultrasonic_val=0;
	if((buf[i]&0xff)>>7==0)
	{
		
		ultrasonic_val = (unsigned short int)(buf[i-1]+buf[i]*256);
	}
//	else if((buf[i]&0xff)>>7==1)
//	{
//		ultrasonic_val = -(unsigned short)(buf[i-1]+(buf[i]&0xff)*256);
//		
//	}
//	else;
	return ultrasonic_val;
}
/**
  *@brief 
  *@ encode_motion_cmd function : Designed to encode v and w 
  *
  *@ input : 
  *			linear_x : set linear velocity along axis x
  *			angular_z : set angular velocity along axis z
  * 		motion_state_ :chassis motion state,which includes run ,stop and brake
  * 
  *@ output : 
  *			send_buf : issue linear velocity along axis and angular velocity along axis z to the chassis
**/
void encode_motion_cmd(chassis_motion_cmd &motion_cmd_para)
{
	int ret = 0;
	short int linear_x = motion_cmd_para.v;
	short int angular_z = motion_cmd_para.w;
	unsigned char motion_state = motion_cmd_para.motion_state;
	unsigned char *send_buf = new unsigned char[(unsigned char)motion_CmdLen] ;	//运动控制指令有13个字节
	//unsigned char send_buf[motion_CmdLen] ={0};
	unsigned short int crc = 0;
	short int i=0;
	int len = 0;
	send_buf[i++] = (unsigned short int)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short int)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short int)motion_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short int)(motion_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short int)motion_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short int)(motion_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	send_buf[i++] = motion_state;		//motion_state
	
	if(linear_x<0)
	{
		linear_x=-linear_x;
		send_buf[i++] = linear_x&Uint16_LowByte;		//vel  mm/s
		send_buf[i++] = (((linear_x&Uint16_HighByte))>>8)|0x80;  
	}
	else 
	{
		send_buf[i++] = linear_x&Uint16_LowByte;		//vel  mm/s
		send_buf[i++] = (linear_x&Uint16_HighByte)>>8;  
	}
	
	if(angular_z<0)
	{
		angular_z=-angular_z;
		send_buf[i++] = angular_z&Uint16_LowByte;	//angular velocity	0.1deg/s
		send_buf[i++] = ((angular_z&Uint16_HighByte)>>8)|0x80;
	}
	else
	{
		send_buf[i++] = angular_z&Uint16_LowByte;	//angular velocity	0.1deg/s
		send_buf[i++] = (angular_z&Uint16_HighByte)>>8;
	}
	len = motion_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short int)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short int)(crc&0xff00)>>8;
	//memcpy(send_buf_pointer,send_buf,motion_CmdLen);
	
	std::cout<<"[\033[33msend_motion_cmd\033[0m : ]"<<std::endl;
	std::cout<<"linear_x = "<<linear_x<<"\tangular_z = "<<angular_z<<std::endl;
	for(int i=0;i<motion_CmdLen;i++)
	{
		printf("%.2x ",send_buf[i]);
	}
	printf("\n");
//	printf("motion_CmdLen = %d\t,sizeof(send_buf) = %d\t",(unsigned int)motion_CmdLen,sizeof(send_buf));
	ret = sendto(udp_sock, send_buf,motion_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	delete[] send_buf;
}
/**
  *@brief 
  *@encode_odometry_cmd function : Designed to encode odometry cmd
  *
  *@input : 
  *			cmdId : command ID
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_odometry_cmd(void)
{
	unsigned char *send_buf = new unsigned char[odom_CmdLen];	//运动控制指令有13个字节
	unsigned short int crc = 0;
	short i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)odom_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(odom_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)odometry_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(odometry_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	
	len = odom_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short)(crc&0xff00)>>8;
	
	ret = sendto(udp_sock, send_buf, odom_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	delete[] send_buf;
}
/**
  *@brief 
  *@encode_ultrasonic_cmd function : Designed to encode ultrasonic ,antiCollisionBar and their brake cmd
  *
  *@input : 
  *			cmdId : command ID
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_ultrasonic_cmd()
{
	unsigned char *send_buf = new unsigned char[ultrasonic_CmdLen];	//运动控制指令有13个字节
	unsigned short int crc = 0;
	short i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)ultrasonic_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(ultrasonic_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)ultrasonic_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(ultrasonic_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	
	len = ultrasonic_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short)(crc&0xff00)>>8;
	ret = sendto(udp_sock, send_buf, ultrasonic_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	delete[] send_buf;
}
/**
  *@brief 
  *@encode_antiColBar_cmd function : Designed to encode ultrasonic ,antiCollisionBar and their brake cmd
  *
  *@input : 
  *			cmdId : command ID
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_antiColBar_cmd(void)
{
	unsigned char *send_buf = new unsigned char[antiCollisionBar_CmdLen];	//运动控制指令有13个字节
	unsigned short int crc = 0;
	short int i=0;
	int len = 0;
	short int ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)antiCollisionBar_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(antiCollisionBar_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)antiCollisionBar_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(antiCollisionBar_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	
	len = antiCollisionBar_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short)(crc&0xff00)>>8;
	ret = sendto(udp_sock, send_buf, antiCollisionBar_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	delete[] send_buf;
}
/**
  *@brief 
  *@encode_ultrasonic_brake_cmd function : Designed to encode brake cmd related to ultrasonic and antiCollisionBar
  *
  *@input : 
  *			cmdId : command ID
  *			cmd : enable or disable 
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_ultrasonic_brake_cmd(unsigned char cmd)//
{
	unsigned char *send_buf = new unsigned char[ultra_antiCol_brake_CmdLen];	//运动控制指令有13个字节
	unsigned short crc = 0;
	short i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)odom_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(odom_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)ultrasonicBrake_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(ultrasonicBrake_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	send_buf[i++] = cmd;
	
	len = ultra_antiCol_brake_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short)(crc&0xff00)>>8;
	delete[]  send_buf;
}
/**
  *@brief 
  *@encode_antiColBar_brake_cmd function : Designed to encode brake cmd related to ultrasonic and antiCollisionBar
  *
  *@input : 
  *			cmdId : command ID
  *			cmd : enable or disable 
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_antiColBar_brake_cmd(unsigned char cmd)//
{
	unsigned char *send_buf = new unsigned char[ultra_antiCol_brake_CmdLen];	//运动控制指令有13个字节
	unsigned short crc = 0;
	short i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)ultra_antiCol_brake_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(ultra_antiCol_brake_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)antiCollisionBarBrake_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(antiCollisionBarBrake_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	send_buf[i++] = cmd;
	
	len = ultra_antiCol_brake_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short)(crc&0xff00)>>8;
	delete[]  send_buf;
}
/**
  *@brief 
  *@encode_driver_exception_cmd function : Designed to encode driver exception cmd
  *
  *@input : 
  *			driverSide : 0x00 left side ; 0x01 right side
  * 		
  *@output : 
  *			send_buf : a frame data to be issued to chassis 
**/
void encode_driver_exception_cmd(unsigned char driverSide)	//right side and left side
{
	unsigned char *send_buf = new unsigned char[driver_CmdLen];	//运动控制指令有13个字节
	unsigned short int crc = 0;
	short int i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short int)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short int)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short int)driver_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short int)(driver_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short int)motorDriver_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short int)(motorDriver_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	send_buf[i++] = driverSide&0xff;
	len =driver_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short int)crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short int)(crc&0xff00)>>8;
	ret = sendto(udp_sock, send_buf, driver_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	delete[]  send_buf;
}
/**
  *@brief 
  *@encode_led_cmd function : Designed to encode led control cmd
  *
  *@input : 
  *			led_para_ : led control variable
  * 		
  *@output : 
  *			send_buf : an frame data to be issued to chassis 
**/
void encode_led_cmd(ledParam led_para_)
{
	unsigned char *send_buf = new unsigned char[led_CmdLen];	//运动控制指令有13个字节
	unsigned short int crc = 0;
	short i=0;
	int len = 0;
	unsigned char ret = 0;
	send_buf[i++] = (unsigned short)GS_ascii&Uint16_LowByte;		//"GS"
	send_buf[i++] = (unsigned short)(GS_ascii&Uint16_HighByte)>>8;	
	send_buf[i++] = (unsigned short)led_CmdLen&Uint16_LowByte;		//Size
	send_buf[i++] = (unsigned short)(led_CmdLen&Uint16_HighByte)>>8;
	send_buf[i++] = (unsigned short)led_CmdId&Uint16_LowByte;		//CmdId
	send_buf[i++] = (unsigned short)(led_CmdId&Uint16_HighByte)>>8;
	send_buf[i++] = 0x00;		//CRC 10 C0  09 00 00 00  
	send_buf[i++] = 0x00;
	
	send_buf[i++] = (unsigned char)led_para_.channel&0xff;		//
	send_buf[i++] = (unsigned char)led_para_.mode&0xff;
	send_buf[i++] = (unsigned char)led_para_.on_lightness&0xff;		//
	
	send_buf[i++] = (unsigned char)led_para_.on_time&0xff;
	send_buf[i++] = (unsigned char)led_para_.off_time&0xff;
	send_buf[i++] = (unsigned char)led_para_.flash_lightness&0xff;
	
	send_buf[i++] = (unsigned char)led_para_.speed&0xff;
	send_buf[i++] = (unsigned char)led_para_.lightness_min&0xff;
	send_buf[i++] = (unsigned char)led_para_.lightness_max&0xff;
	
	len = led_CmdLen-8;
	crc = CRC16(send_buf+8, len);	//仅针对数据部分
	send_buf[6] = (unsigned short int )crc&0x00ff;		//CRC 10 C0  09 00 00 00  
	send_buf[7] = (unsigned short int)(crc&0xff00)>>8;
	
	ret = sendto(udp_sock, send_buf, led_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	//show 
	std::cout<<std::endl;
	for(int i=0;i<led_CmdLen;i++)
	{
		printf("%.2x ",send_buf[i]);
	}
	std::cout<<std::endl;
}
/**@brief
  *@decode_motion_cmd function : Designed to decode v and w 
  *
  *@input :   
  *			buf : uploaded frame data correlated with motion cmd
  *			len : length of frame data
  *
  *@output :
  *			v : feedback linear velocity along axis x 
  *         w : feedbakc angular velocity along axis z 
**/
void decode_motion_cmd(unsigned char* buf,unsigned char len)
{
	short int v=0;	//mm/s
	short int w=0;	//deg/s
	if((buf[len-3]&0xff)>>7==1)
	{
		v =- (buf[len-4]+(buf[len-3]&0x7f)*256);
	}
	else if((buf[len-3]&0xff)>>7==0)
	{
		v = (buf[len-4]+(buf[len-3]&0xff)*256);
	}
	else;
	
	if((buf[len-1]&0xff)>>7==1)
	{
		w =- (buf[len-2]+(buf[len-1]&0x7f)*256);
		//float_w=w*0.1;
	}
	else if((buf[len-1]&0xff)>>7==0)
	{
		w = (buf[len-4]+(buf[len-1]&0xff)*256);
	}
	else;
//	printf("%d ",(char)buf[len-3]);
//	printf("%d ",(char)buf[len-4]);
//	v_high_byte=(char)buf[len-3];
//	v = (float)(buf[len-4]+v_high_byte*256);
//	w = (float)(buf[len-2]+((char)buf[len-1])*256)*0.1;
	chassis_feedback_info.v = v;
	chassis_feedback_info.w = w;
	std::cout<<"current v(mm/s) : "<< v<<"\tcurrent W(0.1deg/s) : "<<w<<std::endl;
}
/**@brief
  *@decode_odometry_cmd function : Designed to decode odometry frame data.
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state   
**/
short decode_odometry_cmd(unsigned char* buf,unsigned char len)
{
	
	double mileage =0.0; //mm
	float angle = 0.0; //deg
	std::cout<<"[odom_info] : ";

	mileage = buf[len-8]+(buf[len-7]<<8)+(buf[len-6]<<16)+(buf[len-5]<<24);
	angle = (buf[len-4]+(buf[len-3]<<8)+(buf[len-2]<<16)+(buf[len-1]<<24))*0.1;

	chassis_feedback_info.mileage = mileage;
	chassis_feedback_info.angle = angle;
	std::cout<<"current mileage(mm) : "<< mileage<<"\tcurrent angle(deg) : "<<angle<<std::endl;
	return 1;
}
/**@brief
  *@decode_ultrasonic_cmd function : Designed to decode ultrasonic frame data
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state   
**/
short int decode_ultrasonic_cmd(unsigned char* buf,unsigned char len)
{
	int i=0;
	std::cout<<"[ultrasonic_info] : ";
	for(int buf_index = 9;buf_index<len;buf_index+=2)	//len = 23
	{
		chassis_feedback_info.ultrasonic_distance[i] = uchar_to_ushort(buf,buf_index);	//mm
		std::cout<<"ultrasonic["<<i<<"] : "<<chassis_feedback_info.ultrasonic_distance[i]<<"\t";
			
		if(chassis_feedback_info.ultrasonic_distance[i]<10)
		{
			return -1;
		}
			i++;
	}
	return 1;	
}
	
/**@brief
  *@decode_antiColBar_cmd function : Designed to decode anti-collision bar frame data
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state   
**/
short int decode_antiColBar_cmd(unsigned char* buf,unsigned char len)
{
	std::cout<<"[antiCollisionBar_info] : ";
	chassis_feedback_info.antiCollisionBarStatus = buf[len-1];
	if(chassis_feedback_info.antiCollisionBarStatus==1)
	{
		std::cout<<"---!!!!!!!!!!!!!!!!collided!!!!!!!!!!!!!!---"<<std::endl;
		return -2;		
	}
	else
	{
		std::cout<<"[no collision]"<<std::endl;
	}
	return 1;
}
/**@brief
  *@decode_ultrasonic_brake_cmd function : Designed to decode brake frame data related to ultrasonic .
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state   
**/
short int decode_ultrasonic_brake_cmd(unsigned char* buf,unsigned char len)
{
	std::cout<<"[ultrasonicBrake_info] : ";
	chassis_feedback_info.ultrasonicBrakeStatus = buf[len-1];
	if(chassis_feedback_info.ultrasonicBrakeStatus==1)
	{
		std::cout<<"Braking from ultrasonic enabled."<<std::endl;
	}
	else
	{
		std::cout<<"Ultrasonic brake diable."<<std::endl;
	}
	return 1;
}
/**@brief
  *@decode_antiColBar_brake_cmd function : Designed to decode brake frame data related to anti-collision bar.
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state   
**/
short decode_antiColBar_brake_cmd(unsigned char* buf,unsigned char len)
{
	std::cout<<"[antiCollisionBarBrake_info] : ";
	chassis_feedback_info.antiCollisionBarBrakeStatus = buf[len-1];
	if(chassis_feedback_info.antiCollisionBarBrakeStatus==1)
	{
		std::cout<<"Braking from antiCollisionBar enabled."<<std::endl;
	}
	else
	{
		std::cout<<"AntiCollisionBar brake diable."<<std::endl;
	}
	return 1;
}
/**@brief
  *@decode_driver_exception_cmd function : Designed to decode driver state 
  * 
  *
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			chassis state 	   
**/
void decode_driver_exception_cmd(unsigned char* buf,unsigned char len)
{
	std::cout<<"[driverExceptionCheck_info] : ";
	chassis_feedback_info.driverStatus = buf[len-1];
	if((chassis_feedback_info.driverStatus&0x00000001)==1)
	{
		std::cout<<"Current of driver too high!!!!!!!!!!!!---"<<std::endl;
	}
	else;
	if(((chassis_feedback_info.driverStatus>>1)&0x00000001)==1)
	{
		std::cout<<"\033[31mVoltage of driver too high!!!!!!!!!!!!---\033[0m"<<std::endl;		//green fonts
	}
	else;
	if(((chassis_feedback_info.driverStatus>>2)&0x00000001)==1)
	{
		std::cout<<"Encode of driver break exception!!!!!!!---."<<std::endl;
	}
	else;
	if(((chassis_feedback_info.driverStatus>>3)&0x00000001)==1)
	{
		std::cout<<"Voltage of driver too low!!!!!!!!!!!!---"<<std::endl;
	}
	else;
	if(((chassis_feedback_info.driverStatus>>4)&0x00000001)==1)
	{
		std::cout<<"Motor overloaded !!!!!!!!!!!!---"<<std::endl;
	}
	else
	{
		std::cout<<"Drivers are fine. "<<std::endl;
	}
}
/**@brief
  *@decode_led_cmd function : Designed to led state
  * 
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 
  *			led control state 	   
**/
void decode_led_cmd(unsigned char* buf,unsigned char len)
{
	std::cout<<"[ledExecute_info] : ";
	chassis_feedback_info.ledExecuteStatus = buf[len-1];
	if(chassis_feedback_info.ledExecuteStatus==0x55)
	{
		std::cout<<"Led executed normally."<<std::endl;
	}
	else if(chassis_feedback_info.ledExecuteStatus == 0xAA)
	{
		std::cout<<"Led executed error."<<std::endl;
	}
	else;
}
/**@brief
  *@decode_led_cmd function : Designed to analyse frame data ,an essential  function used to identify current frame data 
  * 
  *@input :   
  *			buf : uploaded frame data
  *			len : length of frame data
  *@output : 		
  *  	   
**/
short decode_cmd(unsigned char* buffer,unsigned char len)
{
	unsigned short int crc = 0;
	int crc_length = 0;
	unsigned char index = 0;
	unsigned char buffer_index[10];
	short int ret=0;
	bool flag=false;
	short int test_val=0;
	for(int i=0;i+8<len;i++)
	{
	//	motion
//		printf("buffer[cmdId_lowByteIndex+i] =%.2x \n",buffer[cmdId_lowByteIndex+i]);
//		printf("(motion_CmdId&Uint16_LowByte) = %.2x \n",(motion_CmdId&Uint16_LowByte));
//		printf("buffer[cmdId_highByteIndex+i] = %.2x \n",buffer[cmdId_highByteIndex+i]);
//		printf("(motion_CmdId&Uint16_HighByte>>8) = %.2x \n",((motion_CmdId&Uint16_HighByte)>>8));
		if((buffer[cmdId_lowByteIndex+i]==(unsigned char)(motion_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==(unsigned char)((motion_CmdId&Uint16_HighByte)>>8)))
		{	
			std::cout<<"[\033[32mdecode motion_CmdId\033[0m] : "<<std::endl;
//			std::cin>>test_val;
//			std::cout<<"test_val = "<<test_val<<std::endl;
			crc_length = motion_feedback_CmdLen-8;
			std::cout<<"crc_length = "<<crc_length<<std::endl;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
				decode_motion_cmd(buffer+i,motion_feedback_CmdLen);	
//				std::cout<<"[\033[32mmotion_CmdId_crc_ok\033[0m] : "<<std::endl;
				std::cout<<std::endl;
//				if(len>(motion_feedback_CmdLen+i))
//				{
//					i+=motion_feedback_CmdLen;
//					flag=true;
//				}
			}
		}
		else;
		//odom
		if((buffer[cmdId_lowByteIndex+i]==(odometry_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((odometry_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode odometry_CmdId\33[0m] : "<<std::endl;
			crc_length = odom_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32modometry_CmdId_crc_ok\033[0m] : ";
//				decode_odom_ultra_antiColBar_brake_cmd(buffer+i, odom_feedback_CmdLen,0);
				decode_odometry_cmd( buffer+i,odom_feedback_CmdLen);
				if(len>(odom_feedback_CmdLen+i))
				{
					i+=odom_CmdLen;
					flag=true;
				}
			}
		}
		else;
		// ultrasonic
		if((buffer[cmdId_lowByteIndex+i]==(ultrasonic_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((ultrasonic_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode ultrasonic_check_feedback\033[0m] : "<<std::endl;
			crc_length = ultrasonic_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32multrasonic_CmdId_crc_ok\033[0m] : ";
				ret = decode_ultrasonic_cmd(buffer+i, ultrasonic_feedback_CmdLen);	
//				i+=ultrasonic_feedback_CmdLen;
//				flag=true;
				if(ret<0)
				{
					return ret;
				}
			}
		}
		else;
		//antiCollisionBar_check_feedback
		if((buffer[cmdId_lowByteIndex]==(antiCollisionBar_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex]==((antiCollisionBar_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode antiCollisionBar_check_feedback\033[0m] : "<<std::endl;
			crc_length = ultra_antiCol_brake_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{	
//				std::cout<<"[\033[32mantiCollisionBar_CmdId_crc_ok\033[0m] : ";
				ret = decode_antiColBar_cmd(buffer+i,antiColBar_feedback_CmdLen);
				if(ret<0)
				{
					return ret;
				}
			}
		}
		else;
		//ultrasonicBrake_feedback
		if((buffer[cmdId_lowByteIndex+i]==(ultrasonicBrake_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((ultrasonicBrake_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode ultrasonicBrake_check_feedback\033[0m] : "<<std::endl;
			crc_length = ultra_antiCol_brake_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32multrasonicBrake_CmdId_crc_ok\033[0m] : ";
				decode_ultrasonic_brake_cmd( buffer+i,ultrasonic_brake_feedback_CmdLen);
//				i+=ultra_antiCol_brake_feedback_CmdLen;
//				flag=true;
			
			}
		}
		else;
		//antiCollisionBar_feedback
		if((buffer[cmdId_lowByteIndex+i]==(antiCollisionBarBrake_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((antiCollisionBarBrake_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode antiCollisionBarBrake_check_feedback\033[0m] : "<<std::endl;
			
			crc_length = ultra_antiCol_brake_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32mantiCollisionBarBrake_CmdId_crc_ok\033[0m] : ";
				
				decode_antiColBar_brake_cmd(buffer+i, antiCol_brake_feedback_CmdLen);
//				i+=ultra_antiCol_brake_feedback_CmdLen;
//				flag=true;
			}
		}
		else;
		//driver_check_feedback
		if((buffer[cmdId_lowByteIndex+i]==(motorDriver_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((motorDriver_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode motorDriver_check_feedback\033[0m] : "<<std::endl;
			crc_length = driver_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32mmotorDriver_CmdId_crc_ok\033[0m] : ";
				
				decode_driver_exception_cmd( buffer+i,driver_feedback_CmdLen);
//				i+=driver_feedback_CmdLen;
//				flag=true;
			}
		}
		else;
		//led_control_feedback
		if((buffer[cmdId_lowByteIndex+i]==(led_CmdId&Uint16_LowByte))&&(buffer[cmdId_highByteIndex+i]==((led_CmdId&Uint16_HighByte)>>8)))
		{
			std::cout<<"[\033[32mdecode led_check_feedback\033[0m] : "<<std::endl;
			crc_length = led_feedback_CmdLen-8;
			crc = CRC16(buffer+8+i, crc_length);	//仅针对数据部分
			printf("crc = %.4x\n",crc);
			
			if((buffer[crc_lowByteIndex+i]==(crc&Uint16_LowByte))&&(buffer[crc_highByteIndex+i]==((crc&Uint16_HighByte)>>8)))
			{
//				std::cout<<"[\033[32mled_CmdId_crc_ok\033[0m] : ";
				decode_led_cmd(buffer+i,led_feedback_CmdLen);
//				i+=led_CmdLen;
//				flag=true;
			}
		}
		else 
		{
		}
	}
	return 0;	
}
/**
  *@ udp_init function : init udp communication  
  * 
  *@ input :   
  *			 
  *@ output : 
  *			  
  *		 	   
**/
short int udp_init()
{
	if(udp_sock <0)
	{
		std::cout<<"[\033[31msocket unsuccessfully!\033[0m]";
		exit(1);
	}
	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family = AF_INET;                  /* Internet/IP */
	servaddr.sin_port = htons(4002);       /* server port */
    servaddr.sin_addr.s_addr = inet_addr("10.7.5.220");  /* IP address 本机地址，非主控板地址*/  

    int ret_udp = bind(udp_sock, (struct sockaddr*)&servaddr,  sizeof(servaddr));
    if (ret_udp < 0)
	{
		std::cout << "[\033[31mbind failed!\033[0m]" << std::endl;
		close(udp_sock);
		return -1;
	}
	else 
	{
		std::cout << "[\033[32mrecv ready!\033[0m]" << std::endl;
	}
	
	return 1;
}
/**
  *@ param_init function : init basic parameters of joystick  
  * 
  *@ input :   
  *			nh_: node handler  
  *@ output : 
  *			  	 	   
**/
void param_init(ros::NodeHandle &nh_)
{
	bool ifGetPara;

	ifGetPara = ros::param::get("/chassis_4ws4wd/l_scale_1", l_scale_1);//match with<node  * name  />
	if(ifGetPara)
	{
		ROS_INFO("Got l_scale_1");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get l_scale_1");
	}
	ifGetPara = ros::param::get("/chassis_4ws4wd/l_scale_2", l_scale_2);
	if(ifGetPara)
	{
		ROS_INFO("Got l_scale_2");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get l_scale_2");
	}
	ifGetPara = ros::param::get("/chassis_4ws4wd/w_scale_1", w_scale_1);
	if(ifGetPara)
	{
		ROS_INFO("Got w_scale_1");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get w_scale_1");
	}
	ifGetPara = ros::param::get("/chassis_4ws4wd/w_scale_2", w_scale_2);
	if(ifGetPara)
	{
		ROS_INFO("Got w_scale_2");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get w_scale_2");
	}
	//define a subscriber
	ros::Subscriber joy_sub_; 
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 200); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback); 
	
}
/**
  *@ run function : cyclic function in main function   
  * 
  *@ input :   
  *					
  *@ output : 
  *			  	 	   
**/
void run()
{
	short int ret=0;
	short int recv_length=0;
	static int number=1;
	unsigned char steering_feedback_data[MAXLEN] = {0};
	number++;
	recv_length=recvfrom(udp_sock,steering_feedback_data, 1024, 0,(struct sockaddr*)&src_addr, &len);//&number
	if(recv_length>0)
	{
	 	printf("motion_state : %.2x \t   vel_gear ：%.2x \n",motion_cmd_para.motion_state,motion_cmd_para.vel_gear);
	 	printf("[%s:%d]",inet_ntoa(src_addr.sin_addr),ntohs(src_addr.sin_port));//打印消息发送方的ip与端口号
	 	std::cout << "Received " << number << "th data (with length "<< recv_length<<")："<<std::endl;
	 	for(int i=0;i<recv_length;i++)
	 	{
	 		printf("%.2x ",steering_feedback_data[i]);
	 	}
		std::cout<<std::endl;
		ret = decode_cmd(steering_feedback_data, recv_length);
		if(ret==-1)
		{
			encode_ultrasonic_brake_cmd(1);
		}
		else if(ret==-2)
		{
			encode_antiColBar_brake_cmd(1);
		}
		else
		{	
			encode_ultrasonic_brake_cmd(0);
			encode_antiColBar_brake_cmd(0);
		}
	}
	else
	{
		std::cout << "No recevied data !" << std::endl;
	}	
	if(number%5==0)
	{
		encode_odometry_cmd();
		encode_motion_cmd(motion_cmd_para);
	}
	if(number%7==0)
	{	
		encode_ultrasonic_cmd();	
	}
	if(number%9==0)
	{	
		encode_antiColBar_cmd();
	}
	if(number%4==0)
	{
		encode_driver_exception_cmd(0);	//left-side wheel
		encode_driver_exception_cmd(1);	//right-side wheel	
	}
	if(number>10000)
	{
		number=1;
	}	
	std::cout<<std::endl;
}
/**
  *@ chassis_close_udp function : designed to close udp
  * 
  *@ input :   
  *			
  *@ output : 
  *			  			   
**/
void chassis_close_udp(void)
{
	close(udp_sock);
}
/**
  *@ joyCallback function : subscribe joy  
  *
  *@ input :   
  *			joy : command from operator
  *@ output : 
  *			   	   
**/
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{

   	static int count=0;
	static double l_scale_=0.5, w_scale_=0.5; 
	geometry_msgs::Twist twist; 
	//0 : stop 1 : run 2 : brake 
//	std::cout<<"transfor_joy"<<std::endl; 
	//ROS_INFO("transfor_joy_callback"); 
	if(joy->buttons[3])		//X键	 3是Y	7 start	//white beitong joystick
	{
		if(joy->buttons[2])	//A键		
		{
//			l_scale_=0.25*32767.;
			motion_cmd_para.vel_gear = 1;	//1档
			l_scale_=l_scale_1;
			w_scale_=w_scale_1;
		}
		else if(joy->buttons[1])	//B键
		{
			motion_cmd_para.vel_gear = 2;	//2档
			l_scale_=l_scale_2;
			w_scale_=w_scale_2;
		}
		else if(joy->buttons[0])	//Y
		{
			motion_cmd_para.motion_state=0;			//stop
		}
		else if(joy->buttons[9])	//start
		{
			motion_cmd_para.motion_state = 1;		//run
			encode_antiColBar_brake_cmd(0);
			encode_ultrasonic_brake_cmd(0);
		}
		else if(joy->buttons[8]||joy->axes[5])  //back 
		{
			std::cout<<"[\033[33mled_setting\033[0m] : "<<std::endl;
			led_control_para.channel = 1 ;
			led_control_para.mode = !led_control_para.mode;
//			led_control_para.mode = 0x00;
			led_control_para.on_lightness=1;
			if(joy->axes[5]==1&&led_control_para.on_lightness<10)
			{
				led_control_para.on_lightness+=1;	//0-10个等级
			}
			else if(joy->axes[5]==-1&&led_control_para.on_lightness>0)
			{
				led_control_para.on_lightness+=1;	//0-10个等级
			}
			else;
//			led_control_para.on_lightness=5;	//0-10个等级
			printf("%.2x \n",led_control_para.mode);	
			//Sending led cmd toward chassis.
			encode_led_cmd(led_control_para);
		}
		else;
		twist.angular.z = w_scale_*joy->axes[0]; //左摇杆左右
		twist.linear.x = l_scale_*joy->axes[1]; //左摇杆前后
	}
	else if(joy->buttons[6]||joy->buttons[7])	//LT  RT 
	{
			motion_cmd_para.motion_state = 2;	//brake
	}
	else
	{
		twist.angular.z = 0; 
		twist.linear.x = 0; 
	} 
	motion_cmd_para.v = twist.linear.x; 
	motion_cmd_para.w = twist.angular.z;
	vel_pub_.publish(twist);
}
//end of chassis_4ws4wd.cpp






