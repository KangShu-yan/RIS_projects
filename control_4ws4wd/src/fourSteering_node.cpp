
/**
 * 
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "std_msgs/String.h"
#include <stdbool.h>

#include <iostream>
#include <sstream>

#include "sixwheel_chassis_control/msg_chassis.h"
#include "deal_cmd/deal_cmd.h"  //tcp_client is the directory contained in include directory 
#define MAXLEN 1024

double l_scale_1=0.0, w_scale_1=0.0; 
double l_scale_2=0.0, w_scale_2=0.0; 
static char motion_state = 0;  
char vel_gear = 0;
ledParam led_control_para ={0};

int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in servaddr;
struct sockaddr_in  src_addr = { 0 };
socklen_t len = sizeof(src_addr);

ros::Publisher vel_pub_;

float max_limit(float val,const float max_val)
{
	if(val>max_val)
	{
		val = max_val;
	}
	else if(val<-max_val)
	{
		val =-max_val;
	}
	else;
	
	return val;
}
/****************************************************************************
*	function:
		decode_four_steering_info:decode the frame datas from chassis
*	input:
*		rec_data:received frame datas from chassis
*		number:assumed whole size of the frame datas
*	output:
*		 data :
		 data :
*****************************************************************************/
void decode_four_steering_info(unsigned char* rec_data,int number)
{
	static int16_t count=0;
	double press_data=0.0,degree_data=0.0;
	int j=0;
	
	decode_cmd(rec_data,number);
	ROS_INFO("decode_four_steering_info function\n");
//	
	for (int i = 0; i < number; i++)
	{
		printf("%2.2x    ", (unsigned char)rec_data[i]);
//		
	}
	printf("\n");

}
/****************************************************************************
*	function:
		
*	input:
*		
*	output:
*		send_buf 
*****************************************************************************/
void send_motion_cmd(float linear_x=0,float angular_z=0)
//void encode_four_steering_info(void)
{
	int ret = 0;
	unsigned short crc = 0;
	int len = 0;
 	unsigned char *send_buf = encode_motion_cmd(linear_x,angular_z,motion_state);
 	
	ret = sendto(udp_sock, send_buf,motion_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
	
}
/****************************************************************************
*	function:
		
*	input:
*		
*	output:
*		send_buf 
*****************************************************************************/
void send_odom_ultra_antiColBar_check_cmd(unsigned short cmdId)
{
	int ret = 0;
	unsigned short crc = 0;
	int len = 0;
 	unsigned char *send_buf = encode_odom_ultra_antiColBar_cmd(cmdId);
	ret = sendto(udp_sock, send_buf, odom_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}
/****************************************************************************
*	function:
		
*	input:
*		
*	output:
*		send_buf 
*****************************************************************************/
void send_ultra_antiColBar_brake_cmd(unsigned short cmdId,unsigned char cmd=0)
{
	int ret = 0;
	
	//unsigned char cmd = 1;
 	unsigned char *send_buf = encode_ultra_antiColBar_brake_cmd(cmdId,cmd);
	ret = sendto(udp_sock, send_buf, ultra_antiCol_brake_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}
/****************************************************************************
*	function:
		
*	input:
*		
*	output:
*		send_buf 
*****************************************************************************/
void send_driver_exception_check_cmd(unsigned char driverSide)
{
	int ret = 0;
	
	
 	unsigned char *send_buf = encode_driver_exception_cmd(driverSide);
	ret = sendto(udp_sock, send_buf, driver_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}

/****************************************************************************
*	function:
		
*	input:
*		
*	output:
*		send_buf 
*****************************************************************************/
void send_led_cmd(void)
{
	int ret = 0;
 	unsigned char *send_buf = encode_led_cmd(led_control_para);
	ret = sendto(udp_sock, send_buf, led_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	std::cout<<std::endl;
	for(int i=0;i<led_CmdLen;i++)
	{
		printf("%.2x ",send_buf[i]);
	}
	std::cout<<std::endl;
	delete[] send_buf;
}

/****************************************************************************
*	function:
		callback function fo subscriber
*	input:
*		
*	output:
*		
*****************************************************************************/
void param_init(ros::NodeHandle &nh_)
{
	bool ifGetPara;
	nh_.param<double>("l_scale_1", l_scale_1, 0.);
//	nh_.param<double>("l_scale_2", l_scale_2, 0.);
//	nh_.param<double>("w_scale_1", w_scale_1, 0.);
//	nh_.param<double>("w_scale_2", w_scale_2, 0.);
	ifGetPara = ros::param::get("/fourSteering/l_scale_1", l_scale_1);
	if(ifGetPara)
	{
		ROS_INFO("Got l_scale_1");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get l_scale_1");
	}
	ifGetPara = ros::param::get("/fourSteering/l_scale_2", l_scale_2);
	if(ifGetPara)
	{
		ROS_INFO("Got l_scale_2");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get l_scale_2");
	}
	ifGetPara = ros::param::get("/fourSteering/w_scale_1", w_scale_1);
	if(ifGetPara)
	{
		ROS_INFO("Got w_scale_1");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get w_scale_1");
	}
	ifGetPara = ros::param::get("/fourSteering/w_scale_2", w_scale_2);
	if(ifGetPara)
	{
		ROS_INFO("Got w_scale_2");
	}
	else
	{
		ROS_ERROR_STREAM("Didn't get w_scale_2");
	}

	


}
short udp_init()
{
	if(udp_sock <0)
	{
		ROS_WARN_STREAM_ONCE("socket unsuccessfully!");
		exit(1);
	}
	
	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family = AF_INET;                  /* Internet/IP */
	servaddr.sin_port = htons(4002);       /* server port */
    servaddr.sin_addr.s_addr = inet_addr("10.7.5.220");  /* IP address 本机地址，非主控板地址*/  

    int ret_udp = bind(udp_sock, (struct sockaddr*)&servaddr,  sizeof(servaddr));
    if (ret_udp < 0)
	{
		std::cout << "bind failed!" << std::endl;
		close(udp_sock);
		return -1;
	}
	else 
	{
		std::cout << "recv ready!" << std::endl;
	}
	
	return 1;
}

/****************************************************************************
*	function:
		callback function fo subscriber
*	input:
*		
*	output:
*		
*****************************************************************************/

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
			vel_gear = 1;	//1档
			l_scale_=l_scale_1;
			w_scale_=w_scale_1;
		}
		else if(joy->buttons[1])	//B键
		{
			vel_gear = 2;	//2档
			l_scale_=l_scale_2;
			w_scale_=w_scale_2;
		}
		else if(joy->buttons[0])	//Y
		{
			motion_state=0;			//stop
		}
		else if(joy->buttons[9])	//start
		{
			motion_state = 1;		//run
		}
		else if(joy->buttons[6]||joy->buttons[7])	//LT  RT 
		{
			motion_state = 2;	//brake
		}
		else if(joy->buttons[8])  //back 
		{
			std::cout<<"[\033[33mled_setting\033[0m] : "<<std::endl;
			//led_control_para.channel = 5 ;
			//led_control_para.channel = 6 ;
			//led_control_para.channel = 7 ;
			//led_control_para.channel = 8 ;
			led_control_para.channel = 9 ;
			
//			led_control_para.mode = !led_control_para.mode;
			led_control_para.mode = 0x01;
			led_control_para.on_lightness=8;	//0-10个等级
			printf("%.2x \n",led_control_para.mode);	
			send_led_cmd();
		}
		else;
		twist.angular.z = w_scale_*joy->axes[0]; //左摇杆左右
		twist.linear.x = l_scale_*joy->axes[1]; //左摇杆前后
	}
	else
	{
		twist.angular.z = 0; 
		twist.linear.x = 0; 
	} 
	
	
	
	//twist.linear.x = max_limit(twist.linear.x,100);	//< = 100mm/s
	//twist.angular.z = max_limit(twist.angular.z,30);// 30deg/s
	send_motion_cmd(twist.linear.x ,twist.angular.z);
	vel_pub_.publish(twist);
	
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "fourSteering_control");
    // 创建节点句柄
    ros::NodeHandle nh_;
	ros::Rate loop_rate(1000);  //Hz
	unsigned char steering_feedback_data[MAXLEN] = {0};
	int number=0,ret=0;
	short recv_length=0;
	
	param_init(nh_);
	if(udp_init()<0)
	{
		return -1;
	}
	
	ros::Subscriber joy_sub_; 
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 200); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback); 
    while(ros::ok())
	{
		number++;
	 	recv_length=recvfrom(udp_sock,steering_feedback_data, 1024, 0,(struct sockaddr*)&src_addr, &len);//&number
	 	//std::cout<<"ret_2 = "<<ret<<std::endl;
	 	
	 	if(recv_length>0)
	 	{
	 		printf("motion_state : %.2x \t   vel_gear ：%.2x \n",motion_state,vel_gear);
	 		printf("[%s:%d]",inet_ntoa(src_addr.sin_addr),ntohs(src_addr.sin_port));//打印消息发送方的ip与端口号
	 	
	 		std::cout << "recv data  " << number << "th  data (with length "<< recv_length<<")："<<std::endl;
	 		//decode_four_steering_info(steering_feedback_data,recv_length);
	 		
	 		for(int i=0;i<recv_length;i++)
	 		{
	 			printf("%.2x ",steering_feedback_data[i]);
	 		}
	 		std::cout<<std::endl;
			ret = decode_cmd(steering_feedback_data, recv_length);
			if(ret==-1)
			{
				send_ultra_antiColBar_brake_cmd(antiCollisionBarBrake_CmdId,1);
			}
			else if(ret==-2)
			{
				send_ultra_antiColBar_brake_cmd(ultrasonicBrake_CmdId,1);
			}
			else
			{	
//				send_ultra_antiColBar_brake_cmd(ultrasonicBrake_CmdId);
			}
	 	}
	 	else
		{
			std::cout << "No recevied data !" << std::endl;
		}
		
//		if(number%5==0)
//		{
//			send_odom_ultra_antiColBar_check_cmd(odometry_CmdId);		//
//		}
//		if(number%7==0)
//		{
//			send_odom_ultra_antiColBar_check_cmd(ultrasonic_CmdId);
//			
//		}
//		if(number%9==0)
//		{
//			send_odom_ultra_antiColBar_check_cmd(antiCollisionBar_CmdId);
//		}
//		
//		if(number%4==0)
//		{
//			send_driver_exception_check_cmd(0);	//left-side wheel
//			send_driver_exception_check_cmd(1);	//right-side wheel
//		}
		if(number>10000)
		{
			number=0;
		}
		std::cout<<std::endl;
		ros::spinOnce();                 
		loop_rate.sleep();
	}
	close(udp_sock);
    return 0;
}
