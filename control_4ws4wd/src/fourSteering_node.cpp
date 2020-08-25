
/**
 * 
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 
#include <stdbool.h>
#include <iostream>
#include <sstream>

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

/**@brief
  *@send_motion_cmd function : issue motion command to chassis  
  * 
  *@input :   
  *			linear_x: velocity along axis x
  *			angular_z: angular velocity along axis z
  *@output : 
  *			  	 	   
**/
void send_motion_cmd(float linear_x=0,float angular_z=0)
//void encode_four_steering_info(void)
{
	int ret = 0;
	unsigned short int crc = 0;
	int len = 0;
 	unsigned char *send_buf = encode_motion_cmd(linear_x,angular_z,motion_state);
 	
	ret = sendto(udp_sock, send_buf,motion_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
	
}
/**@brief
  *@send_odom_ultra_antiColBar_check_cmd function : issue odometry ,ultrasonic and anti-collision bar command to chassis  
  * 
  *@input :   
  *			cmdId: odometry command id ,ultrasonic  command id or anti-collision bar command id
  *@output : 
  *			  
  *		 	   
**/
void send_odom_ultra_antiColBar_check_cmd(unsigned short int cmdId)
{
	int ret = 0;
	unsigned short int crc = 0;
	int len = 0;
 	unsigned char *send_buf = encode_odom_ultra_antiColBar_cmd(cmdId);
	ret = sendto(udp_sock, send_buf, odom_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}
/**@brief
  *@send_ultra_antiColBar_brake_cmd function : issue brake command related to ultrasonic and anti-collision bar to chassis  
  * 
  *@input :   
  *			cmdId: ultrasonic brake command id or anti-collision bar command id
  *			cmd : 0 as default ,enable or disable 
  *@output : 
  *			  
  *		 	   
**/
void send_ultra_antiColBar_brake_cmd(unsigned short int cmdId,unsigned char cmd=0)
{
	int ret = 0;
	
	//unsigned char cmd = 1;
 	unsigned char *send_buf = encode_ultra_antiColBar_brake_cmd(cmdId,cmd);
	ret = sendto(udp_sock, send_buf, ultra_antiCol_brake_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}
/**@brief
  *@send_driver_exception_check_cmd function : issue motor driver checked command to chassis  
  * 
  *@input :   
  *			 driverSide: left side motor or right side motor
  *@output : 
  *			  
  *		 	   
**/
void send_driver_exception_check_cmd(unsigned char driverSide)
{
	int ret = 0;
	
	
 	unsigned char *send_buf = encode_driver_exception_cmd(driverSide);
	ret = sendto(udp_sock, send_buf, driver_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	
	delete[] send_buf;
}

/**
  *@ send_led_cmd function : issue led control command to chassis  
  * 
  *@ input :   
  *			 
  *@ output : 
  *			  
  *		 	   
**/
void send_led_cmd(void)
{
	int ret = 0;
	unsigned char *send_buf ;
//	for(int i=0;i<=10;i++)
//	{
//		led_control_para.channel=i;
		send_buf = encode_led_cmd(led_control_para);
		ret = sendto(udp_sock, send_buf, led_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
		
//	}
// 	unsigned char *send_buf = encode_led_cmd(led_control_para);
//	ret = sendto(udp_sock, send_buf, led_CmdLen, 0, (struct sockaddr *)&src_addr,sizeof(src_addr)); 
	std::cout<<std::endl;
	for(int i=0;i<led_CmdLen;i++)
	{
		printf("%.2x ",send_buf[i]);
	}
	std::cout<<std::endl;
	delete[] send_buf;
}

/**
  *@ param_init function : init basic parameters of joystick  
  * 
  *@ input :   
  *			nh_: node handler  
  *@ output : 
  *			  
  *		 	   
**/
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
/**
  *@ udp_init function : init udp communication  
  * 
  *@ input :   
  *			 
  *@ output : 
  *			  
  *		 	   
**/
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
/**
  *@ run function : cyclic function in main function   
  * 
  *@ input :   
  *			steering_feedback_data : recevied frame data from chassis
  *			recv_length : length of frame data 
  *@ output : 
  *			  
  *			
  *  	   
**/
void run(unsigned char* steering_feedback_data ,short int &recv_length)
{
	int ret=0;
	static int number=0;
	number++;
	if(recv_length>0)
	 {
	 	printf("motion_state : %.2x \t   vel_gear ：%.2x \n",motion_state,vel_gear);
	 	printf("[%s:%d]",inet_ntoa(src_addr.sin_addr),ntohs(src_addr.sin_port));//打印消息发送方的ip与端口号
	 
	 	std::cout << "Received " << number << "th data (with length "<< recv_length<<")："<<std::endl;
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
	//		send_ultra_antiColBar_brake_cmd(ultrasonicBrake_CmdId);
		}
	 }
	 else
	{
		std::cout << "No recevied data !" << std::endl;
	}
		
	if(number%5==0)
	{
		send_odom_ultra_antiColBar_check_cmd(odometry_CmdId);		//
	}
	if(number%7==0)
	{
		send_odom_ultra_antiColBar_check_cmd(ultrasonic_CmdId);
			
	}
	if(number%9==0)
	{
		send_odom_ultra_antiColBar_check_cmd(antiCollisionBar_CmdId);
	}
		
	if(number%4==0)
	{
		send_driver_exception_check_cmd(0);	//left-side wheel
		send_driver_exception_check_cmd(1);	//right-side wheel
	}
	if(number>10000)
	{
		number=0;
	}	
	std::cout<<std::endl;
	
}

/**
  *@ joyCallback function : subscribe joy  
  * 
  *@ input :   
  *			joy : command from operator
  *
  *@ output : 
  *			  
  *			
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
			send_led_cmd();
		}
		else;
		twist.angular.z = w_scale_*joy->axes[0]; //左摇杆左右
		twist.linear.x = l_scale_*joy->axes[1]; //左摇杆前后
	}
	else if(joy->buttons[6]||joy->buttons[7])	//LT  RT 
	{
			motion_state = 2;	//brake
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
	int number=0;
	short int recv_length=0;
	// parameters initialization of joystick
	param_init(nh_);
	//udp initialization
	if(udp_init()<0)
	{
		return -1;
	}
	//define a subscriber
	ros::Subscriber joy_sub_; 
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 200); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback); 
    while(ros::ok())
	{
		
	 	recv_length=recvfrom(udp_sock,steering_feedback_data, 1024, 0,(struct sockaddr*)&src_addr, &len);//&number
	 	run(steering_feedback_data,recv_length);
	 	
		ros::spinOnce();                 
		loop_rate.sleep();
	}
	close(udp_sock);
    return 0;
}

