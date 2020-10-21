/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "std_msgs/String.h"
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <sstream>

#include "sixwheel_chassis_control/msg_chassis.h"
#include "tcp_client/class_tcp.h"  //tcp_client is the directory contained in include directory 
#define MAXLEN 200

ros::Publisher moto_chassis_pub;
tcp_client PLC_client("192.168.3.250",5001,0);
tcp_client chassis_client("192.168.1.10.",4001,0);
typedef enum
{
	FALSE=0,TRUE
}Bool;
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
	
//		int16_t	 	speed_rpm;       //转速
		std_msgs::Int16	 	speed_rpm; 
	
		int16_t  	real_current;    //转速
	
		int16_t  	given_current;   //实际的转矩电流
		uint8_t  	Temp;           //温度

		uint16_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
	
		uint8_t		buf_idx;
//		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;
enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
	
    POSITION_PID,
    DELTA_PID,
};
struct pid_struct 
{
    float p;
    float i;
    float d;
    
    float set[3];				//
    float get[3];				//
    float err[3];				//
	    
    float pout;					//
    float iout;					//
    float dout;					//
    
//    float pos_out;	 			//
	std_msgs::Float32 pos_out;
    float last_pos_out;			//
    float delta_u;				//
    float delta_out;			//
    float last_delta_out;
    
	float max_err;
	float deadband;			//err < deadband return
    uint32_t pid_mode;
//    uint32_t MaxOutput;			//
//    uint32_t IntegralLimit;		//
    
   	float MaxOutput;			//
    float IntegralLimit;	
   
};

moto_measure_t moto_chassis[6];
pid_struct pid_spd[6];
//int16_t set_spd[6]={0}; 
std_msgs::Int16 set_spd[6];
Bool send_cmd_enable;
char feedback_data[100];
char tcp200_data[13]={0},tcp1FF_data[13]={0};
///Last error code.
int err;
///Last error message.
char err_msg[1024];
///Status of the connection.
bool connected;
int sock;
///Configuration structure
struct sockaddr_in server;
///Return value for functions
int ret;
///Asynchronous comm
bool async;
/**********************************
tcp/ip
client
connect
receive
send
*********************************/
//tcp_client tcp_can_client("192.168.1.10",4001,1);
void tcp_client(const char*ip, int port, bool async_comm)		//服务端地址192.168.1.10  服务端端口号4001  异步
{
    printf("tcp_client\n");
    err=0;//Clear the error variable

    connected=false;
	//创建TCP套接字
    sock = socket(AF_INET, SOCK_STREAM, 0);	//ip地址类型AF_INET 表示 IPv4 地址，例如 127.0.0.1；AF_INET6 表示 IPv6 地址，例如 1030::C9B4:FF12:48AA:1A2B
														// SOCK_STREAM（流格式套接字/面向连接的套接字） 和 SOCK_DGRAM（数据报套接字/无连接的套接字）
														// IPPROTO_TCP 和 IPPTOTO_UDP ,0
	//sock<0出错  sock=0连接关闭  sock>0接收到数据大小
	if(sock<0)											//
    {
        sprintf(err_msg,"Cannot open socket");	
        err=-1;
        return;
    }
	
    async=async_comm;

    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));       /* Clear structure */
    server.sin_family = AF_INET;                  /* Internet/IP */
    server.sin_addr.s_addr = inet_addr(ip);  /* IP address */
    server.sin_port = htons(port);       /* server port */
    
}
int Connect(void)
{
    
    err=0;
	//connect(int 套接字描述符,struct sockaddr * 指向套接字地址结构的指针，socklen_t 指向套接字地址结构的大小)
    ret=connect(sock, (struct sockaddr *) &server, sizeof(server)); 
	//返回0成功，-1时出错
    if(ret<0)
    {
        sprintf(err_msg,"Failed to connect with server");
        
        
        err=-2;
        return -1;
    }

    if(async)
    {
        fcntl(sock,F_SETOWN,getpid());
        fcntl(sock,F_SETFL,O_ASYNC);
    }

    connected=true;
	printf("Connected\n");
    return 0;
}
int Send(char*data, int size)
{
   
//    for(int i=0;i<13;i++){
//		printf("%2.2x ",data[i]);
//	}
//	printf("\n");
	
    if(!connected)
    {
        sprintf(err_msg,"Connection not established");
        err=-4;
        return -1;
    }

    err=0;
    errno=0;
    ret=send(sock,data,size,0);
	if(ret)
	{
		printf("Send ok!\n");
	}
    if(errno==EPIPE)
    {
        sprintf(err_msg,"Pipe broke");
        err=-4;
        return -1;
    }else if(ret<size)
    {
        sprintf(err_msg,"Mismatch in number of sent bytes");
        err=-4;
        return -1;
    }else if(errno!=0)
    {
        sprintf(err_msg,"Alarm! error not caught");
        perror("send");
        err=-3;
        return -1;
    }
	
    return 0;
}

int Receive(char*data, int size, bool peek, int*number, int flags)
{
    if(!connected)
    {
        sprintf(err_msg,"Connection not established");
        err=-4;
        return -1;
    }

    err=0;

    //Get the current time stamp
// 	timestamp=carmen_get_time();

    //If peek mode is selected we don't erase the information from the buffer
    if(peek)
    {
        ret=recv(sock,data,size, MSG_PEEK);
        if(ret<0)
        {
            sprintf(err_msg,"Failed to peek data from server");
            err=-5;
            return -1;
        }

        return 0;
    }

    //Normal read
    ret=recv(sock,data,size,flags);
    if(ret<0)
    {
        sprintf(err_msg,"Failed to receive bytes from server");
        err=-5;
        return -1;
    }else if(number!=NULL)
    {
        *number=ret;
    }
//	printf("Receive\n");
    return 0;
}
/************************
pid init
input:
output:

***********************/

void abs_limit(float* curr_val,float MaxOutput)
{
	if(*curr_val>MaxOutput)
	{
		*curr_val=MaxOutput;
	}
	else if (*curr_val< -MaxOutput)
	{
		*curr_val=-(float)MaxOutput;
	}
	else;
}
float ABS(float val)
{
	if(val>0)
	{
		return val;
	}
	else
	{
		return -val;
	}
}
void min_limit(float* curr_val,float MinOutput)
{
	if(*curr_val<MinOutput)
	{
		*curr_val=0.0;
	}
	else if (*curr_val> -MinOutput)
	{
		*curr_val=0.0;
	}
	else;
}
void PID_struct_init( pid_struct *pid, uint32_t mode,uint32_t maxout,uint32_t intergral_limit,
    float 	kp,float 	ki,float 	kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}

float pid_calc(pid_struct* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
  if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //Î»ÖÃÊ½p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out.data = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out.data), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out.data;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//ÔöÁ¿Ê½P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out.data : pid->delta_out;
}
/****************************************************************************
*	function:
		decode_moto_info:decode the frame datas from GCAN
*	input:
*		rec_data:received frame datas from GCAN
*		number:assumed whole size of the frame datas
*	output:
*		moto_chassis[i].speed_rpm.data:current rpm of six motors
*****************************************************************************/
void decode_moto_info(char* rec_data,int number)
{
	static int16_t count=0;
	char temp[10]={0};
	int k=0;
	int m=0;
//	ROS_INFO("sizeof(rec_data) = %d",sizeof(rec_data));
	int flag1,flag2,flag3,flag4,flag5,flag0;
	flag0=0;
	flag1=0;
	flag2=0;
	flag3=0;
	flag4=0;
	flag5=0;
	count=0;
//	ROS_INFO("I received feedback_data");
//	for(int i=0;i<number;i++)
//	{
//		printf("%2.2x ",rec_data[i]);
//	}
//	printf("\n");
	
	for(int i=0;i<number;i++)
	{
		if(rec_data[i]==0x08)
		{
			temp[k++]=i;
		}
	}
	
//	for(int i=0;i<7;i++)
//	{
//		printf("temp %d",i);printf(": %2.2x  ",temp[i]);
//	}
//	printf("\n");
//		if(rec_data[i]==0x08)
//		{
//			j++;
////			if(rec_data[i+3]==0x01&&moto_chassis_0==FALSE)
	for(int i=0;i<10;i++)
	{
		if(rec_data[temp[i]+4]==0x01&&flag0==0)
		{
			moto_chassis[0].speed_rpm.data = (rec_data[temp[i]+7]*256+rec_data[temp[i]+8]);
//			printf("rec_data_1_7:%2.2x, rec_data_1_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			flag0=1;
			
		}
		else if(rec_data[temp[i]+4]==0x02&&flag1==0)
		{
				
			moto_chassis[1].speed_rpm.data = rec_data[temp[i]+7]*256+rec_data[temp[i]+8];
//			printf("rec_data_2_7:%2.2x, rec_data_2_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			flag1=1;
			
		}
		else if(rec_data[temp[i]+4]==0x03&&flag4==0)
		{
			flag4=1;	
			moto_chassis[4].speed_rpm.data = rec_data[temp[i]+7]*256+rec_data[temp[i]+8];
//			printf("rec_data_5_7:%2.2x, rec_data_5_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			
			
		}
		else if(rec_data[temp[i]+4]==0x04&&flag5==0)
		{
			flag5=1;
			moto_chassis[5].speed_rpm.data = rec_data[temp[i]+7]*256+rec_data[temp[i]+8];
//			printf("rec_data_6_7:%2.2x, rec_data_6_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			
		}
		else if(rec_data[temp[i]+4]==0x06&&flag2==0)
		{
			flag2=1;
			moto_chassis[2].speed_rpm.data = rec_data[temp[i]+7]*256+rec_data[temp[i]+8];
//			printf("rec_data_3_7:%2.2x, rec_data_3_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			
		}
		else if(rec_data[temp[i]+4]==0x07&&flag3==0)
		{
			
			flag3=1;	
			moto_chassis[3].speed_rpm.data = rec_data[temp[i]+7]*256+rec_data[temp[i]+8];
//			printf("rec_data_4_7:%2.2x, rec_data_4_8:%2.2x\n",rec_data[temp[i]+7],rec_data[temp[i]+8]);
			
		}
		else;
	}
			
	
}
/****************************************************************************
*	function:
		decode_suspension_info:decode the frame datas from GCAN
*	input:
*		rec_data:received frame datas from PLC
*		number:assumed whole size of the frame datas
*	output:
*		moto_chassis[i].speed_rpm.data:current rpm of six motors
*****************************************************************************/
void decode_suspension_info(char* rec_data,int number)
{
	static int16_t count=0;
	char temp[10]={0};
	int j=0;
//	ROS_INFO("sizeof(rec_data) = %d",sizeof(rec_data));
	int flag1,flag2,flag3,flag4,flag5,flag0;
	flag0=0;
	flag1=0;
	flag2=0;
	flag3=0;
	flag4=0;
	flag5=0;
	count=0;
	for (int i = 0; i < 20;i++)
	{
		if (0x34 == rec_data[i] && 0x12 == rec_data[i++])
		{
			j = i;
		}
	}
	ROS_INFO("I received plc_data : ");
	for (int i = j; i < j+8; i++)
	{
		printf("rec_data["); printf("%d", i); printf("] = "); printf("%2.2x\n", (unsigned char)rec_data[i]);
	}
	
}
/****************************************************************************
*	function:
		pid_control:pid control and send  cmds based on GCAN(TCP-CAN module) to driver of motor 
*	input:
*		moto_chassis[i].speed_rpm.data:feedback_data of motor,from function of decode_moto_info
*		set_spd[i].data:datas from joystick,from function Callback 
*	output:
*		pid_spd[i].pos_out.data:pid output value 
*		tcp200_data[m]:cmd frame 
*		tcp1FF_data[m]:cmd frame 
*****************************************************************************/
void pid_control(void)
{
	for(int i=0; i<6; i++)
	{ 
		pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm.data, set_spd[i].data);
//		min_limit(&pid_spd[i].pos_out.data,50.0);
	}
	char m=0,n=0;
//	std::cout<<std::endl;
	
	tcp200_data[m++]=0x08;
	tcp200_data[m++]=0x00;
	tcp200_data[m++]=0x00;
	tcp200_data[m++]=0x02;
	tcp200_data[m++]=0x00;
	
	tcp1FF_data[n++]=0x08;
	tcp1FF_data[n++]=0x00;
	tcp1FF_data[n++]=0x00;
	tcp1FF_data[n++]=0x01;
	tcp1FF_data[n++]=0xFF;
	tcp200_data[m++]=pid_spd[0].pos_out.data/256;
	tcp200_data[m++]=pid_spd[0].pos_out.data;
	tcp200_data[m++]=pid_spd[1].pos_out.data/256;
	tcp200_data[m++]=pid_spd[1].pos_out.data;
	tcp200_data[m++]=pid_spd[4].pos_out.data/256;
	tcp200_data[m++]=pid_spd[4].pos_out.data;
	tcp200_data[m++]=pid_spd[5].pos_out.data/256;
	tcp200_data[m++]=pid_spd[5].pos_out.data;
	
	tcp1FF_data[7]=pid_spd[2].pos_out.data/256;
	tcp1FF_data[8]=pid_spd[2].pos_out.data;
	tcp1FF_data[9]=pid_spd[3].pos_out.data/256;
	tcp1FF_data[10]=pid_spd[3].pos_out.data;
//	moto_chassis[0].speed_rpm.data = 0;
//	moto_chassis[1].speed_rpm.data = 0;
//	moto_chassis[2].speed_rpm.data = 0;
//	moto_chassis[3].speed_rpm.data = 0;
//	moto_chassis[4].speed_rpm.data = 0;
//	moto_chassis[5].speed_rpm.data = 0;
		
	for(int i=0;i<6;i++)
	{
		std::cout<<"pid_spd.pos["<<i<<"].pos_out.data : "<<pid_spd[i].pos_out.data<<std::endl;
	}
		
	Send(tcp200_data,13);
	Send(tcp1FF_data,13);
}

void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   	static int count=0;
	double dt=0.001;
	double B=1.604;		//
	double wheel_r=1.;//0.15

	sixwheel_chassis_control::msg_chassis msg_chassis;
	
	set_spd[0].data = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 1 
	set_spd[2].data = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 3
	set_spd[4].data = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 5
	
	set_spd[1].data = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 2
	set_spd[3].data = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 4
	set_spd[5].data = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 6
	
	ROS_INFO("I got twist.linear.x = %lf , twist.angular.z = %lf",msg->linear.x, msg->angular.z);

	msg_chassis.set_spd_info_1 = set_spd[0].data;
	msg_chassis.pid_pos_out_1 = pid_spd[0].pos_out.data;
	msg_chassis.moto_chassis_info_1 = moto_chassis[0].speed_rpm.data;
	
	msg_chassis.set_spd_info_2 = set_spd[1].data;
	msg_chassis.pid_pos_out_2 = pid_spd[1].pos_out.data;
	msg_chassis.moto_chassis_info_2 = moto_chassis[1].speed_rpm.data;
	
	
	msg_chassis.set_spd_info_3 = set_spd[2].data;
	msg_chassis.pid_pos_out_3 = pid_spd[2].pos_out.data;
	msg_chassis.moto_chassis_info_3 = moto_chassis[2].speed_rpm.data;
	
	msg_chassis.set_spd_info_4 = set_spd[3].data;
	msg_chassis.pid_pos_out_4 = pid_spd[3].pos_out.data;
	msg_chassis.moto_chassis_info_4 = moto_chassis[3].speed_rpm.data;
	
	msg_chassis.set_spd_info_5 = set_spd[4].data;
	msg_chassis.pid_pos_out_5 = pid_spd[4].pos_out.data;
	msg_chassis.moto_chassis_info_5 = moto_chassis[4].speed_rpm.data;
	
	msg_chassis.set_spd_info_6 = set_spd[5].data;
	msg_chassis.pid_pos_out_6 = pid_spd[5].pos_out.data;
	msg_chassis.moto_chassis_info_6 = moto_chassis[5].speed_rpm.data;
	

	moto_chassis_pub.publish(msg_chassis);
	
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "cmd_vel_to_current_node");
    // 创建节点句柄
    ros::NodeHandle n;
	ros::Rate loop_rate(1000);
	for(int i=0; i<6; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 5000,2.3f,0.01f,0.0f);  //4 motos angular rate closeloop.
	}
	set_spd[0].data = set_spd[1].data = set_spd[2].data = set_spd[3].data = 0;
    ros::Subscriber vel_to_current_sub = n.subscribe("/cmd_vel", 20, Callback);
    // 创建一个Publisher，发布名为/moto_chassis_pid，消息类型为mbot_teleop::msg_chasiss，队列长度10
    
    moto_chassis_pub = n.advertise<sixwheel_chassis_control::msg_chassis>("/moto_chassis_pid", 20); 
	tcp_client("192.168.1.10",4001,0); //async_comm=1;
	while(Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
	int number=0;
    while(ros::ok())
	{
		number++;
//		Receive(rec_data, 100, 0, &number, 0);
//	 	if(number>20)
//	 	{
	 	ret=recv(sock,feedback_data,100,0);
	 	if(ret<0)
	 	{
	 		ROS_WARN_STREAM_ONCE(" Received wrong!");
	 	}
		decode_moto_info(feedback_data,100);
//	 	}	
		if(number>20)
		{
			number=0;
			pid_control();
		}
		ros::spinOnce();                 
		loop_rate.sleep();
	}
    return 0;
}
