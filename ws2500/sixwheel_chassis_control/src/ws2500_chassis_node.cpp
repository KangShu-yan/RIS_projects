
/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <errno.h>
#include <iostream>
#include <sstream>
#include <pthread.h>
#include "ws2500_chassis_control/msg_chassis.h"
#include "tcp_client/class_tcp.h"  //tcp_client is the directory contained in include directory 
#define MAXLEN 200

ros::Publisher moto_chassis_pub;
//tcp_client PLC_client("192.168.3.250",5001,0);
tcp_client chassis_client("192.168.1.10.",4001,0);
pthread_mutex_t m_mutex;
typedef enum
{
	FALSE=0,TRUE
}Bool;
typedef struct{
	uint16_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
	uint16_t 	last_angle;	  //abs angle range:[0,8191]
//	int16_t	 	speed_rpm;       //转速
	int16_t	 	speed_rpm; 
	int16_t 	torque;		
//	int16_t  	real_current;    //转速
//	int16_t  	given_current;   //实际的转矩电流
	uint8_t  	motor_temp;           //温度
	uint8_t  	driver_temp;  
	uint8_t  	motor_status;           //温度
	uint8_t  	driver_status; 
	
	uint16_t	offset_angle;   //电机启动时候的零偏角度
	int32_t		round_cnt;     //电机转动圈数
	int32_t		total_angle;    //电机转动的总角度
	
//	uint16_t	angle_buf[FILTER_BUF_LEN];
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
union int16Conchar2
{
	uint16_t data;
	char buf[2];
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
    float pos_out;	 			//
//	std_msgs::Float32 pos_out;
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
struct Power_info
{
	unsigned char status;
	uint16_t voltage;
	uint16_t current;
	int16_t temperature;
	unsigned char capacity;
};
Power_info power;
moto_measure_t moto_chassis[6];
pid_struct pid_spd[6];
//int16_t set_spd[6]={0}; 
int16_t set_spd[6];
Bool send_cmd_enable;
char feedback_data[100];
char tcp200_data[13]={0},tcp1FF_data[13]={0};

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
    
    if(pid->pid_mode == POSITION_PID) //pos_pid
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//delta_pid
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
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}
/**
 *	@ analyse frames related to motor and power
 *		decode_moto_info:decode the frame datas from GCAN
 *	input:
 *		rec_data:received frame datas from GCAN
 *		number:assumed whole size of the frame datas
 *	output:
 *		moto_chassis[i].speed_rpm.data:current rpm of six motors
**/
void analyse_frame(char* buf,int len)
{
	static int16_t count=0;
	for(int i=0;i<len;i++)
	{	
		if(buf[i]==0x08&&buf[i+3]==0x03&&buf[i+4]==0x81)	// 解析电池反馈的数据
		{
			ROS_INFO("\033[32mGot power info\033[0m");
			power.status=buf[i+5];
			power.voltage=((buf[i+6]<<8)|buf[i+7])*100.0;	//volt
			power.current=((buf[i+8]<<8)|buf[i+9])*100.0;	//ampere
			power.temperature=((buf[i+6]<<10)|buf[i+11])*10.0;	//celsius degree
			power.capacity=buf[i+12];
			break;
		}
		else if(buf[i]==0x08&&buf[i+3]==0x71)				//解析电机反馈的数据
		{
			if(1<=buf[i+4]&&buf[i+4]<=8)
			{
				int motor_id=buf[i+4];
				ROS_INFO("\033[32mGot %dth motor info\033[0m",motor_id);
				moto_chassis[motor_id-1].speed_rpm=((buf[i+5]<<8)|buf[i+6])*100.0;	//rpm
				moto_chassis[motor_id-1].torque=((buf[i+7]<<8)|buf[i+8])*100.0;		//Nm
				moto_chassis[motor_id-1].motor_temp=buf[i+9]*10.0;					//celsius degree
				moto_chassis[motor_id-1].driver_temp=buf[i+10]*10.0;
				moto_chassis[motor_id-1].motor_status=buf[i+11];
				moto_chassis[motor_id-1].driver_status=buf[i+12];
			}
		}
		else;
	}	
}

/**
 *	@function:
		pid_control:pid control and send  cmds based on GCAN(TCP-CAN module) to driver of motor 
 *	input:
*		moto_chassis[i].speed_rpm.data:feedback_data of motor,from function of decode_moto_info
*		set_spd[i].data:datas from joystick,from function Callback 
*	output:
*		pid_spd[i].pos_out.data:pid output value 
*		tcp200_data[m]:cmd frame 
*		tcp1FF_data[m]:cmd frame 
**/
void pid_control(void)
{
	pthread_mutex_lock(&m_mutex);
	for(int i=0; i<6; i++)
		pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		
	pthread_mutex_unlock(&m_mutex);	
	char send_buf[100]={0};
	for(int motor_idx=1;motor_idx<=6;motor_idx++)	//6个电机
	{
		for(int idx=(motor_idx-1)*13;idx<motor_idx*13;)	//每一帧长度为13
		{
			send_buf[idx++]=0x08;
			send_buf[idx++]=0x00;
			send_buf[idx++]=0x00;
			send_buf[idx++]=0x44;
			send_buf[idx++]=motor_idx;					//电机ID
			send_buf[idx++]=(int16_t)pid_spd[motor_idx-1].pos_out>>8;	//速度
			send_buf[idx++]=(int16_t)pid_spd[motor_idx-1].pos_out;
			send_buf[idx++]=0;								//转矩
			send_buf[idx++]=0;
			send_buf[idx++]=0;								//备用
			send_buf[idx++]=0;
			send_buf[idx++]=0x55;							//0x55 速度模式0xaa 转矩模式,其余无效
			send_buf[idx++]=0;
		}
	}
	chassis_client.Send(send_buf,78);
	for(int i=0;i<6;i++)
		std::cout<<"pid_spd.pos["<<i<<"].pos_out.data : "<<pid_spd[i].pos_out<<std::endl;
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   	static int count=0;
	double dt=0.001;
	double B=1.604;		//
	double wheel_r=1.;//0.15

	ws2500_chassis_control::msg_chassis msg_chassis;
	pthread_mutex_lock(&m_mutex);
	set_spd[0] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 1 
	set_spd[2] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 3
	set_spd[4] = (msg->linear.x-B/2*msg->angular.z)/wheel_r; //wheel 5
	
	set_spd[1] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 2
	set_spd[3] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 4
	set_spd[5] = -(msg->linear.x+B/2*msg->angular.z)/wheel_r; //wheel 6
	pthread_mutex_unlock(&m_mutex);
	ROS_INFO("I got twist.linear.x = %lf , twist.angular.z = %lf",msg->linear.x, msg->angular.z);

	msg_chassis.set_spd_info_1 = set_spd[0];
	msg_chassis.pid_pos_out_1 = pid_spd[0].pos_out;
	msg_chassis.moto_chassis_info_1 = moto_chassis[0].speed_rpm;
	
	msg_chassis.set_spd_info_2 = set_spd[1];
	msg_chassis.pid_pos_out_2 = pid_spd[1].pos_out;
	msg_chassis.moto_chassis_info_2 = moto_chassis[1].speed_rpm;
	
	msg_chassis.set_spd_info_3 = set_spd[2];
	msg_chassis.pid_pos_out_3 = pid_spd[2].pos_out;
	msg_chassis.moto_chassis_info_3 = moto_chassis[2].speed_rpm;
	
	msg_chassis.set_spd_info_4 = set_spd[3];
	msg_chassis.pid_pos_out_4 = pid_spd[3].pos_out;
	msg_chassis.moto_chassis_info_4 = moto_chassis[3].speed_rpm;
	
	msg_chassis.set_spd_info_5 = set_spd[4];
	msg_chassis.pid_pos_out_5 = pid_spd[4].pos_out;
	msg_chassis.moto_chassis_info_5 = moto_chassis[4].speed_rpm;
	
	msg_chassis.set_spd_info_6 = set_spd[5];
	msg_chassis.pid_pos_out_6 = pid_spd[5].pos_out;
	msg_chassis.moto_chassis_info_6 = moto_chassis[5].speed_rpm;

	moto_chassis_pub.publish(msg_chassis);
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "ws2500_chassis");
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
//    n.param<>("x",p_x, 0.3);
//    nh_private.param<int>("/x",p_x, 0.3);
	ros::Rate loop_rate(1000);
	for(int i=0; i<6; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 5000,2.3f,0.01f,0.0f);  //4 motos angular rate closeloop.
	}
	
    ros::Subscriber vel_to_current_sub = nh.subscribe("/cmd_vel", 20, CmdVelCallback);
    // 创建一个Publisher，发布名为/moto_chassis_pid，消息类型为mbot_teleop::msg_chasiss，队列长度10
    
    moto_chassis_pub = nh.advertise<ws2500_chassis_control::msg_chassis>("/ws2500_chassis/moto_chassis_pid", 20); 
	//tcp_client("192.168.1.10",4001,0); //async_comm=1;
	while(chassis_client.Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
	int byte_len=0,while_count=0;
    while(ros::ok())
	{
		while_count++;
		if(while_count>20)
		{
			while_count=0;
			pid_control();
			if(chassis_client.Receive(feedback_data, 100, 0, &byte_len, 0)==1)
		 		analyse_frame(feedback_data,byte_len);	
		 	else
				ROS_WARN_STREAM_ONCE(" Received wrong!");
		}
		ros::spinOnce();                 
		loop_rate.sleep();
	}
    return 0;
}
