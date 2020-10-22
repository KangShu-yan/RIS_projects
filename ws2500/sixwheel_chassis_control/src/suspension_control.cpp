//created by sk on 10-21-2020
/**
 *	@brief 
 * 	suspension_control
 */
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include "tcp_client/class_tcp.h"  //tcp_client is the directory contained in include directory 
#include <geometry_msgs/Twist.h>
#define MAXLEN 200

ros::Publisher susp_pub;		//information of suspension and power are published
//tcp_client tcp_can_client("192.168.1.10",4001,1);
tcp_client PLC_client("192.168.1.250",5001,0);
//tcp_client power_client("192.168.1.40",4001,0);
/***
 *	@brief 
 *	function:
		decode_suspension_info:decode the frame datas from PLC
 *	input:
 *		rec_data:received frame datas from PLC
 *		number:assumed whole size of the frame datas
 *	output:
 *		press data :
		degree data :
***/
void decode_frame(char* rec_data,int number)
{
	static int16_t count=0;
	double press_data=0.0,degree_data=0.0;
	int j=0;
	ROS_INFO("decode_suspension_info function");
	count++;
	for (int i = 0; i < 20;i++)
	{
		if (0x34 == rec_data[i] && 0x12 == rec_data[i+1])
		{
			j = i;
		}
	}
	press_data=rec_data[6]*256+rec_data[5];	//
	degree_data=rec_data[8]*256+rec_data[7]; //
	ROS_INFO("I received %dth plc_data :",count);
//	for (int i = j; i < j+8; i++)
//	{
//		printf("rec_data["); printf("%d", i); printf("] = "); printf("%2.2x\n", (unsigned char)rec_data[i]);
//	}
//	std::cout<<"press_data = "<<press_data<<"\t degree_data = "<<degree_data<<std::endl;
}
/**
*	@brief 
*	function:
		suspension_control:suspension control and send commands to PLC 
*	input:
*		
*	output:
*		sendBuffer 
**/
void suspension_control(void)
{
	char sendBuffer[8]={0};
	ROS_INFO("suspension_control function");
//	PLC_client.Send(tcp200_data,13);
}
/**
*	function:
		suspension_control:suspension control and send commands to PLC 
*	input:
*		
*	output:
*		sendBuffer 
**/
//void power_control(void)
//{
//	char sendBuffer[8]={0};
//	ROS_INFO("suspension_control function");
////	power_client.Send(tcp200_data,13);
//}

/*
*	@brief 
*	function:
		callback function for subscriber
*	input:
*		
*	output:
*		
**/
void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   	static int count=0;
	
	ROS_INFO("Callback function");
}
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "suspension_control");
    // 创建节点句柄
    ros::NodeHandle n;
	ros::Rate loop_rate(1000);  //Hz
	char feedback_data[MAXLEN] = {0};
	int number=0,byte_len=0,ret=1;
  	ros::Subscriber vel_to_current_sub = n.subscribe("/cmd_vel", 20, Callback);
//	suspension_info_pub = n.advertise<sixwheel_chassis_control::msg_chassis>("/moto_chassis_pid", 20); 
	while(PLC_client.Connect()!=0)
	{
		ROS_WARN_STREAM_ONCE("unsuccessfully!");
	}
	ROS_INFO("Connected successfully!");
    while(ros::ok())
	{
		number++;
	 	if(PLC_client.Receive(feedback_data, 100, 0, &byte_len, 0)==1)
	 		decode_frame(feedback_data,byte_len);
	 	else 
			ROS_WARN_STREAM_ONCE(" Received wrong!");
	 	
//		if(number>20)
//		{
//			number=0;
//			suspension_control();
//		}
		ros::spinOnce();                 
		loop_rate.sleep();
	}
    return 0;
}
