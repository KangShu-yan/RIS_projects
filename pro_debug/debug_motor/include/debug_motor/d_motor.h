#ifndef DEBUG_MOTOR_D_MOTOR_H
#define DEBUG_MOTOR_D_MOTOR_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <std_msgs/String.h>
#include <sys/time.h>
#include <unistd.h>  //usleep us
#include <cstdlib>
//#include <stdio.h>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include "ws_tcp/wrap_tcp.h"
// Custom message includes. Auto-generated from msg/ directory.
//#include <node_example/NodeExampleData.h>
#include <debug_motor/debugMotorData.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <debug_motor/debugMotorConfig.h>

namespace debug_motor
{
class DebugMotor
{
public:
	//! Constructor.
	explicit DebugMotor(ros::NodeHandle nh);
	//! Turn on publisher.
	void start();
private:
	//! Callback function for dynamic reconfigure server.
	void configCallback(debug_motor::debugMotorConfig &config, uint32_t level);
  	//! Timer callback for publishing message.
	void timerCallback(const ros::TimerEvent &event);
  	//This function would be triggered after read bytes ,
	char analyse_data();
 	//Used to send command to adjust parameter
	void send_data();
	//batch send command
	void batch_send();
	void send_bit_with_response(int page_index,int pos_index,int32_t set_val,int nth_bit);
	void send_nbyte_with_response(unsigned char operated_object,int page_index,int pos_index,int32_t set_val,char target);
	//Pulishs  message 
	void pub_msg();
	void syschronic_frame(void);
	//Controls motor with CAN protocol
	void control_motor();
	//Main process
	void spin();
	//Sends shake hand frame and identification frame
	void send_shake_id_cmd(std_msgs::String str);
	//Sends frame for operating value of certain bit  
	void send_bit_cmd(int page_index,int pos_index,int32_t set_val,int nth_bit);
	//Sends byte,word,and long word  
	void send_nbyte_cmd(unsigned char operated_object,int page_index,int pos_index,int32_t set_val);
	//	template<typename VST1,typename VST2>
	//	void crc(const VST1 frame,const VST2 length,unsigned char& sum);
	void analyse_CAN_data(unsigned char *frame,int size);
	//Checks sum
	void crc(const unsigned char *frame,const int length,unsigned char* sum);
	//Prints an frame
	void printf_frame(const unsigned char* frame,const int length);
	//Confirm whether frame was sent successfully
	bool confirm_send(const unsigned char* frame,const int size);
  	//! Turns off publisher.
  	void stop();
  	//! ROS node handle.
 	ros::NodeHandle nh_;
  	//! The timer variable used to go to callback function at specified rate.
  	ros::Timer timer_;
  	//! Message publisher.
  	ros::Publisher pub_;
  	//! Dynamic reconfigure server.
  	dynamic_reconfigure::Server<debug_motor::debugMotorConfig> dr_srv_;
  	//! USB message.
  	std::string shake_id_string_,dev_;
  	//! framerate_ is the sleep rate of main process ,baud_rate_ is the baud rate of USB
	int framerate_,baud_rate_;
	//object for calibration 
	int a_,b_,oper_object_,read_write_,page_index_,pos_index_,nth_bit_;
	//calibration value 
	int32_t set_val_;
	//Parameters for motor control 
	int set_speed_rpm_,set_torque_Nm_,control_mode_,motor_id_;
	//Enable variables
	bool send_CAN_,send_params_,batch_read_;
  	//! USB write and read
	serial::Serial ros_ser_;
  	//! USB received data 
	std_msgs::String received_data_,send_data_;
//	float motor_rpm_[6];
//	float motor_Nm_[6];
//	float motor_break_code_[6];
//	float motor_tempoc_[6];
//	float motor_odom_er_[6];
	//TCP
 	
 	
	int64_t get_CAN_counter_,send_CAN_counter_;
	int32_t set_motor_rpm_[6];
	int32_t set_motor_Nm_[6];
	double motor_rpm_[6];	//
	double motor_Nm_[6];
	int32_t motor_break_code_[6];
	double motor_tempoc_[6];
	uint8_t mdriver_tempoc_[6];
	uint16_t motor_odom_er_[6];
	
	unsigned char params_[1024][8];
	unsigned char params_index_;
	//	unsigned char received_frame[100];
	std::mutex m_mutex;
	std::ofstream outfile;
	std::ifstream infile;
	
};
union int32_uchar
{
	int32_t val;
	unsigned char buf[4];
};
}

#endif  // 	DEBUG_MOTOR_H
