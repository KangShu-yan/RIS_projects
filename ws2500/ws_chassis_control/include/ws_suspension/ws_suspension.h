#ifndef WS_SUSPENSION_WS_SUSPENSION_H
#define WS_SUSPENSION_WS_SUSPENSION_H

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
#include "ws_tcp/wrap_tcp.h"  //tcp_client is the directory contained in include directory 
#include <ws_motor/wsSuspensionData.h>

#include <geometry_msgs/Twist.h>

#define pos2angle(l1,l2,set_y) acos((l1*l1+l2*l2-set_y*set_y*set_y)/(2*l1*l2))

namespace ws_suspension
{
class WSSuspension
{
public:
	//! Constructor.
	explicit WSSuspension(ros::NodeHandle nh);
	//! Turn on publisher.
	void init_suspension();
private:
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  	//! Timer callback for publishing message.
	void statePubTimerCallback(const ros::TimerEvent &event);
	void contorlTimerCallback(const ros::TimerEvent &event);
 	//Used to send command to adjust parameter
	void send_suspension_data(unsigned char state);
	void analyse_data(unsigned char *frame,int size);
	//Controls motor with CAN protocol
	void control_motor();
	//Main process
	void spin();
	//Prints an frame
	void printf_frame(const unsigned char* frame,const int length);
	
  	//! Turns off publisher.
  	void stop();
  	//! ROS node handle.
 	ros::NodeHandle nh_;
 	double framerate_;
 	//TCP
 	tcp_client suspension_client_;
  	//! The timer variable used to go to callback function at specified rate.
  	ros::Time current_time_,last_time_;
  	ros::Timer state_pub_timer_;
  	ros::Timer control_timer_;
  	//! Message publisher.
  	ros::Publisher state_pub_;
  	
	//Parameters for motor control 
	int control_mode_;
	//Enable variables
	
	int64_t get_angle_counter_,get_counter_,send_counter_;
	int16_t set_angle_[6];
	uint16_t limited_speed_[3];
	uint16_t limited_position_[3];
	
	int16_t set_pos_[6];
	int16_t set_current_[6];	//给比例换向阀
	int16_t set_pressure_[6];
	int16_t get_angle_[6];
	int16_t get_pressure_[6];
	bool emergency_stop_enable_;
	int16_t motor_control_power_;
	int16_t motor_main_power_;
	int16_t get_angle_origin_[6];
	int16_t get_pos_origin_[6];
	unsigned char brake_pos_limit_[2];
	int plc_control_mode_;
	bool send_confirm_;
	//thread lock
	std::mutex m_mutex_;
	
};
union int32_uchar
{
	int32_t val;
	unsigned char buf[4];
};
union int16_uchar
{
	int16_t val;
	unsigned char buf[2];
};

}

#endif  // 	DEBUG_MOTOR_H
