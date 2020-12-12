#ifndef PLC_TEST_PLC_TEST_H
#define PLC_TEST_PLC_TEST_H

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
#include <cmath>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include "ws_tcp/wrap_tcp.h"
// Custom message includes. Auto-generated from msg/ directory.
//#include <node_example/NodeExampleData.h>
#include <plc_test/testPlcData.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <plc_test/plcTestConfig.h>

#define pos2angle(l1,l2,set_y) acos((l1*l1+l2*l2-set_y*set_y)/(2*l1*l2))

namespace test_plc
{
class TestPlc
{
public:
	//! Constructor.
	explicit TestPlc(ros::NodeHandle nh);
	//! Turn on publisher.
	void start();
private:
	//! Callback function for dynamic reconfigure server.
	void configCallback(plc_test::plcTestConfig &config, uint32_t level);
  	//! Timer callback for publishing message.
	void timerCallback(const ros::TimerEvent &event);
	void contorlTimerCallback(const ros::TimerEvent &event);
  	//This function would be triggered after read bytes ,
	void syschronic_frame();
	
	//Pulishs  message 
	void pub_msg();
	//Controls suspension with user protocol
	void control_plc(unsigned char power_state);
	//Main process
	void spin();
	//Sends shake hand frame and identification frame

	void analyse_frame(unsigned char *frame,int size);
	
	//Prints an frame
	void printf_frame(const unsigned char* frame,const int length);

  	//! Turns off publisher.
  	void stop();
  	//! ROS node handle.
 	ros::NodeHandle nh_;
  	//! The timer variable used to go to callback function at specified rate.
  	ros::Timer timer_;
  	ros::Timer control_timer_;
  	//! Message publisher.
  	ros::Publisher pub_;
  	//! Dynamic reconfigure server.
  	dynamic_reconfigure::Server<plc_test::plcTestConfig> dr_srv_;
  	//! USB message.
  	std::string shake_id_string_,dev_;
  	//! framerate_ is the sleep rate of main process ,baud_rate_ is the baud rate of USB
	double framerate_;
	double a_;
	//Parameters for motor control 
	int plc_control_mode_;
	//Enable variables
	bool send_confirm_;
  	//! USB write and read
	serial::Serial ros_ser_;
  	//! USB received data 
	std_msgs::String received_data_,send_data_;
	//TCP
 	tcp_client chassis_client_;
	int64_t get_counter_,send_counter_,get_angle_counter_;

	uint16_t limited_speed_[3];
	uint16_t limited_position_[3];
	int16_t set_angle_[6];
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
union int16_uchar
{
	int32_t val;
	unsigned char buf[2];
};

}

#endif  // 	SUSPENSION_TEST_H
