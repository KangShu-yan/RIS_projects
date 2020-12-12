#ifndef WS_MOTOR_WS_MOTOR_H
#define WS_MOTOR_WS_MOTOR_H
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
#include <eigen3/Eigen/Geometry> 	//eigen
#include "ws_tcp/wrap_tcp.h"  //tcp_client is the directory contained in include directory 
#include <ws_motor/wsMotorData.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

namespace ws_motor
{
class WSMotor
{
public:
	//! Constructor.
	explicit WSMotor(ros::NodeHandle nh);
	//! init in-wheel motor
	void init_motor();
private:
	//callback function of topic /cmd_vel 
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	//callback function of topic /sensor_msgs/Imu
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  	//! Timer callback for publishing message.
  	//publish states of in-wheel motor and their driver
	void statePubTimerCallback(const ros::TimerEvent &event);
	//定时发布里程计
	void odomPubTimerCallback(const ros::TimerEvent &event);
 	//下发电机指令
	void send_motor_cmd(int idx,int16_t control_mode);
	//分析上传的电机数据
	void analyse_motor_data(unsigned char *frame,int size);
	//由电机及Imu检测到的数据计算twist
	void get_twist();
	//电机同步帧
	void syschronic_frame(void);
	//Controls motor with CAN protocol
	void control_motor();
	//Main process
	void motor_spin_once();
	//Prints an frame
	void printf_frame(const unsigned char* frame,const int length);
  	//! Turns off motor.
  	void stop_motor();
  	//! ROS node handle.
 	ros::NodeHandle nh_;
 	//主程序的循环频率 hz
 	double framerate_;
 	//主程序循环参数
  	ros::Rate loop_rate_;
 	//声明 tcp
 	tcp_client chassis_client_;
  	//! The timer variable used to go to callback function at specified rate.
  	//时间变量
  	ros::Time current_time_,last_time_;
  	//定时器
  	ros::Timer state_pub_timer_,odom_pub_timer_;
  	//! Message publisher.
  	ros::Publisher odom_pub_;
  	ros::Publisher state_pub_;
  	//订阅消息
  	ros::Subscriber imu_sub_;
  	ros::Subscriber twist_sub_;
	//电机控制模式 
	int control_mode_;
	//帧数据计数
	int64_t get_CAN_counter_,send_CAN_counter_;
	//
	int32_t set_motor_rpm_[6];
	int32_t set_motor_Nm_[6];
	double motor_rpm_[6];	//
	double motor_Nm_[6];
	int32_t motor_break_code_[6];
	double motor_tempoc_[6];
	uint8_t mdriver_tempoc_[6];
	uint16_t motor_odom_er_[6];
	double yaw_angle_;
	double pos_x_,pos_y_,th_rad_;
	double vel_x_,vel_y_,vel_th_radps_;
	double acc_x_,acc_y_,acc_th_;
	//thread lock
	std::mutex m_mutex_;
//	struct ws_motor
//	{
//		int32_t set_motor_Nm_[6];
//		int32_t set_motor_Nm_[6];
//		double motor_rpm_[6];
//		double motor_Nm_[6];
//		int32_t motor_break_code_[6];
//		double motor_tempoc_[6];
//		uint8_t mdriver_tempoc_[6];
//		int64_t motor_odom_er_[6]; 
//	}motor_state_;
	typedef enum motor_mode
	{
		POWER_ON=1,
		SPEED_CONTROL,
		TORQUE_CONTROL,
		POWER_OFF
	}motor_mode;
	
};
union int32_uchar
{
	int32_t val;
	unsigned char buf[4];
};

}

#endif  // 	DEBUG_MOTOR_H
