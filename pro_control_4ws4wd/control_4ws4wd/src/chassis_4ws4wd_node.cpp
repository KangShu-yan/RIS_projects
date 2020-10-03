
/**
 * 
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 
#include <stdbool.h>
#include <iostream>
#include <sstream>

#include "chassis_4ws4wd/chassis_4ws4wd.h"  //tcp_client is the directory contained in include directory 


double l_scale_1=0.0, w_scale_1=0.0; 
double l_scale_2=0.0, w_scale_2=0.0; 

static char motion_state = 0;  
char vel_gear = 0;
ledParam led_control_para ={0};
chassis_motion_cmd motion_cmd_para={0};
ros::Publisher vel_pub_;
void param_init(ros::NodeHandle &nh_);

/**
  *@ joyCallback function : subscribe joy  
  * 
  *@ input :   
  *			joy : command from operator
  *
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
	//twist.linear.x = max_limit(twist.linear.x,100);	//< = 100mm/s
	//twist.angular.z = max_limit(twist.angular.z,30);// 30deg/s
	
	vel_pub_.publish(twist);
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "chassis_4ws4wd");
    // 创建节点句柄
    ros::NodeHandle nh_;
	ros::Rate loop_rate(1000);  //Hz
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
	
//	led_control_para.channel = 1 ;
//	led_control_para.mode =!led_control_para.mode;
////	led_control_para.mode = 0x00;
//	led_control_para.on_lightness=1;
//	encode_led_cmd(led_control_para);
	
    while(ros::ok())
	{
	 	run(motion_cmd_para);
		ros::spinOnce();                 
		loop_rate.sleep();
	}
	chassis_close_udp();
	
    return 0;
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
}
