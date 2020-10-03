
/**
 * 
 */
#include "chassis_4ws4wd/chassis_4ws4wd.h"  //tcp_client is the directory contained in include directory 
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
	
    while(ros::ok())
	{
	 	run();
		ros::spinOnce();                 
		loop_rate.sleep();
	}
	chassis_close_udp();
    return 0;
}



