#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 

using namespace std;

ros::Publisher vel_pub_;
ros::Subscriber joy_sub_; 
  
double l_scale_=0.5, a_scale_=0.5; 
double l_scale_1=0.0, w_scale_1=0.0; 
double l_scale_2=0.0, w_scale_2=0.0;
double f_scale_ = 0.5,f_scale_1=20.,f_scale_2=50.;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{ 
	geometry_msgs::Twist twist; 
//	std::cout<<"transfor_joy"<<std::endl; 
	ROS_INFO("transfor_joy"); 
	if(joy->buttons[2])		//X键
	{
		
		if(joy->buttons[0])	//A键
		{
//			l_scale_=0.25*32767.;
			l_scale_=l_scale_1;
			a_scale_=w_scale_1;
			
		}
		else if(joy->buttons[1])	//B键
		{
			l_scale_=l_scale_2;
			a_scale_=w_scale_2;
			
		}
		else;
		twist.angular.z = a_scale_*joy->axes[0]; //左摇杆左右
		twist.linear.x = l_scale_*joy->axes[1]; //左摇杆前后
		twist.linear.y=2;
	}
	else if(joy->buttons[3])
	{
		
        if(joy->buttons[0]==1)
		{	
	   		f_scale_=f_scale_1;
//			twist.linear.x = 20; 
			
		}
		else if(joy->buttons[1])
		{
			f_scale_=f_scale_2;
//			twist.linear.x = 0; 
			
		}
		twist.angular.z = f_scale_*joy->axes[0];; 
		twist.linear.y=3;
	}
	else
	{
		twist.linear.x = 0.;
		twist.linear.y = 0.;
		twist.angular.z = 0.;
	} 
    if(joy->buttons[5]) //right
	{
     	twist.linear.y=1;
	}
	if(joy->buttons[4]) //right
	{
     	twist.linear.y=4;
	}
	vel_pub_.publish(twist);
	std::cout<<"I recived the data: "<<twist.linear.x <<"  "<<twist.angular.z <<std::endl;

}
int main(int argc, char** argv) 
{ 
 	
	ros::init(argc, argv, "transfor_joy");
 	ros::NodeHandle nh_; 
 	ros::param::get("/transfor_joy/l_scale_1", l_scale_1);
 	ros::param::get("/transfor_joy/l_scale_2", l_scale_2);
 	ros::param::get("/transfor_joy/w_scale_1", w_scale_1);
 	ros::param::get("/transfor_joy/w_scale_2", w_scale_2);
 	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback); 
	ros::spin();
}

