#include <ws_motor/ws_motor.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ws_motor");
  ros::NodeHandle nh;
  
  ROS_INFO("MAIN");
  ws_motor::WSMotor node(nh);	
  node.init_motor();
  
  return 0;
}  // end main()
