#include <ws_suspension/ws_suspension.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ws_motor");
  ros::NodeHandle nh;
  
  ROS_INFO("MAIN");
  ws_suspension::WSSuspension node(nh);	
  node.init_suspension();
  
  return 0;
}  // end main()
