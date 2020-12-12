#include <plc_test/plc_test.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "plc_test_node");
  ros::NodeHandle nh;

  // Create a new object.
  ROS_INFO("MAIN");
  test_plc::TestPlc node(nh);	
  node.start();
  // Let ROS handle all callbacks.
  //ros::spin();

  return 0;
}  // end main()
