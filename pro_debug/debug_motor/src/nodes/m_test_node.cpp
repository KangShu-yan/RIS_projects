#include <motor_test/motor_test.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "motor_test_node");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  ROS_INFO("MAIN");
  debug_motor::DebugMotor node(nh);	
  node.start();
  // Let ROS handle all callbacks.
  //ros::spin();

  return 0;
}  // end main()
