cmake_minimum_required(VERSION 3.0.2)
project(plc_test)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs serial)

add_message_files(
  FILES
  testPlcData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/plcTest.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs serial
)

add_definitions(-std=c++11)


include_directories(include ${catkin_INCLUDE_DIRS}
)



add_executable(plc_test  src/plc_test.cpp src/nodes/plc_test_node.cpp src/wrap_tcp.cpp)
target_link_libraries(plc_test ${catkin_LIBRARIES})
add_dependencies(plc_test plc_test_gencfg plc_test_generate_messages_cpp)



