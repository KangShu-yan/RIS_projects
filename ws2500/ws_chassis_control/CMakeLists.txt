cmake_minimum_required(VERSION 3.0.2)
project(ws_motor)

find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure 
	message_generation 
	roscpp 
	rosgraph_msgs
	rospy
	std_msgs
	serial 
	tf 
	geometry_msgs
	sensor_msgs
	cmake_modules
)
find_package(Eigen REQUIRED )


add_message_files(
  FILES
  wsMotorData.msg
  wsSuspensionData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/wsMotor.cfg
#)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs serial tf geometry_msgs
)

add_definitions(-std=c++11)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ws_motor_node src/nodes/ws_motor_node.cpp src/ws_motor.cpp src/wrap_tcp.cpp)
target_link_libraries(ws_motor_node ${catkin_LIBRARIES})
#add_dependencies(ws_motor_node ws_motor_gencfg ws_motor_generate_messages_cpp)
add_dependencies(ws_motor_node ws_motor_generate_messages_cpp)


add_executable(ws_suspension_node src/nodes/ws_suspension_node.cpp src/ws_suspension.cpp src/wrap_tcp.cpp)
target_link_libraries(ws_suspension_node ${catkin_LIBRARIES})
#add_dependencies(ws_suspension_node ws_motor_gencfg ws_motor_generate_messages_cpp)
add_dependencies(ws_suspension_node ws_motor_generate_messages_cpp)



