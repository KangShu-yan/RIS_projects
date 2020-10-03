cmake_minimum_required(VERSION 3.0.2)
project(debug_motor)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs serial)

add_message_files(
  FILES
  debugMotorData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/debugMotor.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs
)

add_definitions(-std=c++11)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(d_motor src/nodes/d_motor_node.cpp src/d_motor.cpp)
target_link_libraries(d_motor ${catkin_LIBRARIES})
add_dependencies(d_motor debug_motor_gencfg debug_motor_generate_messages_cpp)

#add_executable(r_motor src/r_motor.cpp src/nodes/r_motor_node.cpp)
#target_link_libraries(run_motor ${catkin_LIBRARIES})
#add_dependencies(r_motor debug_motor_gencfg debug_motor_generate_messages_cpp)


