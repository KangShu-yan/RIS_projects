#! /bin/bash


### BEGIN INIT INFO
# Provides:          bbzhh.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: tomcat service
# Description:       tomcat service daemon
### END INIT INFO

#source /opt/ros/indigo/setup.sh
#source /home/gosunyun/catkin_ws/devel/setup.bash

#roslaunch /home/gosunyun/catkin_ws/src/auto_start/launch/auto_start.launch


#根据系统启动相应版本的roscore

gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/ws2500A/catkin_ws/devel/setup.bash;roslaunch sixwheel_chassis_control TCP2CAN.launch" #启动节点
gnome-terminal -x bash -c "rosbag record -a" #启动节点

# sleep 8 #等待roscore启动(可以不写)c


# gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosrun turtlesim turtlesim_node" &  #启动roscore

wait
exit 0

