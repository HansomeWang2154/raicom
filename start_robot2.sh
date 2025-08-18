

#!/bin/bash

# 启动第一个命令：roslaunch commu serial_send.launch port:=/dev/ttyUSB1
# 如果USB1不存在，则尝试启动USB2
roslaunch commu serial_send.launch port:=/dev/ttyUSB1 &

# 等待2秒钟，确保第一个命令已经启动
sleep 2

# 启动第二个命令：rosrun mecanum_navigation joint_run2_con.py
rosrun mecanum_navigation joint_run2_con.py &
 
echo "两个命令已经启动."


# #终端1：
# roslaunch commu serial_send.launch port:=/dev/ttyUSB1  #如果USB1不存在则启动USB2
# #终端2：
# rosrun mecanum_navigation joint_run2_con.py 
