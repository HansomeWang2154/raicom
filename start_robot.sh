#!/bin/bash

# 检查lxterminal
if ! command -v lxterminal &> /dev/null; then
    echo "错误: 请先安装lxterminal: sudo apt install lxterminal"
    exit 1
fi

## 关闭残留的ROS节点（保留roscore，只清理其他节点）
#killall -9 roslaunch rosnode &> /dev/null
#
## 清除ROS缓存
#rm -rf ~/.ros/*

# 等待已有roscore启动就绪（根据实际情况调整，确保roscore已运行）
echo "等待已有roscore启动就绪..."
sleep 5  # 若roscore启动较慢，可延长此时间（如8-10秒）

# 终端1: 串口通讯（自动检测USB）
lxterminal -t "Serial Communication" -e "bash -c 'if [ -e /dev/ttyUSB1 ]; then roslaunch commu serial_send.launch port:=/dev/ttyUSB1; else roslaunch commu serial_send.launch port:=/dev/ttyUSB2; fi; exec bash'" &

# 终端2: 雷达读取 + RViz（加载配置文件）
lxterminal -t "Lidar & RViz" -e "bash -c 'roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:=/dev/ttyUSB0 && rviz -d /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/rviz/navigation.rviz; exec bash'" &

# 终端3: 开启move_base导航
lxterminal -t "Move Base" -e "bash -c 'roslaunch mecanum_navigation navigation.launch; exec bash'" &

# 终端4: 底盘运动流程
lxterminal -t "Base Control" -e "bash -c 'rosrun mecanum_navigation joint_run2_std.py; exec bash'" &

echo "所有组件启动中..."
