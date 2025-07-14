# 查看端口号
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB1
# 启动上下位机串口通讯节点（附带键盘遥控）
roslaunch commu serial_send.launch port:=/dev/ttyUSB1
# 雷达读取
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"  #启动雷达   注意不同的usb口的最大电流不同，有的可以在配置文件中修改，100ma不够，会导致连接异常
# Gmapping建图
rosrun gmapping slam_gmapping scan：=scan #gmapping建图


# 保存地图
rosrun map_server map_saver -f /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/maps/my_map  #保存地图

# 开启move_base导航
roslaunch mecanum_navigation navigation.launch
movebase需要一条tf树，可以先运行gmapping再终止快捷建立
# 在rviz中发布nav_goal或者写ros节点发布
rosrun mecanum_navigation multi_goal_publisher.py



# 使用rostopic发布目标/取消目标
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "
header:
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 0.0  # 目标x坐标
    y: 0.0  # 目标y坐标
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0  # 目标朝向(z,w需构成单位四元数)
    w: 0.0
"



rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "
stamp:
  secs: 0
  nsecs: 0
id: ''
"


#命名空间有问题，都多了层move_bae
