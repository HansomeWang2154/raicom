# 查看端口号
```
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB1
```
# 启动上下位机串口通讯节点（附带键盘遥控）  终端1
```
roslaunch commu serial_send.launch port:=/dev/ttyUSB1  #如果USB1不存在则启动USB2
```
# 雷达读取 终端2
```
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"  #启动雷达   注意不同的usb口的最大电流不同，有的可以在配置文件中修改，100ma不够，会导致连接异常
```
# Gmapping建图
```
rosrun gmapping slam_gmapping scan：=scan #gmapping建图
```


# 保存地图
```
rosrun map_server map_saver -f /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/maps/my_map1  #保存地图
```

# 开启move_base导航 终端3
```
roslaunch mecanum_navigation navigation.launch    #movebase需要一条tf树，可以先运行gmapping再终止快捷建立
```
# 在rviz中发布nav_goal或者写ros节点发布
```
rosrun mecanum_navigation multi_goal_publisher.py
```
# 底盘运动流程 #终端4
rosrun mecanum_navigation joint_run2.py

rosparam set /move_base/DWAPlannerROS/min_vel_theta -1.0

rostopic pub /yolo/picked std_msgs/Int32 "data: 1" 

# 使用rostopic发布目标/取消目标
```
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
    z: 0.0  # 目标朝向(z,w需构成单位四元数)
    w: 1.0
"
```


```
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "
stamp:
  secs: 0
  nsecs: 0
id: ''
"
```



  pose: 
    position: 
      x: 1.0922034556661477
      y: 0.8920791876581553
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.056896829935069705
      w: 0.9983800632741721


再改改amcl lauch文件的配置，实在不行就禁止amcl发布odom-map变换，只用里程计数据


现在定位基本没问题，比较稳定，问题主要出在过弯会挂蹭，晚上写下全场的逻辑，几个摘取点，几个路径点