ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB1
roslaunch commu serial_send.launch port:=/dev/ttyUSB0
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"  #启动雷达
rosrun ldlidar_ros fake_odom.py #伪里程计（测试用）
rosrun gmapping slam_gmapping scan：=scan #gmapping建图


rosrun gmapping slam_gmapping scan:=scan \
  _linearUpdate:=0.05 \       # 原默认0.5m，降低触发阈值
  _angularUpdate:=0.05 \     # 原默认0.5rad，改为5°
  _temporalUpdate:=1.0 \     # 原默认-1.0(禁用)，改为1秒强制更新
  _throttle_scans:=1 \       # 原默认1，处理每帧扫描
  _minimumScore:=1000        # 原默认0，提高闭环检测敏感度

rosrun map_server map_saver -f /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/maps/my_map  #保存地图



rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_link 100
rosrun tf tf_echo base_link base_footprint
roslaunch mecanum_navigation navigation.launch



rosparam load amcl_params.yaml /amcl
rostopic echo /amcl_pose
# 使用rostopic发布目标
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
