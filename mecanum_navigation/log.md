ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB1
roslaunch commu serial_send.launch port:=/dev/ttyUSB1





# 实时设置更高电流值
echo 500 | sudo tee /sys/bus/usb/devices/usb3/3-2/bMaxPower >/dev/null

# 立即生效（不需重启）
sudo udevadm control --reload
sudo udevadm trigger
临时电流提升

echo 500 | sudo tee /sys/bus/usb/devices/7-2/power/max_power
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"  #启动雷达   注意不同的usb口的最大电流不同，有的可以在配置文件中修改，100ma不够，会导致连接异常


rosrun ldlidar_ros fake_odom.py #伪里程计（测试用）
rosrun gmapping slam_gmapping scan：=scan #gmapping建图


rosrun gmapping slam_gmapping scan:=scan \
  _linearUpdate:=0.05 \       # 原默认0.5m，降低触发阈值
  _angularUpdate:=0.05 \     # 原默认0.5rad，改为5°
  _temporalUpdate:=1.0 \     # 原默认-1.0(禁用)，改为1秒强制更新
  _throttle_scans:=1 \       # 原默认1，处理每帧扫描
  _minimumScore:=1000        # 原默认0，提高闭环检测敏感度

rosrun gmapping slam_gmapping scan:=scan \
  # === 里程计信任增强 ===
  _odom_alpha1:=0.01 \
  _odom_alpha2:=0.01 \
  _odom_alpha3:=0.01 \
  _odom_alpha4:=0.01 \
  
  # === 雷达作用降权 ===
  _linearUpdate:=5.0 \
  _angularUpdate:=1.57 \
  _temporalUpdate:=10.0 \
  _sigma:=0.1 \
  
  # === 扫描处理弱化 ===
  _iterations:=1 \
  _minimumScore:=3000 \
  _llsamplerange:=0.1 \     # 扩大搜索范围
  
  # === 通用设置 ===
  _particles:=25 \          # 平衡值
  _throttle_scans:=3 \
  
  # === 环境参数优化 ===
  _xmin:=-10.0 \
  _xmax:=10.0 \
  _ymin:=-10.0 \
  _ymax:=10.0 \
  _maxRange:=8.0 \
  _maxUrange:=6.0 \
  _transform_timeout:=0.1













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
