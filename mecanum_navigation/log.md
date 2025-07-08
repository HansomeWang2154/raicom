ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB1
roslaunch commu serial_send.launch port:=/dev/ttyUSB1
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"  #启动雷达
rosrun ldlidar_ros fake_odom.py #伪里程计（测试用）
rosrun gmapping slam_gmapping scan：=scan #gmapping建图
rosrun map_server map_saver -f /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/maps/my_map  #保存地图



rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_link 100
rosrun tf tf_echo base_link base_footprint
roslaunch mecanum_navigation navigation.launch