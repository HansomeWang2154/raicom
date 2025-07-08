# 硬件串口需要配置下，目前使用usb转串口代替下，由于驱动的问题，需要

# 确认驱动已加载（提前与要装下ch340/341的驱动）
lsmod | grep ch341

# 强制绑定设备（需要root权限）
echo "1a86 e523" | sudo tee /sys/bus/usb-serial/drivers/ch341-uart/new_id


sudo chmod +x /dev/ttyUSB0
# 检查是否生成设备节点
ls /dev/ttyUSB*


# 启动串口发送节点
roslaunch commu serial_send.launch 


# 发布速度话题
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"


目前需要完善目标速度结算部分，并且大概需要几个节点：串口发送；速度发布（从键盘或者SLAM节点给）
ok
基本功能完成，看看角速度闭环和上危机读取odem的信息的部分，尝试SLAM
yu将军的模型部署部分，抽空给训练
<!-- 构建tf树时，不加延时好像 有问题 -->
发送端发送的太快，接收端读取到的是历史数据-》及时清空接受buffer
目前有的电机没上电，odom依赖于编码器信息，所以不work

通讯部分应该是ok了，接下来调参数（pid，比例尺）

SLAM先跑跑仿真学习学习


串口1有些地方注释了，出问题了再改回来
















































解决了
