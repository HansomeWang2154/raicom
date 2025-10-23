# 🤖 RaiCom机器人比赛项目

## 📋 项目概述

本项目是为RaiCom机器人比赛专门开发的移动机械臂机器人系统，比赛背景为果实采摘场景。系统集成了麦克纳姆轮底盘、六自由度机械臂、激光雷达、视觉系统等多种传感器和执行器，具备自主导航、环境感知、机械臂操作等功能，可在模拟果实采摘比赛环境中完成目标识别、定位、采摘等指定任务。

<!-- 此处预留视频插入位置 -->
示例：<video src="demo.mp4" width="800" height="600" controls></video>

## 🏗️ 系统架构

![系统架构](frames.pdf)

系统采用ROS分布式架构，主要由以下几个核心模块组成：

1. **底盘控制系统**：基于麦克纳姆轮的全向移动底盘
2. **机械臂控制系统**：Piper六自由度机械臂及MoveIt!控制
3. **感知系统**：激光雷达、深度相机、IMU传感器
4. **导航与定位**：基于AMCL的定位和MoveBase的导航规划
5. **通信系统**：串口通信模块，连接上层ROS与下层STM32控制器

### 📁 项目文件结构

```
raicom/
├── CMakeLists.txt                # 主工作空间CMake配置文件
├── commu/                        # 串口通信模块
├── easy_handeye/                 # 手眼机械臂相关代码
├── fdilink_ahrs/                 # IMU传感器驱动
├── ldlidar_ros/                  # 激光雷达驱动
├── mecanum_navigation/           # 底盘导航功能
├── piper/                        # 机械臂控制
├── piper_description/            # 机械臂描述文件
├── piper_moveit/                 # MoveIt!配置
├── piper_msgs/                   # 机械臂消息定义
├── piper_sim/                    # 仿真环境
├── ros_astra_camera/             # 深度相机驱动
├── frames.gv                      
├── frames.pdf                     
├── readme.md                     # 项目说明文档
├── start_robot.sh                # 一键启动脚本
├── start_robot1.sh               # 备选启动脚本1
└── start_robot2.sh               # 备选启动脚本2
```

## 🎯 功能模块

### 🚗 1. 底盘控制与导航

- **全向移动**：基于麦克纳姆轮的全向移动能力，适用于比赛场地的灵活操作
- **自主导航**：使用MoveBase进行路径规划和避障，快速到达果实采摘点
- **AMCL定位**：自适应蒙特卡洛定位算法，在比赛环境中实现高精度定位
- **地图构建**：支持Gmapping进行SLAM地图构建，适应不同比赛场地
- **多目标导航**：支持多个导航目标点的连续执行，提高比赛效率

### 🔧 2. 机械臂控制

- **六自由度操作**：Piper机械臂，具备6个关节自由度，可完成复杂果实抓取任务
- **MoveIt!集成**：支持直观的可视化编程和路径规划，快速调整机械臂轨迹
- **多种控制方式**：支持关节角度控制、末端位姿控制，适应不同比赛需求
- **夹爪操作**：支持末端执行器的开合控制，可靠抓取比赛中的模拟果实

### 👁️ 3. 感知系统

- **激光雷达**：使用LD06激光雷达进行环境扫描，精确感知周围障碍物
- **深度相机**：Astra深度相机，提供视觉信息，用于果实目标识别和定位
- **IMU传感器**：提供机器人姿态和运动信息，增强导航稳定性

### 📡 4. 通信与接口

- **串口通信**：与STM32等底层控制器通信，实时控制机器人硬件
- **服务接口**：提供ROS服务供外部调用，便于开发果实采摘比赛策略
- **话题发布**：实时发布传感器数据和机器人状态，支持监控和调试
- **键盘遥测**：支持手动控制机器人运动，用于比赛前测试和调整

## 🏃‍♂️ 快速开始

### 🔧 环境要求

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- Python 2.7/3.8
- lxterminal（用于启动多终端）

### 📦 安装依赖

```bash
# 安装必要的ROS包
sudo apt install ros-noetic-move-base ros-noetic-amcl ros-noetic-gmapping ros-noetic-map-server

# 安装lxterminal
sudo apt install lxterminal

# 安装其他依赖（根据需要）
sudo apt install ros-noetic-rviz ros-noetic-robot-state-publisher
```

### 🚀 启动机器人

系统提供了多个启动脚本，方便在不同场景下使用：

```bash
# 进入工作空间
cd /path/to/raicom

# 使用主启动脚本（确保有执行权限）
chmod +x start_robot.sh
./start_robot.sh

# 或使用备选启动脚本（根据实际需求选择）
chmod +x start_robot1.sh start_robot2.sh
./start_robot1.sh  # 备选启动配置1
./start_robot2.sh  # 备选启动配置2
```

### 🖥️ 手动启动各组件

#### 1. 启动上下位机串口通讯节点（终端1）
```bash
# 查看可用串口
echo "查看端口号"
ls /dev/ttyUSB*

# 给予串口权限
sudo chmod 666 /dev/ttyUSB1

# 启动串口通信（附带键盘遥控）
roslaunch commu serial_send.launch port:=/dev/ttyUSB1  #如果USB1不存在则启动USB2
```

#### 2. 启动雷达（终端2）
```bash
roslaunch ldlidar_ros viewer_ld06_noetic.launch port_name:="/dev/ttyUSB0"
# 注意：不同的USB口的最大电流不同，有的可能需要在配置文件中修改，100mA可能不够，会导致连接异常
```

#### 3. 开启MoveBase导航（终端3）
```bash
roslaunch mecanum_navigation navigation.launch
# movebase需要一条tf树，可以先运行gmapping再终止快捷建立
```

#### 4. 启动底盘运动流程（终端4）
```bash
rosrun mecanum_navigation joint_run2.py
```

## 🗺️ 地图构建

### 🔨 Gmapping建图
```bash
rosrun gmapping slam_gmapping scan:=scan
```

### 💾 保存地图
```bash
rosrun map_server map_saver -f /path/to/your/map_name
# 例如：
rosrun map_server map_saver -f /home/orangepi/eyes_piper/piper_ros/src/mecanum_navigation/maps/my_map
```

## 🧭 导航控制

### 🖱️ 在RViz中设置目标点
可以通过RViz的"2D Nav Goal"工具直接在地图上点击目标位置。

### ⌨️ 使用命令行发布目标点
```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "
header:
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.0  # 目标x坐标
    y: 0.5  # 目标y坐标
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0  # 目标朝向(z,w需构成单位四元数)
    w: 1.0
"
```

### ❌ 取消导航目标
```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "
stamp:
  secs: 0
  nsecs: 0
id: ''
"
```

### 🎯 多目标发布
```bash
rosrun mecanum_navigation multi_goal_publisher.py
```

## 🦾 机械臂控制

### 🖼️ MoveIt!可视化控制
1. 启动MoveIt!界面
2. 在MotionPlanning面板中拖动机械臂到目标位置
3. 点击"Plan & Execute"开始规划并执行运动

### 🔧 服务端控制（命令行）

#### 机械臂关节角度控制
```bash
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 
```

#### 机械臂末端位置控制
```bash
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 
```

#### 夹爪控制
```bash
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

#### 机械臂和夹爪联合控制
```bash
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

### 📱 客户端控制
```bash
rosrun moveit_ctrl joint_moveit_ctrl.py
```

## ⚙️ 配置与调优

### 🛠️ 导航参数调优
可以通过修改以下参数来优化导航性能，适应果实采摘比赛环境：

```bash
# 增加角速度范围，改善转弯性能
rosparam set /move_base/DWAPlannerROS/min_vel_theta -1.0
```

### 🔍 AMCL参数调优
修改`mecanum_navigation/launch/amcl.launch`文件中的参数，如：
- 粒子数量（`min_particles`, `max_particles`）：根据比赛场地复杂度调整
- 运动模型参数（`odom_alpha1`~`odom_alpha5`）：提高定位精度
- 初始位姿（`initial_pose_x`, `initial_pose_y`, `initial_pose_a`）：设置比赛初始位置

## ❓ 常见问题与排查

### 🔌 串口连接问题
- 检查串口权限：使用`sudo chmod 666 /dev/ttyUSB*`
- 确认正确的串口号：通过`ls /dev/ttyUSB*`查看
- 检查USB供电：某些设备需要足够的电流供应

### 🧭 导航定位问题
- 确保地图与比赛环境匹配
- 调整AMCL参数以适应不同比赛场地
- 检查TF转换关系是否正确

### 🔧 机械臂控制问题
- 确认关节角度范围是否在有效范围内
- 检查机械臂是否有足够的空间进行运动
- 调整运动速度和加速度参数，确保在比赛中稳定可靠

## 👨‍💻 开发指南

### 🔨 创建新的功能包
```bash
cd /path/to/raicom/src
catkin_create_pkg my_new_package std_msgs rospy roscpp
```

### 🏗️ 编译工作空间
```bash
cd /path/to/raicom
catkin_make
```

### 🧪 运行测试
```bash
# 运行特定功能包的测试
rosrun my_new_package test_node.py
```

## 📜 许可证

本项目采用MIT许可证。详见LICENSE文件。

## 👥 作者与贡献

如有任何问题或建议，请联系项目维护者。欢迎提交Issue和Pull Request。

---

*文档最后更新时间：$(date +%Y-%m-%d)*