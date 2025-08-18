#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32

class SimpleNavigator:
    def __init__(self):
        rospy.init_node('simple_navigator', anonymous=True)
        
        # 目标点列表 (x, y)
        self.goals = [
            (0.25, 0.0),    # 目标点1
            (0.25, 1.20),   # 目标点2
            (1.8, 1.20),    # 目标点3
            (3.0, 1.20),    # 目标点4
            (4.2, 1.20),    # 目标点5
            (6.5, 1.20),    # 目标点6
            (6.5, 3.65),    # 目标点7
            (0.3, 3.65),    # 目标点8
            (0.00, -1.4),   # 目标点9 (相对坐标)
            (0.05, -1.4),   # 目标点10 (相对坐标)
            (1.6, -1.4),    # 目标点11 (相对坐标)
            (2.8, -1.4),    # 目标点12 (相对坐标)
            (4.0, -1.4),    # 目标点13 (相对坐标)
            (4.0, 0.0),     # 目标点14 (相对坐标)
            (0.0, 0.0)      # 目标点15 (相对坐标)
        ]
        
        # 运动参数
        self.speed = 0.3  # 线速度 (m/s)
        self.angular_speed = 0.1  # 角速度 (rad/s)
        self.tolerance = 0.05  # 位置容忍度 (m)
        self.angle_tolerance = 0.1  # 角度容忍度 (rad)
        
        # 当前状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_goal_index = 0
        self.reference_x = 0.0  # 参考零点X坐标
        self.reference_y = 0.0  # 参考零点Y坐标
        self.use_relative = False  # 是否使用相对坐标
        self.start_received = False  # 是否收到启动信号
        self.yolo_received = False  # 是否收到YOLO信号
        
        # 发布者和订阅者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arrive_pub = rospy.Publisher('/wheels/arrive', Int32, queue_size=10)
        self.arrive_box_pub = rospy.Publisher('/wheels/arrive_box', Int32, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.start_sub = rospy.Subscriber('/wheels/start', Int32, self.start_callback)
        self.yolo_sub = rospy.Subscriber('/yolo/picked', Int32, self.yolo_callback)
        
        rospy.loginfo("简易导航节点已启动，等待/wheels/start信号...")

    def odom_callback(self, msg):
        """处理里程计信息"""
        # 获取位置信息
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 获取姿态信息 (转换为欧拉角)
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = euler_from_quaternion(quaternion)

    def start_callback(self, msg):
        """处理启动信号"""
        if msg.data == 1 and not self.start_received:
            self.start_received = True
            rospy.loginfo("收到/wheels/start信号，开始导航任务")

    def yolo_callback(self, msg):
        """处理YOLO检测信号"""
        if msg.data == 1:
            self.yolo_received = True
            rospy.loginfo("收到/yolo/picked信号")

    def publish_velocity(self, x, y, duration):
        """发布指定速度并保持指定时间"""
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = 0.0
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo(f"发布速度指令: x={x}, y={y}，持续{duration}秒")
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止运动
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("速度指令结束")

    def reach_goal(self, goal_x, goal_y):
        """移动到目标点"""
        twist = Twist()
        
        # 如果使用相对坐标，将目标点转换为绝对坐标
        target_x = goal_x + (self.reference_x if self.use_relative else 0)
        target_y = goal_y + (self.reference_y if self.use_relative else 0)
        
        # 判断是x方向还是y方向移动
        if abs(target_x - self.current_x) > abs(target_y - self.current_y):
            # x方向移动
            move_in_x = True
            if target_x > self.current_x:
                twist.linear.x = self.speed
            else:
                twist.linear.x = -self.speed
            twist.linear.y = 0.0
        else:
            # y方向移动
            move_in_x = False
            if target_y > self.current_y:
                twist.linear.y = self.speed
            else:
                twist.linear.y = -self.speed
            twist.linear.x = 0.0
        
        # 移动直到到达目标点附近
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 根据运动方向计算相应轴的距离
            if move_in_x:
                # x方向运动，只计算x轴距离
                distance = abs(target_x - self.current_x)
            else:
                # y方向运动，只计算y轴距离
                distance = abs(target_y - self.current_y)
            
            if distance < self.tolerance:
                # 到达目标点，停止移动
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo(f"到达目标点 ({goal_x}, {goal_y}) {'(相对坐标)' if self.use_relative else '(绝对坐标)'}")
                return True
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        return False

    def adjust_angle(self):
        """调整角度至接近0rad"""
        rospy.loginfo("开始调整角度...")
        twist = Twist()
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 计算与0rad的偏差
            angle_error = self.current_yaw
            
            # 确保角度误差在[-π, π]范围内
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_error) < self.angle_tolerance:
                # 角度已调整到位
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo(f"角度调整完成，当前角度: {self.current_yaw:.3f} rad")
                return True
            
            # 设置角速度 (根据误差方向)
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        return False

    def wait_for_yolo(self):
        """等待YOLO信号"""
        rospy.loginfo("等待/yolo/picked信号...")
        self.yolo_received = False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.yolo_received:
            rate.sleep()

    def run(self):
        """运行导航程序"""
        rate = rospy.Rate(10)  # 10Hz
        
        # 等待里程计信息
        while not rospy.is_shutdown() and (self.current_x == 0 and self.current_y == 0):
            rospy.loginfo("等待里程计信息...")
            rate.sleep()
        
        # 等待启动信号
        while not rospy.is_shutdown() and not self.start_received:
            rate.sleep()
        
        # 依次访问每个目标点
        for i, (goal_x, goal_y) in enumerate(self.goals):
            self.current_goal_index = i
            rospy.loginfo(f"前往目标点 {i+1}/{len(self.goals)}: ({goal_x}, {goal_y}) {'(相对坐标)' if self.use_relative else '(绝对坐标)'}")
            
            # 移动到目标点
            if not self.reach_goal(goal_x, goal_y):
                rospy.logerr("无法到达目标点，程序终止")
                return
            
            # 检查角度是否需要调整
            if abs(self.current_yaw) > 0.15:
                self.adjust_angle()
            
            # 目标点8的特殊操作
            if i == 7:  # 因为索引从0开始，i=7对应目标点8
                rospy.loginfo("执行目标点8的特殊操作")
                # 发布速度（x=0，y=0.1）2秒
                self.publish_velocity(0.0, 0.1, 4.0)
                # 发布速度（x=0.1，y=0.1）2秒
                self.publish_velocity(-0.1, 0.0, 4.0)
                rospy.loginfo("发布/wheels/arrive_box信号 (目标点8)")
                self.arrive_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

                # 记录当前/odom数据作为参考零点
                self.reference_x = self.current_x
                self.reference_y = self.current_y
                rospy.loginfo(f"设置参考零点: ({self.reference_x:.2f}, {self.reference_y:.2f})")
                
                # 启用相对坐标模式
                self.use_relative = True
            
            # 目标点2，3，4，5，10，11，12，13到达后发布'/wheels/arrive'
            if i in [1, 2, 3, 4, 9, 10, 11, 12]:
                rospy.loginfo(f"发布/wheels/arrive信号 (目标点{i+1})")
                self.arrive_pub.publish(Int32(1))
            
            # 目标点15到达后发布'/wheels/arrive_box'
            if i == 14:  # 目标点15
                rospy.loginfo("发布/wheels/arrive_box信号 (目标点15)")
                self.arrive_box_pub.publish(Int32(1))
            
            # 目标点3，4，5，6，11，12，13，14需要等待/yolo/picked后再继续
            if i in [1,2, 3, 4, 10, 11, 12, 13]:
                self.wait_for_yolo()
            
            # 不需要固定sleep，直接准备下一个目标点
        
        rospy.loginfo("所有目标点已到达，导航完成")

if __name__ == '__main__':
    try:
        navigator = SimpleNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航程序被中断")
    