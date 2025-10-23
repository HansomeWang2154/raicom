#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32
import time

class SimpleNavigator:
    def __init__(self):
        rospy.init_node('simple_navigator', anonymous=True)
        
        # 目标点列表 (x, y)
        self.goals = [
            (0.5, 0.0),     # 目标点1
            (0.5, 1.2),     # 目标点2
            (1.95, 1.2),     # 目标点3
            (3.55, 1.2),     # 目标点4
            (5.05, 1.2),     # 目标点5
            (6.5, 1.2),     # 目标点6
            (6.5, 4.0),    # 目标点7
            (0.3, 4.0),    # 目标点8
            (0.00, -1.4),   # 目标点9 (相对坐标)
            (0.30, -1.4),   # 目标点10 (相对坐标)
            (1.75, -1.4),    # 目标点11 (相对坐标)
            (3.35, -1.4),    # 目标点12 (相对坐标)
            (4.85, -1.4),    # 目标点13 (相对坐标)
            (4.85, 0.0),     # 目标点14 (相对坐标)
            (0.0, 0.0)      # 目标点15 (相对坐标)
        ]
        
        # 运动参数
        self.speed = 0.4  # 线速度 (m/s)
        self.angular_speed = 0.10  # 角速度 (rad/s)
        self.tolerance = 0.05  # 位置容忍度 (m)
        self.angle_tolerance = 0.03  # 角度容忍度 (rad)
        self.timeout_duration = 240  # 超时时间（根据日志显示为30秒）
        
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
        self.start_time = None  # 记录收到start信号的时间
        self.goal6_published = False  # 标记目标点6是否已发布
        self.is_jumping = False  # 标记是否正在执行超时跳转
        
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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 转换姿态为欧拉角
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = euler_from_quaternion(quaternion)

    def start_callback(self, msg):
        """处理启动信号"""
        if msg.data == 1 and not self.start_received:
            self.start_received = True
            self.start_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"收到/wheels/start信号，开始导航任务，启动时间: {self.start_time:.2f}")

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
        rate = rospy.Rate(10)
        
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
        """移动到目标点，添加跳转中状态判断"""
        twist = Twist()
        
        # 转换为绝对坐标
        target_x = goal_x + (self.reference_x if self.use_relative else 0)
        target_y = goal_y + (self.reference_y if self.use_relative else 0)
        
        # 判断运动方向
        if abs(target_x - self.current_x) > abs(target_y - self.current_y):
            move_in_x = True
            twist.linear.x = self.speed if target_x > self.current_x else -self.speed
            twist.linear.y = 0.0
        else:
            move_in_x = False
            twist.linear.y = self.speed if target_y > self.current_y else -self.speed
            twist.linear.x = 0.0
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 超时检测：跳转过程中暂时禁用超时检查
            if not self.is_jumping and not self.goal6_published and self.start_time:
                current_time = rospy.Time.now().to_sec()
                elapsed_time = current_time - self.start_time
                if elapsed_time > self.timeout_duration:
                    rospy.logwarn(f"超时警告: 已超过{self.timeout_duration}秒未发布目标点6，将直接跳转至目标点6")
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    self.cmd_vel_pub.publish(twist)
                    return False  # 触发跳转
            
            # 计算距离
            if move_in_x:
                distance = abs(target_x - self.current_x)
            else:
                distance = abs(target_y - self.current_y)
            
            if distance < self.tolerance:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo(f"到达目标点 ({goal_x}, {goal_y}) {'(相对坐标)' if self.use_relative else '(绝对坐标)'}")
                return True
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        return False



    def adjust_angle(self):
        """调整角度至接近0rad，带3秒超时限制"""
        rospy.loginfo("开始调整角度...")
        twist = Twist()
        rate = rospy.Rate(10)
        
        start_time = time.time()  # 记录开始时间
        
        while not rospy.is_shutdown():
            # 计算当前角度误差
            angle_error = self.current_yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # 归一化到[-π, π]
            
            # 检查是否超时
            if time.time() - start_time > 3.0:  # 超过3秒
                rospy.logwarn("调整角度超时，退出调整")
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return False
            
            # 如果角度误差足够小，则认为调整完成
            if abs(angle_error) < self.angle_tolerance:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo(f"角度调整完成，当前角度: {self.current_yaw:.3f} rad")
                return True
            
            # 根据角度误差调整旋转方向
            twist.angular.z = -self.angular_speed if angle_error > 0 else self.angular_speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        return False

    def wait_for_yolo(self):
        """等待YOLO信号，添加超时检测"""
        rospy.loginfo("等待/yolo/picked信号...")
        self.yolo_received = False
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown() and not self.yolo_received:
            if not self.is_jumping and not self.goal6_published and self.start_time:
                current_time = rospy.Time.now().to_sec()
                elapsed_time = current_time - self.start_time
                if elapsed_time > self.timeout_duration:
                    rospy.logwarn(f"超时警告: 已超过{self.timeout_duration}秒未发布目标点6，将直接跳转至目标点6")
                    return False  # 触发跳转
            rate.sleep()
        return True

    def run(self):
        """运行导航程序，修复跳转逻辑"""
        rate = rospy.Rate(10)
        
        # 等待里程计信息
        while not rospy.is_shutdown() and (self.current_x == 0 and self.current_y == 0):
            rospy.loginfo("等待里程计信息...")
            rate.sleep()
        
        # 等待启动信号
        while not rospy.is_shutdown() and not self.start_received:
            rate.sleep()
        
        i = 0
        while i < len(self.goals) and not rospy.is_shutdown():
            # 超时跳转检查
            if not self.is_jumping and not self.goal6_published and self.start_time:
                current_time = rospy.Time.now().to_sec()
                elapsed_time = current_time - self.start_time
                if elapsed_time > self.timeout_duration and i < 5:  # 目标点6索引为5
                    rospy.logwarn(f"已超过{self.timeout_duration}秒未发布目标点6，执行跳转")
                    self.is_jumping = True  # 标记为跳转状态
                    i = 5  # 跳转到目标点6
                    continue  # 重新进入循环处理目标点6
            
            self.current_goal_index = i
            goal_x, goal_y = self.goals[i]
            rospy.loginfo(f"前往目标点 {i+1}/{len(self.goals)}: ({goal_x}, {goal_y}) {'(相对坐标)' if self.use_relative else '(绝对坐标)'}")
            if i in [7,6] :
                self.speed = 0.5
                
            else :
                self.speed = 0.3
            if i == 7:
                self.adjust_angle()
            # 移动到目标点
            if not self.reach_goal(goal_x, goal_y):
                if not self.goal6_published and i < 5:
                    self.is_jumping = True  # 标记为跳转状态
                    i = 5
                    continue
                else:
                    rospy.logerr("无法到达目标点，程序终止")
                    return
            
            # 跳转完成后重置状态
            if self.is_jumping and i == 5:
                self.is_jumping = False
                self.goal6_published = True  # 标记目标点6已发布
            
            # 标记目标点6已发布（正常流程）
            if i == 5 and not self.goal6_published:
                self.goal6_published = True
            
            # 角度调整
            if abs(self.current_yaw) > 0.15:  #可以调小点
                self.adjust_angle()
            
            # 目标点8特殊操作
            if i == 7:
                rospy.loginfo("执行目标点8的特殊操作")
                self.publish_velocity(0.0, 0.4, 4.0)
                self.publish_velocity(-0.3, 0.0, 4.0)
                rospy.loginfo("发布/wheels/arrive_box信号 (目标点8)")
                self.arrive_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

                self.reference_x = self.current_x
                self.reference_y = self.current_y
                rospy.loginfo(f"设置参考零点: ({self.reference_x:.2f}, {self.reference_y:.2f})")
                self.use_relative = True
            
            # 发布到达信号
            if i in [1, 2, 3, 4, 9, 10, 11, 12]:
                rospy.loginfo(f"发布/wheels/arrive信号 (目标点{i+1})")
                self.arrive_pub.publish(Int32(1))
            
            # 目标点15发布箱子到达信号
            if i == 14:
                rospy.loginfo("发布/wheels/arrive_box信号 (目标点15)")
                self.arrive_box_pub.publish(Int32(1))
            
            # 等待YOLO信号
            if i in [1, 2, 3, 4,9, 10, 11, 12]:
                if not self.wait_for_yolo():
                    if not self.goal6_published and i < 5:
                        self.is_jumping = True
                        i = 5
                        continue
            
            i += 1
        
        rospy.loginfo("所有目标点已到达，导航完成")

if __name__ == '__main__':
    try:
        navigator = SimpleNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航程序被中断")