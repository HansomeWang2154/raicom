#!/usr/bin/env python
import rospy
import actionlib
import threading
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, Twist, PoseStamped
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler

class MultiGoalPublisher:
    def __init__(self):
        rospy.init_node('multi_goal_publisher')

        # 初始化标志位
        self.start_received = False
        self.start_time = None
        self.goal6_published = False
        self.timeout_triggered = False
        self.last_operation_time = time.time()

        # 创建move_base客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # 创建速度发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器...")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("无法连接到move_base服务器!")
            rospy.signal_shutdown("move_base服务器连接失败")
            return
        rospy.loginfo("成功连接move_base服务器")

        # 创建发布者和订阅者
        self.arrived_pub = rospy.Publisher('/wheels/arrive', Int32, queue_size=10)
        self.arrived_box_pub = rospy.Publisher('/wheels/arrive_box', Int32, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/yolo/picked', Int32, self.yolo_callback)
        self.start_sub = rospy.Subscriber('/wheels/start', Int32, self.start_callback)
        self.yolo_received = False

        # 定义目标位置
        self.goal_list = [
            (0.90, 1.90, 0.0),     # 目标点1
            (1.25, 1.90, 0.0),     # 目标点2
            (2.8, 1.90, 0.0),      # 目标点3
            (4.2, 1.90, 0.0),      # 目标点4
            (5.8, 1.90, 0.0),      # 目标点5
            (7.0, 1.5, 0.0),      # 目标点6
            (7.0, 2.5, 0.0),      # 目标点7
            (7.0, 4.5, 0.0),      # 目标点8
            (0.5, 3.5, 0.0),      # 目标点9
            (2.5, 3.5, 0.0),      # 目标点10
            (0.8, 4.5, 0.0),      # 目标点11
            (0.70, 3.0, 0.0),     # 目标点12
            (1.15, 3.5, 0.0),     # 目标点13
            (2.7, 3.50, 0.0),     # 目标点14
            (4.1, 3.50, 0.0),     # 目标点15
            (5.7, 3.50, 0.0),     # 目标点16
            (4.0, 4.5, 0.0),      # 目标点17
            (1.0, 4.5, 0.0),      # 目标点18
            (0.8, 4.5, 0.0)       # 目标点19
        ]

        # 状态管理
        self.current_goal_index = 0
        self.goal_active = False
        self.goal_completed = False

        # 启动定时任务
        rospy.Timer(rospy.Duration(10.0), self.print_runtime)  # 每10秒打印一次运行时间
        rospy.Timer(rospy.Duration(1.0), self.check_timeout)   # 每秒检查超时

        rospy.loginfo("等待/wheels/start消息...")

        # 确保节点关闭时清理资源
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        """节点关闭时的清理工作"""
        if self.goal_active:
            self.client.cancel_goal()
        rospy.loginfo(f"[{time.time()}] 节点关闭，清理资源完成")

    def start_callback(self, msg):
        """处理启动消息"""
        if msg.data == 1 and not self.start_received:
            self.start_received = True
            self.start_time = time.time()
            rospy.loginfo(f"[{self.start_time}] 收到/wheels/start消息，开始导航任务")

            # 在新线程中启动导航以避免阻塞
            nav_thread = threading.Thread(target=self.publish_next_goal, daemon=True)
            nav_thread.start()

    def check_timeout(self, event=None):
        """检查超时"""
        if self.start_time and not self.goal6_published and not self.timeout_triggered:
            elapsed = time.time() - self.start_time
            if elapsed > 30:  # 超过30秒
                rospy.logwarn(f"[{time.time()}] 超过30秒未到达目标点6，强制跳转")
                self.timeout_triggered = True
                if self.goal_active:
                    self.client.cancel_goal()
                    rospy.sleep(0.5)
                self.current_goal_index = 5  # 直接跳到目标点6
                threading.Thread(target=self.publish_next_goal, daemon=True).start()

    def print_runtime(self, event):
        """打印运行时间"""
        if self.start_time:
            elapsed_time = time.time() - self.start_time
            rospy.loginfo(f"[{time.time()}] 程序已运行 {elapsed_time:.2f} 秒")

    def publish_velocity(self, linear_x, linear_y, duration):
        """发布速度指令并保持指定时间"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = 0.0
        
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo(f"[{start_time}] 发布速度指令: x方向 {linear_x} m/s, y方向 {linear_y} m/s (持续{duration}秒)")
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"[{time.time()}] 速度指令结束")

    def yolo_callback(self, msg):
        """处理YOLO检测消息"""
        if msg.data == 1:
            self.last_operation_time = time.time()
            rospy.loginfo(f"[{self.last_operation_time}] 收到YOLO检测消息")
            self.yolo_received = True

    def publish_next_goal(self):
        """发布下一个目标点"""
        try:
            current_time = time.time()

            if self.current_goal_index >= len(self.goal_list):
                rospy.loginfo(f"[{current_time}] 所有目标点已完成!")
                rospy.signal_shutdown("任务完成")
                return

            # 获取当前目标
            goal_info = self.goal_list[self.current_goal_index]
            x, y, theta = goal_info

            # 创建目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = Point(x, y, 0.0)
            quat = quaternion_from_euler(0, 0, theta)
            goal.target_pose.pose.orientation = Quaternion(*quat)

            # 发布目标
            rospy.loginfo(f"[{current_time}] 发布目标#{self.current_goal_index+1}: 位置({x:.1f}, {y:.1f}), 朝向: {theta:.2f} 弧度")

            # 异步发送目标
            self.client.send_goal(goal, done_cb=self.goal_done_cb)
            self.goal_active = True

        except Exception as e:
            rospy.logerr(f"[{time.time()}] 发布目标异常: {str(e)}")

    def goal_done_cb(self, status, result):
        """目标完成回调"""
        current_time = time.time()

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"[{current_time}] 成功到达目标#{self.current_goal_index+1}")
        else:
            rospy.logwarn(f"[{current_time}] 无法到达目标#{self.current_goal_index+1}，状态代码: {status}")

        # 更新目标点索引
        self.current_goal_index += 1
        threading.Thread(target=self.publish_next_goal, daemon=True).start()

if __name__ == '__main__':
    try:
        node = MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
