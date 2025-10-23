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
        self.goal6_published = False  # 标记目标点6是否已发布
        self.last_operation_time = time.time()
        
        # 超时配置参数（单位：秒）
        self.TOTAL_MOTION_TIMEOUT = 240  # 总运动超时时间（4分20秒）
        
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
            (7.0, 1.5, 0.0),       # 目标点6
            (7.0, 2.5, 0.0),       # 目标点7
            (7.0, 4.5, 0.0),       # 目标点8
            (0.5, 3.5, 0.0),       # 目标点9
            (1.2, 3.5, 0.0),       # 目标点10
            (1.2, 4.5, 0.0),       # 目标点11
            (0.70, 3.0, 0.0),      # 目标点12
            (1.15, 3.5, 0.0),      # 目标点13
            (2.7, 3.50, 0.0),      # 目标点14
            (4.1, 3.50, 0.0),      # 目标点15
            (5.7, 3.50, 0.0),      # 目标点16
            (4.0, 4.5, 0.0),       # 目标点17
            (1.0, 4.5, 0.0),       # 目标点18
            (0.8, 4.5, 0.0)        # 目标点19
        ]
        
        # 状态管理
        self.current_goal_index = 0
        self.goal_active = False
        self.goal_completed = False
        self.state_lock = threading.Lock()
        
        rospy.loginfo("等待/wheels/start消息...")
        
        # 确保节点关闭时清理资源
        rospy.on_shutdown(self.shutdown_hook)
    
    def shutdown_hook(self):
        """节点关闭时的清理工作"""
        if self.goal_active:
            self.client.cancel_goal()
        rospy.loginfo(f"[{rospy.Time.now().to_sec()}] 节点关闭，清理资源完成")

    def start_callback(self, msg):
        """处理启动消息"""
        if msg.data == 1 and not self.start_received:
            self.start_received = True
            self.start_time = rospy.Time.now().to_sec()  # 使用ROS时间
            self.last_operation_time = self.start_time
            rospy.loginfo(f"[{self.start_time}] 收到/wheels/start消息，开始导航任务")
            
            # 在新线程中启动导航以避免阻塞
            nav_thread = threading.Thread(target=self.publish_next_goal, daemon=True)
            nav_thread.start()
    
    def yolo_callback(self, msg):
        """处理YOLO检测消息"""
        if msg.data == 1:
            self.last_operation_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"[{self.last_operation_time}] 收到YOLO检测消息")
            self.yolo_received = True
    
    def publish_velocity(self, linear_x, linear_y, duration):
        """发布速度指令并保持指定时间"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = 0.0
        
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo(f"[{start_time}] 发布速度指令: x方向 {linear_x} m/s, y方向 {linear_y} m/s (持续{duration}秒)")
        while (rospy.Time.now().to_sec() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"[{rospy.Time.now().to_sec()}] 速度指令结束")

    def create_pose_stamped(self, x, y, theta):
        """创建带时间戳的位姿"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation = Quaternion(*quat)
        return pose
    
    def publish_estimated_pose(self, pose):
        """发布估计位姿"""
        # 复用Publisher而非重复创建
        if not hasattr(self, 'pose_pub'):
            self.pose_pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=10, latch=True)
        self.pose_pub.publish(pose)
        rospy.loginfo(f"[{rospy.Time.now().to_sec()}] 已发布估计位姿: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        rospy.sleep(0.1)
    
    def publish_next_goal(self):
        """发布下一个目标点，增加底盘运动超时检查"""
        try:
            if not self.state_lock.acquire(blocking=True, timeout=2.0):
                rospy.logwarn(f"[{rospy.Time.now().to_sec()}] 获取状态锁超时")
                return

            current_time = rospy.Time.now().to_sec()
            self.last_operation_time = current_time

            # 超时检查逻辑
            if self.start_time and not self.goal6_published and self.current_goal_index < 6:
                total_elapsed = current_time - self.start_time
                if total_elapsed > self.TOTAL_MOTION_TIMEOUT:
                    rospy.logwarn(f"[{current_time}] 底盘运动超时! 总耗时 {total_elapsed:.1f}秒 > {self.TOTAL_MOTION_TIMEOUT}秒，强制跳转至目标点6")
                    
                    # 取消当前活动目标
                    if self.goal_active:
                        self.client.cancel_goal()
                        rospy.sleep(0.5)
                    
                    # 直接设置当前目标为6（索引5）
                    self.current_goal_index = 5
                    self.goal6_published = True  # 标记目标点6已发布

            # 在发布目标点12前发送向前速度
            if self.current_goal_index == 11:  # 目标点12
                self.publish_velocity(0.2, 0.0, 2.0)
                self.publish_velocity(0.0, -0.2, 2.0)
                rospy.sleep(0.5)  # 等待速度指令完成

            # 检查是否所有目标已完成
            if self.current_goal_index >= len(self.goal_list):
                rospy.loginfo(f"[{current_time}] 所有目标点已完成!")
                rospy.signal_shutdown("任务完成")
                return

            # 确保前一个目标已完全清理
            if self.goal_active:
                rospy.logwarn(f"[{current_time}] 取消当前活动目标")
                self.client.cancel_goal()
                rospy.sleep(0.5)
                self.goal_active = False

            # 重置状态标志
            self.goal_completed = False
            self.yolo_received = False

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

            # 发送目标
            try:
                self.client.send_goal(goal, done_cb=self.goal_done_cb)
                self.goal_active = True
                rospy.loginfo(f"[{current_time}] 目标发布成功")
            except Exception as e:
                rospy.logerr(f"[{current_time}] 目标发送失败: {str(e)}")
                self.goal_active = False

            # 设置20秒超时（使用正确的定时器关闭方法）
            if hasattr(self, 'timeout_timer'):
                self.timeout_timer.shutdown()  # 修复：使用shutdown()而非cancel()
            self.timeout_timer = rospy.Timer(rospy.Duration(20.0), self.timeout_cb, oneshot=True)
            rospy.loginfo(f"[{current_time}] 等待20秒，如果超时将放弃目标#{self.current_goal_index+1}")

        except Exception as e:
            rospy.logerr(f"[{rospy.Time.now().to_sec()}] 发布目标异常: {str(e)}")
        finally:
            if self.state_lock.locked():
                self.state_lock.release()

    def goal_done_cb(self, status, result):
        """目标完成回调"""
        with self.state_lock:
            current_time = rospy.Time.now().to_sec()
            self.last_operation_time = current_time

            if self.goal_completed:
                return
            self.goal_completed = True

            # 取消超时计时器（使用正确的方法）
            if hasattr(self, 'timeout_timer') and self.timeout_timer.is_alive():
                self.timeout_timer.shutdown()  # 修复：使用shutdown()而非cancel()

            # 标记目标不再活动
            self.goal_active = False

            # 检查目标状态
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[{current_time}] 成功到达目标#{self.current_goal_index+1}")
                # 标记目标点6是否已发布
                if self.current_goal_index == 5:  # 目标点6
                    self.goal6_published = True
            else:
                rospy.logwarn(f"[{current_time}] 无法到达目标#{self.current_goal_index+1}，状态代码: {status}")

            # 特殊点处理
            if self.current_goal_index == 10:  # 目标点11
                rospy.loginfo(f"[{current_time}] 发布到达box消息")
                self.publish_velocity(0.0, 0.2, 2.0)
                self.publish_velocity(-0.2, 0.0, 3.0)
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

            elif self.current_goal_index == 18:  # 目标点19
                rospy.loginfo(f"[{current_time}] 发布到达box消息")
                self.publish_velocity(0.0, 0.2, 2.0)
                self.publish_velocity(-0.2, 0.0, 3.0)
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

            elif self.current_goal_index in [1, 2, 3, 4, 12, 13, 14, 15]:  # 需要等待YOLO的目标点
                rospy.loginfo(f"[{current_time}] 发布到达消息")
                self.arrived_pub.publish(Int32(1))
                
                rospy.loginfo(f"[{current_time}] 等待YOLO检测消息...")
                self.yolo_received = False
                # 增加YOLO等待超时（30秒）
                yolo_start = current_time
                while not self.yolo_received and not rospy.is_shutdown():
                    if rospy.Time.now().to_sec() - yolo_start >1000:
                        rospy.logwarn("等待YOLO消息超时，继续下一个目标")
                        break
                    rospy.sleep(0.1)
                rospy.loginfo(f"[{rospy.Time.now().to_sec()}] 退出YOLO等待")

            # 准备下一个目标
            self.current_goal_index += 1

            # 延迟发布下一个目标
            rospy.Timer(rospy.Duration(0.5), self.delayed_publish, oneshot=True)

    def delayed_publish(self, event):
        """延迟发布下一个目标"""
        threading.Thread(target=self.publish_next_goal, daemon=True).start()

    def timeout_cb(self, event):
        """单目标超时处理"""
        with self.state_lock:
            current_time = rospy.Time.now().to_sec()
            self.last_operation_time = current_time

            if self.goal_completed:
                return
            self.goal_completed = True

            if not self.goal_active:
                return

            rospy.logwarn(f"[{current_time}] 目标#{self.current_goal_index+1} 20秒超时，取消当前目标")
            self.client.cancel_goal()
            rospy.sleep(0.5)
            self.goal_active = False

            # 根据目标点序号执行不同操作
            if self.current_goal_index == 10:  # 目标点11
                rospy.loginfo(f"[{current_time}] 发布到达box消息")
                self.publish_velocity(0.0, 0.2, 2.0)
                self.publish_velocity(-0.2, 0.0, 3.0)
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

            elif self.current_goal_index == 18:  # 目标点19
                rospy.loginfo(f"[{current_time}] 发布到达box消息")
                self.publish_velocity(0.0, 0.2, 2.0)
                self.publish_velocity(-0.2, 0.0, 3.0)
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)

            elif self.current_goal_index in [1, 2, 3, 4, 12, 13, 14, 15]:  # 需要等待YOLO的目标点
                rospy.loginfo(f"[{current_time}] 发布到达消息")
                self.arrived_pub.publish(Int32(1))
                
                rospy.loginfo(f"[{current_time}] 等待YOLO检测消息...")
                self.yolo_received = False
                # 增加YOLO等待超时（30秒）
                yolo_start = current_time
                while not self.yolo_received and not rospy.is_shutdown():
                    if rospy.Time.now().to_sec() - yolo_start > 30:
                        rospy.logwarn("等待YOLO消息超时，继续下一个目标")
                        break
                    rospy.sleep(0.1)
                rospy.loginfo(f"[{rospy.Time.now().to_sec()}] 退出YOLO等待")

            # 准备下一个目标
            self.current_goal_index += 1

            # 延迟发布下一个目标
            rospy.Timer(rospy.Duration(0.5), self.delayed_publish, oneshot=True)

if __name__ == '__main__':
    try:
        node = MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
