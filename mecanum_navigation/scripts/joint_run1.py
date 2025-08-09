#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
import threading

class MultiGoalPublisher:
    def __init__(self):
        rospy.init_node('multi_goal_publisher')
        
        # 创建move_base客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器...")
        server_ready = self.client.wait_for_server(rospy.Duration(10.0))
        if not server_ready:
            rospy.logerr("无法连接到move_base服务器!")
            rospy.signal_shutdown("move_base服务器连接失败")
            return
        
        rospy.loginfo("成功连接move_base服务器")
        
        # 创建发布者和订阅者
        self.arrived_pub = rospy.Publisher('/wheels/arrive', Int32, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/yolo/picked', Int32, self.yolo_callback)
        self.yolo_received = False
        
        # 定义目标位置
        self.goal_list = [
            (0.85, 0.0, 0.0), # 目标点1 开机就发布，到达或超时后直接发布目标点2
            (0.85, 0.55, 0.0), # 目标点2 到达或超时后向/wheels/arrive发布消息，并等待/yolo/picked话题,接收到消息后发布目标点3
            (2.21, 0.50, 0.0), # 目标点3 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题，接收到消息后发布目标点4
            (3.5, 0.45, 0.0),  # 目标点4 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题，接受到消息后发布目标点5
            (5.0, 0.40, 0.0),  # 目标点5 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题， 接收到消息后发布目标点6
            (6.30, -0.31, 0.0), #目标点6 到达或超时后直接发布目标点7
            (6.47, 1.03, 0.0),  # 目标点7 到达或超时后直接发布目标点8
            (6.34, 2.39, 0.0),  # 目标点8 到达或超时后直接发布目标点9
            (0.92, 3.84, 0.0),    # 目标点9 到达或超时后向/wheels/arrive_box发布消息，延时5后发布目标点10
            (0.92, 3.00, 0.0),   # 目标点10 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题， 接收到消息后发布目标点11
            (2.21, 3.00, 0.0)    # 目标点11 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题， 接收到消息后发布目标点12
            (3.5, 3.00, 0.0),  # 目标点12 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题，接受到消息后发布目标点13
            (5.0, 3.00, 0.0),  # 目标点13 到达或超时后后向/wheels/arrive发布消息，并等待/yolo/picked话题， 接收到消息后发布目标点14
            (0.92, 3.84, 0.0),    # 目标点14 到达或超时后向/wheels/arrive_box发布消息，延时5后结束程序
        ]
        
        # 状态管理
        self.current_goal_index = 0
        self.goal_active = False
        self.goal_completed = False
        self.state_lock = threading.Lock()  # 添加状态锁
        
        # 启动目标发布流程
        self.publish_next_goal()
    
    def yolo_callback(self, msg):
        """处理YOLO检测消息"""
        if msg.data == 1:
            rospy.loginfo("收到YOLO检测消息")
            self.yolo_received = True
    
    def publish_next_goal(self):
        with self.state_lock:  # 加锁保护状态
            if self.current_goal_index >= len(self.goal_list):
                rospy.loginfo("所有目标点已完成!")
                rospy.signal_shutdown("任务完成")
                return
            
            # 确保前一个目标已完全清理
            if self.goal_active:
                rospy.logwarn("取消当前活动目标")
                self.client.cancel_goal()
                rospy.sleep(0.5)  # 等待取消完成
                self.goal_active = False
            
            # 重置状态标志
            self.goal_completed = False
            self.yolo_received = False
            
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
            rospy.loginfo(f"发布目标#{self.current_goal_index+1}: 位置({x:.1f}, {y:.1f}), 朝向: {theta:.2f} 弧度")
            
            # 关键修复：检查客户端状态
            if self.client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.logwarn("客户端仍在处理目标，先取消")
                self.client.cancel_goal()
                rospy.sleep(0.2)
            
            self.client.send_goal(goal, done_cb=self.goal_done_cb)
            self.goal_active = True
            
            # 设置20秒超时
            if hasattr(self, 'timeout_timer'):
                self.timeout_timer.shutdown()
            self.timeout_timer = rospy.Timer(rospy.Duration(20.0), self.timeout_cb, oneshot=True)
            rospy.loginfo(f"等待20秒，如果超时将放弃目标#{self.current_goal_index+1}")
    
    def goal_done_cb(self, status, result):
        with self.state_lock:  # 加锁保护状态
            if self.goal_completed:
                return
            self.goal_completed = True
            
            # 取消超时计时器
            if hasattr(self, 'timeout_timer') and self.timeout_timer.is_alive():
                self.timeout_timer.shutdown()
            
            # 标记目标不再活动
            self.goal_active = False
            
            # 检查目标状态
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"成功到达目标#{self.current_goal_index+1}")
            else:
                rospy.logwarn(f"无法到达目标#{self.current_goal_index+1}，状态代码: {status}")
            
            # 发布到达消息
            rospy.loginfo("发布到达消息")
            self.arrived_pub.publish(Int32(1))
            
            # 如果是前8个目标，等待YOLO检测消息
            if self.current_goal_index < 8:
                rospy.loginfo("等待YOLO检测消息...")
                self.yolo_received = False
                while not self.yolo_received and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                rospy.loginfo("收到YOLO检测消息")
            
            # 准备下一个目标
            self.current_goal_index += 1
            
            # 关键修复：延迟发布下一个目标
            rospy.Timer(rospy.Duration(0.5), self.delayed_publish, oneshot=True)
    
    def delayed_publish(self, event):
        """延迟发布下一个目标"""
        self.publish_next_goal()
    
    def timeout_cb(self, event):
        with self.state_lock:  # 加锁保护状态
            if self.goal_completed:
                return
            self.goal_completed = True
                
            if not self.goal_active:
                return
                
            rospy.logwarn(f"目标#{self.current_goal_index+1} 20秒超时，取消当前目标")
            
            # 取消当前目标
            self.client.cancel_goal()
            rospy.sleep(0.5)
            self.goal_active = False
            
            # 发布到达消息
            rospy.loginfo("发布到达消息")
            self.arrived_pub.publish(Int32(1))
            
            # 如果是前8个目标，等待YOLO检测消息
            if self.current_goal_index < 8:
                rospy.loginfo("等待YOLO检测消息...")
                self.yolo_received = False
                while not self.yolo_received and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                rospy.loginfo("收到YOLO检测消息")
            
            # 准备下一个目标
            self.current_goal_index += 1
            
            # 关键修复：延迟发布下一个目标
            rospy.Timer(rospy.Duration(0.5), self.delayed_publish, oneshot=True)

if __name__ == '__main__':
    try:
        node = MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass