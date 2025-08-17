#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, Twist
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
import threading


import rospy
from std_srvs.srv import Empty
import threading


import rospy
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter

import rospy
from dynamic_reconfigure.client import Client


def create_pose_stamped(self, x, y, theta):
    """创建带时间戳的位姿"""
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, theta)
    pose.pose.orientation = Quaternion(*quat)
    return pose

def publish_estimated_pose(self, pose):
    """发布估计位姿"""
    # 创建一个临时发布者
    pose_pub = rospy.Publisher('/estimated_pose', geometry_msgs.msg.PoseStamped, queue_size=10, latch=True)
    
    # 发布位姿
    pose_pub.publish(pose)
    rospy.loginfo(f"已发布估计位姿: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
    
    # 确保消息已发送
    rospy.sleep(0.1)


def set_dwa_param():
    client = Client("/move_base/DWAPlannerROS", timeout=5)
    # 非阻塞更新参数
    client.update_configuration({"max_vel_theta": 1.0})
    client.update_configuration({"min_vel_theta": 1.0})
    client.update_configuration({"acc_lim_theta": 1.0})

def set_move_base_param(param_name, value):
    try:
        # 根据参数类型选择服务
        if "DWAPlannerROS" in param_name:
            service_name = "/move_base/DWAPlannerROS/set_parameters"
        elif "global_costmap" in param_name:
            service_name = "/move_base/global_costmap/set_parameters"
        elif "local_costmap" in param_name:
            service_name = "/move_base/local_costmap/set_parameters"
        else:
            service_name = "/move_base/set_parameters"
        
        rospy.wait_for_service(service_name, timeout=2.0)
        set_param = rospy.ServiceProxy(service_name, Reconfigure)
        
        # 构建参数配置
        config = Config()
        param = DoubleParameter()
        param.name = param_name.split('/')[-1]  # 提取参数名
        param.value = float(value)
        config.doubles = [param]
        
        # 非阻塞调用
        future = set_param.call_async(config)
        return future
    except Exception as e:
        rospy.logerr(f"参数设置失败: {str(e)}")
        return None



def reload_params_non_blocking():
    """非阻塞参数重载函数"""
    def _reload_thread():
        try:
            rospy.loginfo("开始异步重载参数...")
            rospy.wait_for_service('/move_base/reload_parameters', timeout=2.0)
            reload_srv = rospy.ServiceProxy('/move_base/reload_parameters', Empty)
            
            # 带超时的非阻塞调用
            future = reload_srv.call_async(Empty._request_class())
            start_time = rospy.Time.now()
            
            while not future.done() and (rospy.Time.now() - start_time).to_sec() < 3.0:
                rospy.sleep(0.1)
                
            if future.done():
                rospy.loginfo("参数重载成功")
            else:
                rospy.logwarn("参数重载超时")
                
        except Exception as e:
            rospy.logerr(f"参数重载线程异常: {str(e)}")

    # 启动独立线程执行重载
    threading.Thread(target=_reload_thread, daemon=True).start()



class MultiGoalPublisher:
    def __init__(self):
        rospy.init_node('multi_goal_publisher')
        
        # 创建move_base客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 创建速度发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
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
        self.arrived_box_pub = rospy.Publisher('/wheels/arrive_box', Int32, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/yolo/picked', Int32, self.yolo_callback)
        self.yolo_received = False
        
        # 定义目标位置
        self.goal_list = [
            (0.55, 0.2, 0.0),     # 目标点1
            (0.55, 0.70, 0.0),   # 目标点2
            (2.21, 0.65, 0.0),   # 目标点3
            (3.5, 0.45, 0.0),    # 目标点4
            (5.0, 0.38, 0.0),    # 目标点5
            (6.23, -0.30, 0.0),  # 目标点6
            (6.35, 1.03, 0.0),   # 目标点7
            (6.35, 3.0, 0.0),   # 目标点8
            (4.0, 3.5, 0.0),     # 目标点9
            (1.0, 3.5, 0.0),     # 目标点10
            (0.40, 3.8, 0.0),    # 目标点11 到达后发布arrive_box消息，发送向后速度
            (0.70, 3.0, 0.0),    # 目标点12 - 发布前发送向前速度
            (0.82, 2.30, 0.0),   # 目标点13
            (2.11, 2.20, 0.0),   # 目标点14
            (3.7, 2.10, 0.0),    # 目标点15
            (5.4, 2.10, 0.0),    # 目标点16
            (4.0, 3.5, 0.0),     # 目标点17
            (1.0, 3.5, 0.0),     # 目标点18
            (0.40, 3.8, 0.0)     # 目标点19 到达后发布arrive_box消息，发送向后速度
        ]
        
        # 状态管理
        self.current_goal_index = 0
        self.goal_active = False
        self.goal_completed = False
        self.state_lock = threading.Lock()
        
        # 启动目标发布流程
        self.publish_next_goal()
    
    def yolo_callback(self, msg):
        """处理YOLO检测消息"""
        if msg.data == 1:
            rospy.loginfo("收到YOLO检测消息")
            self.yolo_received = True
    
    def publish_velocity(self, linear_x, linear_y, duration):
        """发布速度指令并保持指定时间"""
        twist = Twist()
        twist.linear.x = linear_x  # x方向速度
        twist.linear.y = linear_y  # y方向速度
        twist.angular.z = 0.0      # 角速度为0
        
        start_time = rospy.Time.now()
        
        rospy.loginfo(f"发布速度指令: x方向 {linear_x} m/s, y方向 {linear_y} m/s (持续{duration}秒)")
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        # 停止
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("速度指令结束")

    
    def publish_next_goal(self):
        with self.state_lock:
            if self.current_goal_index >= len(self.goal_list):
                rospy.loginfo("所有目标点已完成!")
                rospy.signal_shutdown("任务完成")
                return
            
            # 在发布目标点12前发送向前速度
            if self.current_goal_index == 11:  # 目标点12
            
                self.publish_velocity(0.2, 0.0, 2.0)  # x方向-0.1，y方向0.1
                self.publish_velocity(0.0, -0.2, 2.0)  # x方向-0.1，y方向0.1
                rospy.sleep(0.5)  # 等待速度指令完成
            
            # 确保前一个目标已完全清理
            if self.goal_active:
                rospy.logwarn("取消当前活动目标")
                self.client.cancel_goal()
                rospy.sleep(0.5)
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
            
            # 检查客户端状态
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
        with self.state_lock:
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
            
            # 特殊点处理
            if self.current_goal_index == 10:  # 目标点11
                rospy.loginfo("发布到达box消息")
                # 设置新的运动参数（旋转相关）
                # # 设置新参数
                # rospy.set_param('/move_base/DWAPlannerROS/max_vel_theta', 1.0)
                # rospy.set_param('/move_base/DWAPlannerROS/min_vel_theta', -1.0)
                # rospy.set_param('/move_base/DWAPlannerROS/acc_lim_theta', 1.0)

                # # 非阻塞重载
                # reload_params_non_blocking()

                # set_move_base_param("/move_base/DWAPlannerROS/max_vel_theta", 1.0)
                # set_move_base_param("/move_base/DWAPlannerROS/min_vel_theta", 1.0)
                # set_move_base_param("/move_base/DWAPlannerROS/acc_lim_theta", 1.0)
                # set_dwa_param()
                # 主线程继续执行其他任务
                rospy.loginfo("参数重载已启动，主线程继续运行...")
                self.publish_velocity(-0.0, 0.2, 2.0)  # x方向-0.1，y方向0.1
                self.publish_velocity(-0.2, 0.0, 1.5)  # x方向-0.1，y方向0.1

                self.arrived_box_pub.publish(Int32(1))
                estimated_pose = self.create_pose_stamped(0.55, 3.50, 0.0)
                self.publish_estimated_pose(estimated_pose)
                rospy.sleep(5.0)  # 延时5秒
            
            elif self.current_goal_index == 18:  # 目标点19
                rospy.loginfo("发布到达box消息")
                
                self.publish_velocity(-0.0, 0.2, 2.0)  # x方向-0.1，y方向0.1
                self.publish_velocity(-0.2, 0.0, 1.5)  # x方向-0.1，y方向0.1
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)  # 延时5秒
            
            elif self.current_goal_index in [1, 2, 3, 4, 12, 13, 14, 15]:  # 需要等待YOLO的目标点
                rospy.loginfo("发布到达消息")
                self.arrived_pub.publish(Int32(1))
                
                rospy.loginfo("等待YOLO检测消息...")
                self.yolo_received = False
                while not self.yolo_received and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                rospy.loginfo("收到YOLO检测消息")
            
            elif self.current_goal_index in [8, 9, 11, 16, 17]:  # 需要立即发布下一个目标点的点
                rospy.loginfo("目标点#{self.current_goal_index+1}完成，立即发布下一个目标")
            
            # 准备下一个目标
            self.current_goal_index += 1
            
            # 延迟发布下一个目标
            rospy.Timer(rospy.Duration(0.5), self.delayed_publish, oneshot=True)
    
    def delayed_publish(self, event):
        """延迟发布下一个目标"""
        self.publish_next_goal()
    
    def timeout_cb(self, event):
        with self.state_lock:
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
            
            # 根据目标点序号执行不同操作
            if self.current_goal_index == 10:  # 目标点11
                rospy.loginfo("发布到达box消息")
                
                self.publish_velocity(-0.0, 0.2, 2.0)  # x方向-0.1，y方向0.1
                self.publish_velocity(-0.2, 0.0, 1.5)  # x方向-0.1，y方向0.1
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)
            
            elif self.current_goal_index == 18:  # 目标点19
                rospy.loginfo("发布到达box消息")
                self.publish_velocity(-0.0, 0.2, 2.0)  # x方向-0.1，y方向0.1
                self.publish_velocity(-0.2, 0.0, 1.5)  # x方向-0.1，y方向0.1
                self.arrived_box_pub.publish(Int32(1))
                rospy.sleep(5.0)
            
            elif self.current_goal_index in [1, 2, 3, 4, 12, 13, 14, 15]:  # 需要等待YOLO的目标点
                rospy.loginfo("发布到达消息")
                self.arrived_pub.publish(Int32(1))
                
                rospy.loginfo("等待YOLO检测消息...")
                self.yolo_received = False
                while not self.yolo_received and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                rospy.loginfo("收到YOLO检测消息")
            
            elif self.current_goal_index in [8, 9, 11, 16, 17]:  # 需要立即发布下一个目标点的点
                rospy.loginfo("目标点#{self.current_goal_index+1}超时，继续下一个目标")
            
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