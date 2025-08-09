#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from tf.transformations import quaternion_from_euler

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
        
        # 创建服务
        self.run_service = rospy.Service('/wheel/run', Trigger, self.handle_run_service)
        self.yolo_pick_client = rospy.ServiceProxy('/yolo/pick', Trigger)
        
        # 定义9个目标位置 (x, y, theta角度) - 根据您的实际地图修改这些坐标
        self.goal_list = [
            (1.0, 0.69, 0.0),     # 目标点1
            (1.95, 0.3, 0.0),     # 目标点2
            (2.5, 0.3, 0.0),     # 目标点3
            (3.0, 0.3, 0.0),     # 目标点4
            (4.0, 0.3, 0.0),     # 目标点5
            (5.0, 0.3, 0.0),     # 目标点6
            (6.0, 0.3, 0.0),     # 目标点7
            (6.5, 0.3, 0.0),     # 目标点8
            (7.0, 0.3, 0.0)      # 目标点9
        ]
        
        # 初始化状态
        self.current_goal_index = 0
        self.goal_active = False
        self.waiting_for_run = False
        self.running = False
        
        rospy.loginfo("等待/wheel/run服务调用以开始导航...")
    
    def handle_run_service(self, req):
        """处理/wheel/run服务调用"""
        if not self.running:
            # 开始导航流程
            self.running = True
            self.publish_next_goal()
            return TriggerResponse(success=True, message="导航已启动")
        elif self.waiting_for_run:
            # 继续下一个目标
            self.waiting_for_run = False
            self.publish_next_goal()
            return TriggerResponse(success=True, message="继续下一个目标")
        else:
            return TriggerResponse(success=False, message="导航已在运行中")
    
    def publish_next_goal(self):
        if self.current_goal_index >= len(self.goal_list):
            rospy.loginfo("所有目标点已完成!")
            self.running = False
            return
        
        # 确保前一个目标已完全清理
        if self.goal_active:
            rospy.logwarn("取消当前活动目标")
            self.client.cancel_goal()
            rospy.sleep(0.5)  # 等待取消完成
            self.goal_active = False
        
        # 获取当前目标
        goal_info = self.goal_list[self.current_goal_index]
        x, y, theta = goal_info
        
        # 创建目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 确保使用正确坐标系
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position = Point(x, y, 0.0)
        
        # 设置方向 - 将欧拉角转换为四元数
        quat = quaternion_from_euler(0, 0, theta)  # (roll, pitch, yaw)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        
        # 发布目标
        rospy.loginfo(f"发布目标#{self.current_goal_index+1}: 位置({x:.1f}, {y:.1f}), 朝向: {theta:.2f} 弧度")
        self.client.send_goal(goal, done_cb=self.goal_done_cb)
        self.goal_active = True
        
        # 设置20秒超时
        self.timeout_timer = rospy.Timer(rospy.Duration(20.0), self.timeout_cb, oneshot=True)
        rospy.loginfo(f"等待20秒，如果超时将放弃目标#{self.current_goal_index+1}")
    
    def goal_done_cb(self, status, result):
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
        
        # 如果是前8个目标，调用YOLO pick服务
        if self.current_goal_index < 8:  # 前8个目标索引0-7
            try:
                rospy.loginfo("调用/yolo/pick服务...")
                response = self.yolo_pick_client(TriggerRequest())
                if response.success:
                    rospy.loginfo("YOLO pick服务调用成功")
                else:
                    rospy.logwarn(f"YOLO pick服务调用失败: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"调用YOLO pick服务失败: {str(e)}")
        
        # 准备下一个目标
        self.current_goal_index += 1
        
        # 等待/wheel/run服务调用
        self.waiting_for_run = True
        rospy.loginfo("等待/wheel/run服务调用以继续下一个目标...")
    
    def timeout_cb(self, event):
        if not self.goal_active:
            return  # 如果目标已不活动，忽略超时
            
        rospy.logwarn(f"目标#{self.current_goal_index+1} 20秒超时，取消当前目标")
        
        # 取消当前目标
        self.client.cancel_goal()
        
        # 等待确认取消
        rospy.sleep(0.5)
        self.goal_active = False
        
        # 发布到达消息
        rospy.loginfo("发布到达消息")
        self.arrived_pub.publish(Int32(1))
        
        # 如果是前8个目标，调用YOLO pick服务
        if self.current_goal_index < 8:  # 前8个目标索引0-7
            try:
                rospy.loginfo("调用/yolo/pick服务...")
                response = self.yolo_pick_client(TriggerRequest())
                if response.success:
                    rospy.loginfo("YOLO pick服务调用成功")
                else:
                    rospy.logwarn(f"YOLO pick服务调用失败: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"调用YOLO pick服务失败: {str(e)}")
        
        # 准备下一个目标
        self.current_goal_index += 1
        
        # 等待/wheel/run服务调用
        self.waiting_for_run = True
        rospy.loginfo("等待/wheel/run服务调用以继续下一个目标...")

if __name__ == '__main__':
    try:
        node = MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass