#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
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
        
        # 定义三个目标位置 (x, y, theta角度) - 根据您的实际地图修改这些坐标
        self.goal_list = [
            (1.0, 0.5, 0.0),     # 目标点1
            (2.0, -1.0, 1.57),   # 目标点2 (90度)
            (0.5, 1.5, 3.14)     # 目标点3 (180度)
        ]
        
        # 启动目标发布流程
        self.current_goal_index = 0
        self.publish_next_goal()
    
    def publish_next_goal(self):
        if self.current_goal_index >= len(self.goal_list):
            rospy.loginfo("所有目标点已完成!")
            rospy.signal_shutdown("任务完成")
            return
        
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
        
        # 设置20秒超时
        self.timeout_timer = rospy.Timer(rospy.Duration(20.0), self.timeout_cb, oneshot=True)
        rospy.loginfo(f"等待20秒，如果超时将放弃目标#{self.current_goal_index+1}")
    
    def goal_done_cb(self, status, result):
        # 取消超时计时器
        if hasattr(self, 'timeout_timer') and self.timeout_timer.is_alive():
            self.timeout_timer.shutdown()
        
        # 检查目标状态
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"成功到达目标#{self.current_goal_index+1}")
        else:
            rospy.logwarn(f"无法到达目标#{self.current_goal_index+1}，状态代码: {status}")
        
        # 移动到下一个目标前等待3秒
        rospy.loginfo("等待3秒后继续...")
        rospy.sleep(3.0)
        
        # 准备下一个目标
        self.current_goal_index += 1
        self.publish_next_goal()
    
    def timeout_cb(self, event):
        rospy.logwarn(f"目标#{self.current_goal_index+1} 20秒超时，取消当前目标")
        
        # 取消当前目标
        self.client.cancel_goal()
        
        # 等待确认取消
        rospy.sleep(0.5)
        
        # 移动到下一个目标
        self.current_goal_index += 1
        self.publish_next_goal()

if __name__ == '__main__':
    try:
        node = MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass