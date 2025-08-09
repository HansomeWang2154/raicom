#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class TFStabilizer:
    def __init__(self):
        rospy.init_node('tf_stabilizer')
        
        # 参数
        self.smoothing_factor = 0.3  # 平滑系数
        self.max_deviation = 0.2      # 最大允许偏差
        
        # 状态
        self.last_transform = None
        # 直接发布到/tf话题
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
    def tf_callback(self, msg):
        # 查找odom->map变换
        odom_to_map = None
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                odom_to_map = transform
                break
        
        if not odom_to_map:
            # 如果没有找到目标变换，直接转发所有消息
            self.tf_pub.publish(msg)
            return
            
        # 初始化
        if self.last_transform is None:
            self.last_transform = odom_to_map
            return
            
        # 计算变化量
        dx = odom_to_map.transform.translation.x - self.last_transform.transform.translation.x
        dy = odom_to_map.transform.translation.y - self.last_transform.transform.translation.y
        
        # 应用平滑
        if abs(dx) > self.max_deviation or abs(dy) > self.max_deviation:
            rospy.logwarn("检测到TF突变，应用平滑")
            odom_to_map.transform.translation.x = self.last_transform.transform.translation.x + self.smoothing_factor * dx
            odom_to_map.transform.translation.y = self.last_transform.transform.translation.y + self.smoothing_factor * dy
        
        # 更新并发布
        self.last_transform = odom_to_map
        
        # 创建新的TF消息，替换原始变换
        new_transforms = []
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                # 使用平滑后的变换
                new_transforms.append(odom_to_map)
            else:
                # 保留其他变换
                new_transforms.append(transform)
        
        # 发布处理后的TF树
        self.tf_pub.publish(TFMessage(new_transforms))

if __name__ == '__main__':
    ts = TFStabilizer()
    rospy.spin()