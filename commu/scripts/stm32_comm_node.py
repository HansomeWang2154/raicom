#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf

class Stm32CommNode:
    def __init__(self):
        rospy.init_node('stm32_comm_node')
        
        # 参数设置
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.timeout = rospy.get_param('~timeout', 1.0)
        self.rate = rospy.get_param('~rate', 10)  # Hz
        
        # 串口初始化

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            rtscts=True  # 启用硬件流控
        )


        if not self.serial.is_open:
            self.serial.open()
        
        # ROS发布和订阅
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 变量初始化
        self.last_cmd_time = rospy.Time.now()
        self.cmd_vel_timeout = rospy.Duration(0.5)  # 0.5秒无命令则停止
        
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        self.last_cmd_time = rospy.Time.now()
        
        # 构建数据包 (0xAA + 4字节线速度 + 4字节角速度 + 0xBB)
        packet = bytearray()
        packet.append(0xAA)  # 帧头
        
        # 添加线速度 (float 4字节)
        packet.extend(struct.pack('<f', msg.linear.x))
        
        # 添加角速度 (float 4字节)
        packet.extend(struct.pack('<f', msg.angular.z))
        
        packet.append(0xBB)  # 帧尾
        
        # 发送数据
        try:
            self.serial.write(packet)
        except serial.SerialException as e:
            rospy.logerr("Serial write error: %s", str(e))
    
    def parse_odom_data(self, data):
        """解析里程计数据"""
        if len(data) != 20:  # 5个float (4字节每个) + 1字节换行符
            return None
        
        try:
            # 解析二进制数据 (小端格式)
            pos_x = struct.unpack('<f', data[0:4])[0]
            pos_y = struct.unpack('<f', data[4:8])[0]
            ang_rad = struct.unpack('<f', data[8:12])[0]
            v_linear = struct.unpack('<f', data[12:16])[0]
            v_angular = struct.unpack('<f', data[16:20])[0]
            
            return pos_x, pos_y, ang_rad, v_linear, v_angular
        except struct.error as e:
            rospy.logerr("Data parsing error: %s", str(e))
            return None
    
    def publish_odom(self, pos_x, pos_y, ang_rad, v_linear, v_angular):
        """发布里程计信息"""
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # 设置位置
        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = 0.0
        
        # 设置方向 (从偏航角创建四元数)
        quat = tf.transformations.quaternion_from_euler(0, 0, ang_rad)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # 设置速度
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        
        self.odom_pub.publish(odom)
    
    def run(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # 检查命令超时
            if (rospy.Time.now() - self.last_cmd_time) > self.cmd_vel_timeout:
                # 发送停止命令
                zero_cmd = Twist()
                self.cmd_vel_callback(zero_cmd)
            
            # # 读取串口数据
            # if self.serial.in_waiting >= 21:  # 等待完整数据包
            #     data = self.serial.read(21)
            #     if data[-1] == ord('\n'):  # 检查结束符
            #         odom_data = self.parse_odom_data(data[:-1])  # 去掉换行符
            #         if odom_data:
            #             self.publish_odom(*odom_data)
            
            rate.sleep()
        
        # 关闭串口
        self.serial.close()

if __name__ == '__main__':
    try:
        node = Stm32CommNode()
        node.run()
    except rospy.ROSInterruptException:
        pass