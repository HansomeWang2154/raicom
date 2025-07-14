#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度指令串口发送节点prototype"""
import rospy
from geometry_msgs.msg import Twist
import serial
import struct

class VelocitySerialNode:
    def __init__(self):
        rospy.init_node('velocity_serial_node')
        
        # 获取参数
        port = rospy.get_param('~port', '/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 921600)
        
        # 打开串口
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            rospy.loginfo(f"成功连接串口 {port}，波特率 {baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"无法打开串口 {port}: {e}")
            rospy.signal_shutdown("串口错误")
            return

        # 协议参数 (与STM32端完全匹配)
        self.PROTOCOL_HEADER = 0x3F
        self.PROTOCOL_CMD_FLAG = 0x21
        self.PROTOCOL_FOOTER = 0x21
        self.MAX_SPEED = 1.0  # m/s
        self.MAX_ANGULAR = 1.0  # rad/s

        # 订阅速度指令话题
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
        
        rospy.loginfo("速度指令串口节点准备就绪")
    

    def velocity_callback(self, msg):
        """ 完全匹配STM32解析逻辑的速度指令发送 """
        try:
            # 1. 速度归一化处理 (-1.0 ~ 1.0)
            vx = max(min(msg.linear.x, self.MAX_SPEED), -self.MAX_SPEED)
            vy = max(min(msg.linear.y, self.MAX_SPEED), -self.MAX_SPEED)
            omega = max(min(msg.angular.z, self.MAX_ANGULAR), -self.MAX_ANGULAR)
            
            # 2. 转换为STM32协议格式 (0-255)
            # 注意：STM32端有 -((int)buf - 128)/128.0f 的处理
            vx_byte = int(128 - vx * 128)
            vy_byte = int(128 - vy * 128)
            omega_byte = int(128 - omega * 128)
            
            # 3. 构建数据包 (完全匹配STM32解析结构)
            buf = bytearray()
            buf.append(self.PROTOCOL_HEADER)    # 0x3F
            buf.append(self.PROTOCOL_CMD_FLAG)  # 0x21
            buf.append(0x01)                    # 命令类型
            buf.append(vx_byte & 0xFF)          # linear_x
            buf.append(vy_byte & 0xFF)          # linear_y 
            buf.append(omega_byte & 0xFF)       # angular_z
            buf.append(0x00)                    # 保留位
            buf.append(self.PROTOCOL_FOOTER)    # 0x21
            
            # 4. 发送数据
            if self.ser.is_open:
                self.ser.write(buf)
                
                # 调试输出
                rospy.loginfo(f"Sent: {[f'0x{b:02X}' for b in buf]}")
                rospy.loginfo(f"Converted: vx={vx:.2f}->{vx_byte}, "
                              f"vy={vy:.2f}->{vy_byte}, "
                              f"ω={omega:.2f}->{omega_byte}")
                
        except Exception as e:
            rospy.logerr(f"Velocity command error: {e}")


    def run(self):
        rospy.spin()
        
        # 清理资源
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = VelocitySerialNode()
    node.run()