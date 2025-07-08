#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
        
        # 订阅速度指令话题
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
        
        rospy.loginfo("速度指令串口节点准备就绪")
    

    def velocity_callback(self, msg):
        try:
            # 调试：打印原始数据
            rospy.loginfo(f"准备发送数据: lineax={msg.linear.x}, lineax={msg.linear.y}, angular={msg.angular.z}")
            
            # 检查串口是否打开
            if not hasattr(self, 'ser') or not self.ser.is_open:
                rospy.logerr("串口未初始化或已关闭！")
                return

            # 打包数据（显式指定浮点数类型）
            buf = bytearray()
            buf.append(0xAA)  # 帧头
            
            # 打包线速度（确保是 float 类型）
            linearx_bytes = struct.pack('<f', float(msg.linear.x))
            buf.extend(linearx_bytes)

            # 打包线速度（确保是 float 类型）
            lineary_bytes = struct.pack('<f', float(msg.linear.y))
            buf.extend(lineary_bytes)

            # 打包角速度（确保是 float 类型）
            angular_bytes = struct.pack('<f', float(msg.angular.z))
            buf.extend(angular_bytes)
            
            buf.append(0xBB)  # 帧尾

            # 调试：打印字节内容
            rospy.loginfo(f"发送字节: {buf.hex()}")
            
            # 发送数据
            self.ser.write(buf)
            
        except struct.error as e:
            rospy.logerr(f"数据打包失败: {e}")
        except serial.SerialException as e:
            rospy.logerr(f"串口发送失败: {e}")
        except Exception as e:
            rospy.logerr(f"未知错误: {e}")


    def run(self):
        rospy.spin()
        
        # 清理资源
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = VelocitySerialNode()
    node.run()