#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import serial
import struct
import threading
from tf import transformations
from tf.broadcaster import TransformBroadcaster

class VelocitySerialNode:
    def __init__(self):
        rospy.init_node('velocity_serial_node')
        
        # 获取参数
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)  # 必须与STM32一致
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # 打开串口
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.5
            )
            rospy.loginfo(f"成功连接串口 {port}，波特率 {baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"无法打开串口 {port}: {e}")
            rospy.signal_shutdown("串口错误")
            return
        
        # 订阅速度指令话题
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
        
        # 里程计发布
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        
        # 启动接收线程
        self.rx_thread = threading.Thread(target=self._rx_data_handler)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        rospy.loginfo("接收线程已启动")
        rospy.loginfo("速度指令串口节点准备就绪")

    def velocity_callback(self, msg):
        """匹配STM32协议的速度指令发送"""
        try:
            # 速度归一化处理（-1.0~1.0）
            # vx = vx/2
            # vy = vy/2
            # omega = omega/2
            vx = max(min(msg.linear.x, 1.0), -1.0)
            vy = max(min(msg.linear.y, 1.0), -1.0)
            omega = -max(min(msg.angular.z, 1.0), -1.0)
            
            # 转换为STM32协议格式 (0-255)
            vx_byte = int(128 - vx * 128)
            vy_byte = int(128 - vy * 128)
            omega_byte = int(128 - omega * 128)
            
            # 构建协议帧（8字节）
            cmd = bytearray([
                0x3F,   # 帧头
                0x21,   # 命令标志
                0x01,   # 命令类型
                vx_byte & 0xFF,
                vy_byte & 0xFF,
                omega_byte & 0xFF,
                0x00,   # 保留位
                0x21    # 帧尾
            ])
            
            # 发送数据
            if self.ser.is_open:
                self.ser.write(cmd)
                
                # 调试输出
                rospy.logdebug(f"发送速度指令: vx={vx:.2f}->{vx_byte}, "
                              f"vy={vy:.2f}->{vy_byte}, "
                              f"ω={omega:.2f}->{omega_byte}")
                rospy.logdebug(f"原始数据: {[f'0x{b:02X}' for b in cmd]}")
                
        except Exception as e:
            rospy.logerr(f"速度指令发送错误: {str(e)}")

    def _rx_data_handler(self):
        """独立线程处理接收数据"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                # 非阻塞读取
                # data = self.ser.read(self.ser.in_waiting or 1)
                data = self.ser.read(self.ser.in_waiting) 
                if data:
                    # rospy.loginfo(f"收到 {len(data)} 字节数据")
                    buffer.extend(data)
                    # rospy.loginfo(f"当前缓冲区长度: {len(buffer)}")
                    self._parse_odom_data(buffer)
            except serial.SerialException as e:
                rospy.logerr(f"串口接收错误: {e}")
                rospy.sleep(0.1)



    def _parse_odom_data(self, buffer):
        """
        解析STM32发送的里程计数据 (110字节协议)
        协议格式:
        [0-3]: 'S','T','M',':' (4字节)
        [4-103]: 结构体数据 (100字节)
        [104-109]: '>','R','O','S','\r','\n' (6字节)
        """
        # rospy.loginfo(f"串口接收数据: {len(buffer)} 字节")
        while len(buffer) >= 110:  # 检查是否够一帧
            # 查找帧头 "STM:"
            head_pos = buffer.find(b'STM:')
            if head_pos < 0:
                # 没有找到有效帧头，清除缓冲区
                buffer.clear()
                return
                
            # 检查帧完整性
            if len(buffer) < head_pos + 110:
                return  # 等待更多数据
                
            # 验证帧尾 ">ROS\r\n"
            if (buffer[head_pos+104] != ord('>') or
                buffer[head_pos+105] != ord('R') or
                buffer[head_pos+106] != ord('O') or
                buffer[head_pos+107] != ord('S') or
                buffer[head_pos+108] != ord('\r') or
                buffer[head_pos+109] != ord('\n')):
                # 帧尾不匹配，跳过此帧
                del buffer[:head_pos+4]  # 移除已检查部分
                continue
                
            try:
                # 提取数据区 (4-103字节)
                data_block = buffer[head_pos+4:head_pos+104]
                
                # 解析结构体 (假设前16字节包含4个float)
                # 位置x (0-3字节)
                pos_x = struct.unpack('<f', data_block[0:4])[0]
                # 位置y (4-7字节)
                pos_y = struct.unpack('<f', data_block[4:8])[0]
                # 角度 (8-11字节)
                ang_rad = struct.unpack('<f', data_block[8:12])[0]
                # 角速度 (12-15字节)
                v_angular = struct.unpack('<f', data_block[12:16])[0]
                
                # 创建里程计消息
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                
                # 设置位置
                odom.pose.pose.position.x = pos_x
                odom.pose.pose.position.y = pos_y
                odom.pose.pose.position.z = 0.0
                
                # 设置方向 (从偏航角转换)
                q = transformations.quaternion_from_euler(0, 0, ang_rad)
                odom.pose.pose.orientation = Quaternion(*q)
                
                # 设置速度
                odom.twist.twist.linear.x = 0.0  # 下位机未提供
                odom.twist.twist.linear.y = 0.0  # 下位机未提供
                odom.twist.twist.angular.z = v_angular
                
                # 发布里程计
                self.odom_pub.publish(odom)
                
                # 发布TF
                self.tf_pub.sendTransform(
                    (pos_x, pos_y, 0.0),
                    q,
                    odom.header.stamp,
                    self.base_frame,
                    self.odom_frame
                )
                
                rospy.logdebug(f"里程计更新: x={pos_x:.2f}, y={pos_y:.2f}, "
                              f"θ={ang_rad:.2f} rad, ω={v_angular:.2f} rad/s")
                
            except Exception as e:
                rospy.logwarn(f"里程计解析错误: {e}")
            finally:
                # 移除已处理数据
                del buffer[:head_pos+110]
                buffer.clear()  # 彻底清空缓冲区（激进策略，根据需求选择）


    def run(self):
        rospy.spin()
        
        # 清理资源
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = VelocitySerialNode()
    node.run()