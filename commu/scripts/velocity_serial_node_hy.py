#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import threading
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf import transformations
from tf.broadcaster import TransformBroadcaster

class VelocitySerialNode:
    def __init__(self):
        rospy.init_node('velocity_serial_node')
        
        # 获取参数
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # 标志位状态
        self.flag_status = 0
        self.lock = threading.Lock()
        
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
        
        # 创建发布者
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        self.start_pub = rospy.Publisher('/wheels/start', Int32, queue_size=10, latch=True)
        
        # 订阅话题
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
        rospy.Subscriber('/wheels/arrive_box', Int32, self.arrive_box_callback)
        
        # 启动接收线程
        self.rx_thread = threading.Thread(target=self._rx_data_handler, daemon=True)
        self.rx_thread.start()
        rospy.loginfo("接收线程已启动")
        
        # 确保节点关闭时清理资源
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("速度指令串口节点准备就绪")

    def shutdown_hook(self):
        """节点关闭时的清理工作"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("串口已关闭")

    def arrive_box_callback(self, msg):
        """处理到达box消息"""
        if msg.data == 1:
            with self.lock:
                rospy.loginfo("收到/wheels/arrive_box消息，发送标志位为1的指令")
                self._send_command(0.0, 0.0, 0.0, 1)  # 发送停止指令+标志位
                rospy.sleep(0.1)
                self._send_command(0.0, 0.0, 0.0, 0)  # 立即恢复标志位

    def velocity_callback(self, msg):
        """处理速度指令"""
        with self.lock:
            # 速度归一化处理
            vx = max(min(msg.linear.x, 1.0), -1.0)
            vy = max(min(msg.linear.y, 1.0), -1.0)
            omega = -max(min(msg.angular.z, 1.0), -1.0)
            
            # 发送正常速度指令（标志位为0）
            self._send_command(vx, vy, omega, 0)

    def _send_command(self, vx, vy, omega, flag):
        """发送指令到串口"""
        try:
            # 转换为STM32协议格式 (0-255)
            vx_byte = int(128 - vx * 128)
            vy_byte = int(128 - vy * 128)
            omega_byte = int(128 - omega * 128)
            
            # 构建协议帧（9字节）
            cmd = bytearray([
                0x3F,   # 帧头
                0x21,   # 命令标志
                0x01,   # 命令类型
                vx_byte & 0xFF,
                vy_byte & 0xFF,
                omega_byte & 0xFF,
                0x00,   # 保留位
                flag & 0xFF,  # 标志位
                0x21    # 帧尾
            ])
            
            # 发送数据
            if self.ser.is_open:
                self.ser.write(cmd)
                rospy.logdebug(f"发送指令: vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f}, flag={flag}")
                
        except Exception as e:
            rospy.logerr(f"指令发送错误: {str(e)}")

    def _rx_data_handler(self):
        """独立线程处理接收数据"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    self._parse_odom_data(buffer)
            except serial.SerialException as e:
                rospy.logerr(f"串口接收错误: {e}")
                rospy.sleep(0.1)

    def _parse_odom_data(self, buffer):
        """解析STM32发送的里程计数据"""
        while len(buffer) >= 111:  # 完整帧长度
            # 查找帧头 "STM:"
            head_pos = buffer.find(b'STM:')
            if head_pos < 0:
                buffer.clear()
                return
                
            # 检查帧完整性
            if len(buffer) < head_pos + 111:
                return
                
            # 验证帧尾 ">ROS\r\n"
            if (buffer[head_pos+105] != ord('>') or
                buffer[head_pos+106] != ord('R') or
                buffer[head_pos+107] != ord('O') or
                buffer[head_pos+108] != ord('S') or
                buffer[head_pos+109] != ord('\r') or
                buffer[head_pos+110] != ord('\n')):
                del buffer[:head_pos+4]
                continue
                
            try:
                # 提取数据区 (4-104字节)
                data_block = buffer[head_pos+4:head_pos+104]
                
                # 解析结构体数据
                pos_x = struct.unpack('<f', data_block[0:4])[0]
                pos_y = struct.unpack('<f', data_block[4:8])[0]
                ang_rad = struct.unpack('<f', data_block[8:12])[0]
                v_angular = struct.unpack('<f', data_block[12:16])[0]
                
                # 获取标志位 (104字节)
                flag_status = buffer[head_pos+104]
                
                # 处理标志位
                if flag_status == 1:
                    rospy.loginfo("收到下位机启动标志位，发布/wheels/start消息")
                    self.start_pub.publish(Int32(1))
                
                # 创建里程计消息
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                
                odom.pose.pose.position.x = pos_x
                odom.pose.pose.position.y = pos_y
                odom.pose.pose.position.z = 0.0
                q = transformations.quaternion_from_euler(0, 0, ang_rad)
                odom.pose.pose.orientation = Quaternion(*q)
                
                odom.twist.twist.linear.x = 0.0  # 下位机未提供
                odom.twist.twist.linear.y = 0.0  # 下位机未提供
                odom.twist.twist.angular.z = v_angular
                
                # 发布里程计和TF
                self.odom_pub.publish(odom)
                self.tf_pub.sendTransform(
                    (pos_x, pos_y, 0.0),
                    q,
                    odom.header.stamp,
                    self.base_frame,
                    self.odom_frame
                )
                
                rospy.logdebug(f"里程计更新: x={pos_x:.2f}, y={pos_y:.2f}, θ={ang_rad:.2f} rad, ω={v_angular:.2f} rad/s")
                
            except Exception as e:
                rospy.logwarn(f"里程计解析错误: {e}")
            finally:
                # 移除已处理数据
                del buffer[:head_pos+111]

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VelocitySerialNode()
        node.run()
    except rospy.ROSInterruptException:
        pass