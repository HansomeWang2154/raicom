#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import threading
from tf import transformations
from geometry_msgs.msg import Quaternion
import time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped  # 新增导入
import math  


class VelocitySerialNode:
    def __init__(self):
        rospy.init_node('velocity_serial_node')
        

        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 921600)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # 打开串口 (原代码保持不变)
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            rospy.loginfo(f"成功连接串口 {port}，波特率 {baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"无法打开串口 {port}: {e}")
            rospy.signal_shutdown("串口错误")
            return
        
        # 订阅速度指令话题 (原代码保持不变)
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)

        # 新增：里程计发布
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        
        # 新增：启动接收线程
        self.rx_thread = threading.Thread(target=self._rx_data_handler)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        
        rospy.loginfo("速度指令串口节点准备就绪 (已扩展接收功能)")

        # 新增AMCL订阅 (添加在初始化末尾)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, 
                                      self.amcl_callback)
        self.amcl_send_interval = rospy.Duration(0.1)  # 100ms发送间隔

        self.amcl_timeout = rospy.Duration(5.0)  # 设置一个合适的超时值，例如5秒
        self.amcl_ready = False
        self.last_amcl_send_time = rospy.Time.now()

        # 延迟初始化AMCL订阅
        self._init_amcl_subscriber()

    def _init_amcl_subscriber(self):
        """智能初始化AMCL订阅"""
        def check_amcl():
            try:
                # 等待AMCL话题出现
                rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 
                                    timeout=self.amcl_timeout.to_sec())
                self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                                            self.amcl_callback)
                self.amcl_ready = True
                rospy.loginfo("AMCL订阅初始化成功")
            except rospy.ROSException:
                rospy.logwarn(f"AMCL未在{self.amcl_timeout.to_sec()}秒内启动，将重试...")
                rospy.Timer(rospy.Duration(5.0), lambda _: self._init_amcl_subscriber(), oneshot=True)

        # 启动后台线程检查
        threading.Thread(target=check_amcl, daemon=True).start()     



    # 新增AMCL回调函数
    def amcl_callback(self, msg):
        """处理AMCL定位数据并发送给下位机"""
        current_time = rospy.Time.now()
        if current_time - self.last_amcl_send_time < self.amcl_send_interval:
            return  # 频率控制
        """增强版AMCL回调"""
        if not self.amcl_ready:
            return

        try:
            # 提取位置和姿态
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            _, _, yaw = transformations.euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            
            # 调试输出
            rospy.loginfo(f"[AMCL] 准备发送位置: x={x:.2f}m, y={y:.2f}m, yaw={math.degrees(yaw):.1f}°")
            
            # 打包数据 (小端格式)
            buf = bytearray()
            buf.append(0xCC)  # 新帧头
            buf.extend(struct.pack('<f', x))
            buf.extend(struct.pack('<f', y))
            buf.extend(struct.pack('<f', yaw))
            buf.append(0xDD)  # 新帧尾
            
            # 发送数据
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(buf)
                self.last_amcl_send_time = current_time
                
        except Exception as e:
            rospy.logerr(f"AMCL数据发送失败: {str(e)}")


    def velocity_callback(self, msg):
        try:
            # 速度缩放系数
            velocity_scale = 25.0
            omega_scale = 25.0
            # 调试：打印原始数据
            rospy.loginfo(f"原始数据: lineax={msg.linear.x}, lineay={msg.linear.y}, angular={msg.angular.z}")
            
            # 检查串口是否打开
            if not hasattr(self, 'ser') or not self.ser.is_open:
                rospy.logerr("串口未初始化或已关闭！")
                return

            # 缩放速度值
            scaled_linear_x = float(msg.linear.x) * velocity_scale
            scaled_linear_y = float(msg.linear.y) * velocity_scale
            scaled_angular_z = float(msg.angular.z) * (-omega_scale)
            
            # 调试：打印缩放后的数据
            rospy.loginfo(f"缩放后数据: lineax={scaled_linear_x}, lineay={scaled_linear_y}, angular={scaled_angular_z}")

            # 打包数据（显式指定浮点数类型）
            buf = bytearray()
            buf.append(0xAA)  # 帧头
            
            # 打包缩放后的线速度
            linearx_bytes = struct.pack('<f', scaled_linear_x)
            buf.extend(linearx_bytes)

            # 打包缩放后的线速度
            lineary_bytes = struct.pack('<f', scaled_linear_y)
            buf.extend(lineary_bytes)

            # 打包缩放后的角速度
            angular_bytes = struct.pack('<f', scaled_angular_z)
            buf.extend(angular_bytes)
            
            buf.append(0xBB)  # 帧尾

            # 调试：打印字节内容
            # rospy.loginfo(f"发送字节: {buf.hex()}")
            
            # 发送数据
            self.ser.write(buf)
        except struct.error as e:
            rospy.logerr(f"数据打包失败: {e}")
        except serial.SerialException as e:
            rospy.logerr(f"串口发送失败: {e}")
        except Exception as e:
            rospy.logerr(f"未知错误: {e}")


    # 原发送回调函数完全保持不变
    # def velocity_callback(self, msg):
    #     try:
    #         # 调试：打印原始数据
    #         rospy.loginfo(f"准备发送数据: lineax={msg.linear.x}, lineax={msg.linear.y}, angular={msg.angular.z}")
            
    #         # 检查串口是否打开
    #         if not hasattr(self, 'ser') or not self.ser.is_open:
    #             rospy.logerr("串口未初始化或已关闭！")
    #             return

    #         # 打包数据（显式指定浮点数类型）
    #         buf = bytearray()
    #         buf.append(0xAA)  # 帧头
            
    #         # 打包线速度（确保是 float 类型）
    #         linearx_bytes = struct.pack('<f', float(msg.linear.x))
    #         buf.extend(linearx_bytes)

    #         # 打包线速度（确保是 float 类型）
    #         lineary_bytes = struct.pack('<f', float(msg.linear.y))
    #         buf.extend(lineary_bytes)

    #         # 打包角速度（确保是 float 类型）
    #         angular_bytes = struct.pack('<f', float(msg.angular.z))
    #         buf.extend(angular_bytes)
            
    #         buf.append(0xBB)  # 帧尾

    #         # 调试：打印字节内容
    #         # rospy.loginfo(f"发送字节: {buf.hex()}")
            
    #         # 发送数据
    #         self.ser.write(buf)
            
    #     except struct.error as e:
    #         rospy.logerr(f"数据打包失败: {e}")
    #     except serial.SerialException as e:
    #         rospy.logerr(f"串口发送失败: {e}")
    #     except Exception as e:
    #         rospy.logerr(f"未知错误: {e}")

    # 新增：接收数据处理函数
    def _rx_data_handler(self):
        """独立线程处理接收数据"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                # 非阻塞读取
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    self._parse_odom_data(buffer)
            except serial.SerialException as e:
                rospy.logerr(f"串口接收错误: {e}")
                rospy.sleep(0.1)

    # 新增：里程计数据解析
    def _parse_odom_data(self, buffer):
        """
        解析STM32发送的里程计数据 (26字节协议)
        协议格式:
        [0]: 0xAA (帧头)
        [1-4]: pos_x (float)
        [5-8]: pos_y (float)
        [9-12]: ang_rad (float)
        [13-16]: v_linear_x (float)
        [17-20]: v_linear_y (float)
        [21-24]: v_angular (float)
        [25]: '\n' (帧尾)
        """
        while len(buffer) >= 26:  # 检查是否够一帧
            # 查找帧头
            head_pos = buffer.find(0xAA)
            if head_pos < 0:
                buffer.clear()
                return
                
            # 检查帧完整性
            if len(buffer) < head_pos + 26:
                return
                
            # 验证帧尾
            if buffer[head_pos+25] != ord('\n'):
                buffer = buffer[head_pos+1:]
                continue
                
            # 提取完整帧
            frame = buffer[head_pos:head_pos+26]
            # rospy.loginfo(f"原始帧数据: {frame.hex()}")  # 打印16进制数据
            try:
                # 解析数据
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                
                # 位置解析
                odom.pose.pose.position.x = struct.unpack('<f', frame[1:5])[0]
                odom.pose.pose.position.y = struct.unpack('<f', frame[5:9])[0]
                
                # 角度解析 (转四元数)
                yaw = struct.unpack('<f', frame[9:13])[0]
                q = transformations.quaternion_from_euler(0, 0, yaw)
                odom.pose.pose.orientation = Quaternion(*q)
                
                # 速度解析
                odom.twist.twist.linear.x = struct.unpack('<f', frame[13:17])[0]
                odom.twist.twist.linear.y = struct.unpack('<f', frame[17:21])[0]
                odom.twist.twist.angular.z = struct.unpack('<f', frame[21:25])[0]


                # rospy.loginfo(f"发布前的完整数据:\n{odom}")  # 打印整个Odometry消息
                # time.sleep(1)
                ##这部分不加延时tf的处理就会出问题，tf更新失败？？
                # 发布里程计
                odom.header.stamp = rospy.Time.now()  # 必须放在所有数据赋值之后！
                self.odom_pub.publish(odom)
                
                # 发布TF
                self.tf_pub.sendTransform(
                    (odom.pose.pose.position.x, odom.pose.pose.position.y, 0),
                    (q[0], q[1], q[2], q[3]),
                    odom.header.stamp,
                    self.base_frame,
                    self.odom_frame
                )
                
                # rospy.loginfo(f"里程计更新: x={odom.pose.pose.position.x:.2f}, "
                #               f"y={odom.pose.pose.position.y:.2f}, "
                #               f"θ={yaw:.2f} rad")
                
            except Exception as e:
                rospy.logwarn(f"里程计解析错误: {e}")
            finally:
                del buffer[:head_pos+26]  # 关键！移除已处理数据（包括当前帧）
                buffer.clear()  # 彻底清空缓冲区（激进策略，根据需求选择）

    def run(self):
        rospy.spin()
        
        # 清理资源
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    from tf.transformations import quaternion_from_euler
    from tf.broadcaster import TransformBroadcaster
    node = VelocitySerialNode()
    node.run()