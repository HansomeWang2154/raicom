#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度指令串口发送节点 完善"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import serial
import struct
import threading

class UnifiedSerialNode:
    def __init__(self):
        rospy.init_node('unified_serial_node')
        
        # 串口配置
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)
        
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.1
            )
            rospy.loginfo(f"串口已连接: {port} @ {baudrate}bps")
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {str(e)}")
            rospy.signal_shutdown("硬件错误")
            return
        
        # 协议参数
        self.HEADER = 0xAA
        self.FOOTER = 0xBB
        self.CMD_VELOCITY = 0x01
        self.CMD_AMCL = 0x02
        
        # 订阅话题
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # 线程锁
        self.serial_lock = threading.Lock()
        rospy.loginfo("统一串口节点准备就绪")

    def _send_packet(self, cmd_type, data_bytes):
        """通用数据包发送函数"""
        # 构建数据包
        buf = bytearray()
        buf.append(self.HEADER)              # 帧头
        buf.append(cmd_type)                # 命令类型
        buf.append(len(data_bytes))         # 数据长度
        
        # 添加数据区
        buf.extend(data_bytes)
        
        # 计算校验和 (从帧头到数据区结束)
        checksum = sum(buf) & 0xFF
        buf.append(checksum)
        
        # 添加帧尾
        buf.append(self.FOOTER)
        
        # 发送数据
        with self.serial_lock:
            try:
                self.ser.write(buf)
                rospy.logdebug(f"发送数据包: {[f'0x{b:02X}' for b in buf]}")
            except Exception as e:
                rospy.logwarn(f"串口发送失败: {str(e)}")

    def velocity_callback(self, msg):
        """速度指令处理"""
        # 打包速度数据
        data = struct.pack('<fff', 
                          msg.linear.x,
                          msg.linear.y,
                          msg.angular.z)
        
        # 发送
        self._send_packet(self.CMD_VELOCITY, data)

    def amcl_callback(self, msg):
        """AMCL位置处理"""
        # 提取偏航角
        _, _, yaw = transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # 打包位置数据
        data = struct.pack('<fff',
                          msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          yaw)
        
        # 发送
        self._send_packet(self.CMD_AMCL, data)

    def run(self):
        rospy.spin()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    try:
        from tf import transformations
        node = UnifiedSerialNode()
        node.run()
    except ImportError:
        rospy.logerr("缺少tf模块，请安装: sudo apt install ros-<distro>-tf")
    except rospy.ROSInterruptException:
        pass