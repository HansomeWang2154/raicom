#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import termios, sys, tty

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 速度参数配置
        self.linear_speed = 0.1    # 默认线速度(m/s)
        self.angular_speed = 0.1   # 默认角速度(rad/s)
        self.speed_step = 0.02      # 速度调节步长
        
        # 当前速度指令
        self.twist = Twist()
        self.update_interval = 0.1  # 控制指令发送间隔(秒)
        
        print("键盘控制说明:")
        print("  W/S: 前进/后退")
        print("  A/D: 左移/右移")
        print("  Q/E: 左转/右转")
        print("  +/-: 增加/减小速度")
        print("  空格: 紧急停止")
        print("  Ctrl+C: 退出")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # 重置速度指令
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.angular.z = 0.0
                
                # 处理按键
                if key == 'w':
                    self.twist.linear.x = self.linear_speed  # 前进
                elif key == 's':
                    self.twist.linear.x = -self.linear_speed # 后退
                elif key == 'a':
                    self.twist.linear.y = self.linear_speed  # 左移
                elif key == 'd':
                    self.twist.linear.y = -self.linear_speed # 右移
                elif key == 'q':
                    self.twist.angular.z = self.angular_speed # 左转
                elif key == 'e':
                    self.twist.angular.z = -self.angular_speed # 右转
                elif key == '+':
                    # 增加速度
                    self.linear_speed = min(2.0, self.linear_speed + self.speed_step)
                    self.angular_speed = min(3.0, self.angular_speed + self.speed_step)
                    print(f"速度增加: 线速度={self.linear_speed:.2f}m/s, 角速度={self.angular_speed:.2f}rad/s")
                elif key == '-':
                    # 减小速度 (最小0.1)
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_step)
                    self.angular_speed = max(0.1, self.angular_speed - self.speed_step)
                    print(f"速度减小: 线速度={self.linear_speed:.2f}m/s, 角速度={self.angular_speed:.2f}rad/s")
                elif key == ' ':
                    # 紧急停止
                    self.twist = Twist()
                    print("紧急停止!")
                elif key == '\x03':  # Ctrl+C
                    break
                
                # 发布指令
                self.pub.publish(self.twist)
                
        except Exception as e:
            print(f"错误: {e}")
        finally:
            # 退出时发送零速度
            self.pub.publish(Twist())

if __name__ == '__main__':
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass