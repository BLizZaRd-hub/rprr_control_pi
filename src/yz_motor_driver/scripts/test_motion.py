#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time
import random

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # 只创建3号和4号电机的命令发布者
        self.motor3_pub = self.create_publisher(Float32, '/motor3_cmd_deg', 10)
        self.motor4_pub = self.create_publisher(Float32, '/motor4_cmd_deg', 10)
        
        # 创建定时器，每2秒发送一次命令
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.position = 90.0  # 起始位置
        
    def timer_callback(self):
        # 切换位置 90° <-> -90°
        self.position = -90.0 if self.position > 0 else 90.0
        
        # 添加小的随机相位增量 (±2°)
        phase3 = random.uniform(-2.0, 2.0)
        phase4 = random.uniform(-2.0, 2.0)
        
        # 只发布到3号和4号电机话题
        msg3 = Float32()
        msg3.data = self.position + phase3
        self.motor3_pub.publish(msg3)
        
        msg4 = Float32()
        msg4.data = self.position + phase4
        self.motor4_pub.publish(msg4)
        
        self.get_logger().info(f'发送位置命令: 电机3={self.position + phase3:.2f}°, 电机4={self.position + phase4:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    motor_tester = MotorTester()
    rclpy.spin(motor_tester)
    motor_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()