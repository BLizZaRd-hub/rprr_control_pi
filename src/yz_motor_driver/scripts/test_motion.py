#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time
import random

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # 创建单独的电机命令发布者
        self.motor1_pub = self.create_publisher(Float32, '/motor1_cmd_deg', 10)
        self.motor2_pub = self.create_publisher(Float32, '/motor2_cmd_deg', 10)
        self.motor3_pub = self.create_publisher(Float32, '/motor3_cmd_deg', 10)
        self.motor4_pub = self.create_publisher(Float32, '/motor4_cmd_deg', 10)
        
        # 创建组命令发布者
        self.group_pub = self.create_publisher(Float32MultiArray, '/joint_group_cmd', 10)
        
        # 创建定时器，每2秒发送一次命令
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.position = 90.0  # 起始位置
        
    def timer_callback(self):
        # 切换位置 90° <-> -90°
        self.position = -90.0 if self.position > 0 else 90.0
        
        # 添加小的随机相位增量 (±2°)
        phase_increments = [random.uniform(-2.0, 2.0) for _ in range(4)]
        
        # 发布到单独的电机话题
        msg1 = Float32()
        msg1.data = self.position + phase_increments[0]
        self.motor1_pub.publish(msg1)
        
        msg2 = Float32()
        msg2.data = self.position + phase_increments[1]
        self.motor2_pub.publish(msg2)
        
        msg3 = Float32()
        msg3.data = self.position + phase_increments[2]
        self.motor3_pub.publish(msg3)
        
        msg4 = Float32()
        msg4.data = self.position + phase_increments[3]
        self.motor4_pub.publish(msg4)
        
        # 同时发布到组话题
        group_msg = Float32MultiArray()
        group_msg.data = [self.position + inc for inc in phase_increments]
        self.group_pub.publish(group_msg)
        
        self.get_logger().info(f'发送位置命令: {[self.position + inc for inc in phase_increments]}')

def main(args=None):
    rclpy.init(args=args)
    motor_tester = MotorTester()
    rclpy.spin(motor_tester)
    motor_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
