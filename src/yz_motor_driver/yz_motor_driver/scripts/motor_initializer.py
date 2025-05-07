#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, SetBool
import time

class MotorInitializer(Node):
    def __init__(self):
        super().__init__('motor_initializer')

        # 创建服务客户端 - 电机3
        self.motor3_enable_client = self.create_client(Trigger, '/motor3/enable')
        self.motor3_position_mode_client = self.create_client(SetBool, '/motor3/position_mode')

        # 创建服务客户端 - 电机4
        self.motor4_enable_client = self.create_client(Trigger, '/motor4/enable')
        self.motor4_position_mode_client = self.create_client(SetBool, '/motor4/position_mode')

        # 创建位置命令发布者（用于测试）
        self.motor3_pub = self.create_publisher(Float32, '/motor3_cmd_deg', 10)
        self.motor4_pub = self.create_publisher(Float32, '/motor4_cmd_deg', 10)

        # 订阅位置到达话题（用于测试）
        self.motor3_reached_sub = self.create_subscription(
            Bool, '/motor3/position_reached', self.motor3_reached_callback, 10)
        self.motor4_reached_sub = self.create_subscription(
            Bool, '/motor4/position_reached', self.motor4_reached_callback, 10)

        # 等待服务可用
        self.get_logger().info('等待服务可用...')
        self.wait_for_services()

        # 初始化电机
        self.get_logger().info('开始初始化电机...')
        self.initialize_motors()

        # 等待2秒，确保电机初始化完成
        self.get_logger().info('等待2秒，确保电机初始化完成...')
        time.sleep(2)

        # 发送测试位置命令
        self.get_logger().info('发送测试位置命令...')
        self.send_test_position_command()

    def motor3_reached_callback(self, msg):
        self.get_logger().info(f'收到电机3位置到达消息: {msg.data}')

    def motor4_reached_callback(self, msg):
        self.get_logger().info(f'收到电机4位置到达消息: {msg.data}')

    def wait_for_services(self):
        """等待所有服务可用"""
        self.get_logger().info('等待电机3 enable服务...')
        self.motor3_enable_client.wait_for_service()
        self.get_logger().info('等待电机3 position_mode服务...')
        self.motor3_position_mode_client.wait_for_service()
        self.get_logger().info('等待电机4 enable服务...')
        self.motor4_enable_client.wait_for_service()
        self.get_logger().info('等待电机4 position_mode服务...')
        self.motor4_position_mode_client.wait_for_service()
        self.get_logger().info('所有服务已可用')

    def initialize_motors(self):
        """初始化电机：设置位置模式并使能"""
        # 设置电机3为位置模式
        self.get_logger().info('设置电机3为位置模式...')
        req = SetBool.Request()
        req.data = True
        future = self.motor3_position_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机3设置位置模式成功')
        else:
            self.get_logger().error(f'电机3设置位置模式失败: {future.result().message}')

        # 设置电机4为位置模式
        self.get_logger().info('设置电机4为位置模式...')
        req = SetBool.Request()
        req.data = True
        future = self.motor4_position_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4设置位置模式成功')
        else:
            self.get_logger().error(f'电机4设置位置模式失败: {future.result().message}')

        # 使能电机3
        self.get_logger().info('使能电机3...')
        req = Trigger.Request()
        future = self.motor3_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机3使能成功')
        else:
            self.get_logger().error(f'电机3使能失败: {future.result().message}')

        # 使能电机4
        self.get_logger().info('使能电机4...')
        req = Trigger.Request()
        future = self.motor4_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4使能成功')
        else:
            self.get_logger().error(f'电机4使能失败: {future.result().message}')

        self.get_logger().info('电机初始化完成')

    def send_test_position_command(self):
        """发送测试位置命令"""
        # 创建位置命令消息并发送到电机3
        msg3 = Float32()
        msg3.data = 10000.0  # 移动10000度
        self.motor3_pub.publish(msg3)
        self.get_logger().info(f'发送位置命令 - 电机3: 10000.0°')

        # 创建位置命令消息并发送到电机4
        msg4 = Float32()
        msg4.data = 10000.0  # 移动10000度
        self.motor4_pub.publish(msg4)
        self.get_logger().info(f'发送位置命令 - 电机4: 10000.0°')

        # 多次发布命令，确保命令被接收
        self.get_logger().info('多次发布命令，确保命令被接收...')
        for i in range(5):
            time.sleep(1)
            self.motor3_pub.publish(msg3)
            self.motor4_pub.publish(msg4)
            self.get_logger().info(f'重复发送位置命令 - 电机3和电机4: 10000.0° (第{i+1}次)')

        # 等待10秒，确保命令被处理
        self.get_logger().info('等待10秒，确保命令被处理...')
        time.sleep(10)

        # 初始化完成后退出节点
        self.get_logger().info('初始化节点任务完成，准备退出...')
        # 等待一秒，确保日志被打印
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MotorInitializer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
