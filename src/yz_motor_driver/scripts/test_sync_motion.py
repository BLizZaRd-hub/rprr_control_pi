#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, SetBool
import time

class SyncMotionTester(Node):
    def __init__(self):
        super().__init__('sync_motion_tester')

        # 创建3号和4号电机的命令发布者
        self.motor3_pub = self.create_publisher(Float32, '/motor3_cmd_deg', 10)
        self.motor4_pub = self.create_publisher(Float32, '/motor4_cmd_deg', 10)

        # 创建服务客户端 - 电机3
        self.motor3_enable_client = self.create_client(Trigger, '/motor3/enable')
        self.motor3_position_mode_client = self.create_client(SetBool, '/motor3/position_mode')

        # 创建服务客户端 - 电机4
        self.motor4_enable_client = self.create_client(Trigger, '/motor4/enable')
        self.motor4_position_mode_client = self.create_client(SetBool, '/motor4/position_mode')

        # 等待服务可用
        self.wait_for_services()

        # 初始化电机
        self.initialize_motors()

        # 开始测试
        self.get_logger().info('开始同步运动测试...')
        self.run_test()

    def wait_for_services(self):
        """等待所有服务可用"""
        self.get_logger().info('等待服务可用...')

        services = [
            (self.motor3_enable_client, '/motor3/enable'),
            (self.motor3_position_mode_client, '/motor3/position_mode'),
            (self.motor4_enable_client, '/motor4/enable'),
            (self.motor4_position_mode_client, '/motor4/position_mode')
        ]

        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待服务 {service_name} 可用...')

        self.get_logger().info('所有服务已可用')

    def initialize_motors(self):
        """初始化电机：设置位置模式并使能"""
        self.get_logger().info('初始化电机...')

        # 设置电机3为位置模式
        req = SetBool.Request()
        req.data = True
        future = self.motor3_position_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机3设置为位置模式成功')
        else:
            self.get_logger().error(f'电机3设置位置模式失败: {future.result().message}')

        # 设置电机4为位置模式
        future = self.motor4_position_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4设置为位置模式成功')
        else:
            self.get_logger().error(f'电机4设置位置模式失败: {future.result().message}')

        # 使能电机3
        req = Trigger.Request()
        future = self.motor3_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机3使能成功')
        else:
            self.get_logger().error(f'电机3使能失败: {future.result().message}')

        # 使能电机4
        future = self.motor4_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4使能成功')
        else:
            self.get_logger().error(f'电机4使能失败: {future.result().message}')

        self.get_logger().info('电机初始化完成')

        # 等待电机稳定
        time.sleep(1.0)

    def send_position(self, position):
        """同时向两个电机发送相同的位置命令"""
        msg = Float32()
        msg.data = position

        # 同时发布到两个电机话题
        self.motor3_pub.publish(msg)
        self.motor4_pub.publish(msg)

        self.get_logger().info(f'同时发送位置命令: {position}°')

    def run_test(self):
        """运行测试序列"""
        positions = [0.0, 90.0, 0.0, -90.0, 0.0]

        for pos in positions:
            self.send_position(pos)
            time.sleep(2.0)  # 等待2秒让电机到达位置

        # 测试完成
        self.get_logger().info('测试完成')

def main(args=None):
    rclpy.init(args=args)
    tester = SyncMotionTester()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
