#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
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

        # 订阅位置到达话题
        self.motor3_reached_sub = self.create_subscription(
            Bool, '/motor3/position_reached', self.motor3_reached_callback, 10)
        self.motor4_reached_sub = self.create_subscription(
            Bool, '/motor4/position_reached', self.motor4_reached_callback, 10)
        
        # 初始化状态
        self.motor3_reached = False
        self.motor4_reached = False
        self.motors_ready = False
        
        # 测试序列
        self.test_sequence = [
            0.0,    # 回到零位
            90.0,   # 转到90度
            0.0,    # 回到零位
            -90.0,  # 转到-90度
            0.0     # 回到零位
        ]
        
        self.current_step = 0

        # 等待服务可用
        self.wait_for_services()

        # 初始化电机
        self.initialize_motors()

    def wait_for_services(self):
        """等待所有服务可用"""
        self.get_logger().info('等待服务可用...')
        self.motor3_enable_client.wait_for_service()
        self.motor3_position_mode_client.wait_for_service()
        self.motor4_enable_client.wait_for_service()
        self.motor4_position_mode_client.wait_for_service()
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
            return

        # 设置电机4为位置模式
        future = self.motor4_position_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4设置为位置模式成功')
        else:
            self.get_logger().error(f'电机4设置位置模式失败: {future.result().message}')
            return

        # 使能电机3
        req = Trigger.Request()
        future = self.motor3_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机3使能成功')
        else:
            self.get_logger().error(f'电机3使能失败: {future.result().message}')
            return

        # 使能电机4
        future = self.motor4_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('电机4使能成功')
        else:
            self.get_logger().error(f'电机4使能失败: {future.result().message}')
            return

        self.get_logger().info('电机初始化完成')
        self.motors_ready = True
        
        # 开始测试
        self.get_logger().info('开始同步运动测试...')
        self.execute_next_step()

    def motor3_reached_callback(self, msg):
        """电机3到达位置回调"""
        if msg.data:
            self.get_logger().info('电机3到达目标位置')
            self.motor3_reached = True
            self.check_motors_reached()

    def motor4_reached_callback(self, msg):
        """电机4到达位置回调"""
        if msg.data:
            self.get_logger().info('电机4到达目标位置')
            self.motor4_reached = True
            self.check_motors_reached()

    def check_motors_reached(self):
        """检查两个电机是否都到达目标位置"""
        if self.motor3_reached and self.motor4_reached:
            self.get_logger().info('两个电机都已到达目标位置')
            
            # 重置状态
            self.motor3_reached = False
            self.motor4_reached = False
            
            # 执行下一步
            self.current_step += 1
            if self.current_step < len(self.test_sequence):
                self.get_logger().info(f'执行测试步骤 {self.current_step + 1}/{len(self.test_sequence)}')
                self.execute_next_step()
            else:
                self.get_logger().info('测试序列完成')

    def send_position(self, position):
        """同时向两个电机发送相同的位置命令"""
        msg = Float32()
        msg.data = position

        # 同时发布到两个电机话题
        self.motor3_pub.publish(msg)
        self.motor4_pub.publish(msg)

        self.get_logger().info(f'同时发送位置命令: {position}°')

    def execute_next_step(self):
        """执行测试序列的下一步"""
        if self.current_step < len(self.test_sequence):
            position = self.test_sequence[self.current_step]
            self.send_position(position)

def main(args=None):
    rclpy.init(args=args)
    tester = SyncMotionTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
