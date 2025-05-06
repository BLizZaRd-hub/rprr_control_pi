#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, SetBool
import time

class Motors34Tester(Node):
    def __init__(self):
        super().__init__('motors_34_tester')

        # 创建3号和4号电机的命令发布者
        self.motor3_pub = self.create_publisher(Float32, '/motor3_cmd_deg', 10)
        self.motor4_pub = self.create_publisher(Float32, '/motor4_cmd_deg', 10)

        # 创建服务客户端
        self.motor3_enable_client = self.create_client(Trigger, '/motor3/enable')
        self.motor3_position_mode_client = self.create_client(SetBool, '/motor3/position_mode')

        self.motor4_enable_client = self.create_client(Trigger, '/motor4/enable')
        self.motor4_position_mode_client = self.create_client(SetBool, '/motor4/position_mode')

        # 订阅位置到达话题
        self.motor3_reached_sub = self.create_subscription(
            Bool, '/motor3/position_reached', self.motor3_reached_callback, 10)
        self.motor4_reached_sub = self.create_subscription(
            Bool, '/motor4/position_reached', self.motor4_reached_callback, 10)

        # 订阅当前位置话题
        self.motor3_pos_sub = self.create_subscription(
            Float32, '/motor3_pos_deg', self.motor3_pos_callback, 10)
        self.motor4_pos_sub = self.create_subscription(
            Float32, '/motor4_pos_deg', self.motor4_pos_callback, 10)

        # 初始化状态
        self.motor3_reached = False
        self.motor4_reached = False
        self.motor3_current_pos = 0.0
        self.motor4_current_pos = 0.0
        self.test_stage = 0  # 0: 初始化, 1: 正转90°, 2: 反转90°, 3: 完成

        # 创建定时器，用于初始化
        self.create_timer(1.0, self.init_timer_callback)

    def init_timer_callback(self):
        # 只执行一次
        self.destroy_timer(self.init_timer_callback)

        # 等待服务可用
        self.wait_for_services()

        # 初始化电机
        self.initialize_motors()

    def wait_for_services(self):
        self.get_logger().info('等待服务可用...')

        # 等待电机3的服务
        self.motor3_enable_client.wait_for_service()
        self.motor3_position_mode_client.wait_for_service()

        # 等待电机4的服务
        self.motor4_enable_client.wait_for_service()
        self.motor4_position_mode_client.wait_for_service()

        self.get_logger().info('所有服务已可用')

    def initialize_motors(self):
        # 设置电机3为位置模式
        self.get_logger().info('设置电机3为位置模式...')
        req = SetBool.Request()
        req.data = True
        self.motor3_position_mode_client.call_async(req).add_done_callback(
            lambda future: self.position_mode_callback(future, 3))

        # 设置电机4为位置模式
        self.get_logger().info('设置电机4为位置模式...')
        req = SetBool.Request()
        req.data = True
        self.motor4_position_mode_client.call_async(req).add_done_callback(
            lambda future: self.position_mode_callback(future, 4))

    def position_mode_callback(self, future, motor_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'电机{motor_id}设置位置模式成功')

                # 使能电机
                self.get_logger().info(f'使能电机{motor_id}...')
                req = Trigger.Request()

                if motor_id == 3:
                    self.motor3_enable_client.call_async(req).add_done_callback(
                        lambda future: self.enable_callback(future, 3))
                else:
                    self.motor4_enable_client.call_async(req).add_done_callback(
                        lambda future: self.enable_callback(future, 4))
            else:
                self.get_logger().error(f'电机{motor_id}设置位置模式失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'电机{motor_id}设置位置模式调用异常: {str(e)}')

    def enable_callback(self, future, motor_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'电机{motor_id}使能成功')

                # 如果两个电机都已使能，开始测试
                if motor_id == 3:
                    self.motor3_ready = True
                else:
                    self.motor4_ready = True

                if hasattr(self, 'motor3_ready') and hasattr(self, 'motor4_ready'):
                    self.get_logger().info('两个电机都已准备就绪，开始测试序列')
                    self.start_test_sequence()
            else:
                self.get_logger().error(f'电机{motor_id}使能失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'电机{motor_id}使能调用异常: {str(e)}')

    def motor3_pos_callback(self, msg):
        self.motor3_current_pos = msg.data

    def motor4_pos_callback(self, msg):
        self.motor4_current_pos = msg.data

    def start_test_sequence(self):
        # 开始测试 - 先相对当前位置正转90°
        self.test_stage = 1
        self.get_logger().info(f'开始测试: 相对当前位置正转90°')
        self.get_logger().info(f'当前位置 - 电机3: {self.motor3_current_pos}°, 电机4: {self.motor4_current_pos}°')

        # 计算目标位置 (当前位置 + 90°)
        target_pos_3 = self.motor3_current_pos + 90.0
        target_pos_4 = self.motor4_current_pos + 90.0

        self.get_logger().info(f'目标位置 - 电机3: {target_pos_3}°, 电机4: {target_pos_4}°')
        self.send_position_command(target_pos_3, target_pos_4)

    def motor3_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('电机3到达目标位置')
            self.motor3_reached = True
            self.check_both_motors_reached()

    def motor4_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('电机4到达目标位置')
            self.motor4_reached = True
            self.check_both_motors_reached()

    def check_both_motors_reached(self):
        # 如果两个电机都到达目标位置，执行下一步
        if self.motor3_reached and self.motor4_reached:
            self.get_logger().info('两个电机都已到达目标位置')

            # 重置状态
            self.motor3_reached = False
            self.motor4_reached = False

            # 根据当前测试阶段执行下一步
            if self.test_stage == 1:
                # 完成正转90°，现在反转90°
                self.test_stage = 2
                self.get_logger().info(f'开始下一阶段: 相对当前位置反转90°')
                self.get_logger().info(f'当前位置 - 电机3: {self.motor3_current_pos}°, 电机4: {self.motor4_current_pos}°')

                # 计算目标位置 (当前位置 - 90°)
                target_pos_3 = self.motor3_current_pos - 90.0
                target_pos_4 = self.motor4_current_pos - 90.0

                self.get_logger().info(f'目标位置 - 电机3: {target_pos_3}°, 电机4: {target_pos_4}°')
                self.send_position_command(target_pos_3, target_pos_4)
            elif self.test_stage == 2:
                # 完成反转90°，测试结束
                self.test_stage = 3
                self.get_logger().info('测试序列完成')

    def send_position_command(self, position_3, position_4=None):
        # 如果只提供一个位置，两个电机使用相同的位置
        if position_4 is None:
            position_4 = position_3

        # 创建位置命令消息并发送到电机3
        msg3 = Float32()
        msg3.data = float(position_3)
        self.motor3_pub.publish(msg3)

        # 创建位置命令消息并发送到电机4
        msg4 = Float32()
        msg4.data = float(position_4)
        self.motor4_pub.publish(msg4)

        self.get_logger().info(f'发送位置命令 - 电机3: {position_3}°, 电机4: {position_4}°')


def main(args=None):
    rclpy.init(args=args)
    node = Motors34Tester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()