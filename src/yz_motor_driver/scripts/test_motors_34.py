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
        self.test_stage = 0  # 0: 初始化, 1: 正转90°, 2: 反转90°

        # 创建定时器，等待一段时间后开始测试
        # 这给电机初始化节点一些时间来完成初始化
        self.get_logger().info('等待3秒后开始测试序列...')
        self.create_timer(3.0, self.start_test_sequence)

    def motor3_pos_callback(self, msg):
        # 记录上一次位置，用于检测运动
        old_pos = self.motor3_current_pos
        self.motor3_current_pos = msg.data

        # 检测位置变化
        if abs(self.motor3_current_pos - old_pos) > 0.1:
            self.get_logger().info(f'电机3位置变化: {old_pos:.2f}° -> {self.motor3_current_pos:.2f}°')

    def motor4_pos_callback(self, msg):
        # 记录上一次位置，用于检测运动
        old_pos = self.motor4_current_pos
        self.motor4_current_pos = msg.data

        # 检测位置变化
        if abs(self.motor4_current_pos - old_pos) > 0.1:
            self.get_logger().info(f'电机4位置变化: {old_pos:.2f}° -> {self.motor4_current_pos:.2f}°')

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
        self.get_logger().info(f'收到电机3位置到达消息: {msg.data}')
        if msg.data:
            self.get_logger().info('电机3到达目标位置')
            self.motor3_reached = True
            self.check_both_motors_reached()

    def motor4_reached_callback(self, msg):
        self.get_logger().info(f'收到电机4位置到达消息: {msg.data}')
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

            # 取消超时定时器（如果存在）
            if hasattr(self, 'timeout_timer'):
                self.destroy_timer(self.timeout_timer)
                delattr(self, 'timeout_timer')

            # 等待一段时间，确保电机状态稳定
            self.create_timer(2.0, self.next_movement)

    def next_movement(self):
        # 只执行一次
        self.destroy_timer(self.next_movement)

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
            # 完成反转90°，回到正转阶段，循环执行
            self.test_stage = 1
            self.get_logger().info(f'开始下一循环: 相对当前位置正转90°')
            self.get_logger().info(f'当前位置 - 电机3: {self.motor3_current_pos}°, 电机4: {self.motor4_current_pos}°')

            # 计算目标位置 (当前位置 + 90°)
            target_pos_3 = self.motor3_current_pos + 90.0
            target_pos_4 = self.motor4_current_pos + 90.0

            self.get_logger().info(f'目标位置 - 电机3: {target_pos_3}°, 电机4: {target_pos_4}°')
            self.send_position_command(target_pos_3, target_pos_4)

    def send_position_command(self, position_3, position_4=None):
        # 如果只提供一个位置，两个电机使用相同的位置
        if position_4 is None:
            position_4 = position_3

        # 重置到位状态
        self.motor3_reached = False
        self.motor4_reached = False

        # 创建位置命令消息并发送到电机3
        msg3 = Float32()
        msg3.data = float(position_3)
        self.motor3_pub.publish(msg3)

        # 创建位置命令消息并发送到电机4
        msg4 = Float32()
        msg4.data = float(position_4)
        self.motor4_pub.publish(msg4)

        self.get_logger().info(f'发送位置命令 - 电机3: {position_3}°, 电机4: {position_4}°')

        # 设置超时定时器，如果10秒内没有收到位置到达消息，则继续执行
        self.timeout_timer = self.create_timer(10.0, self.movement_timeout_callback)

    def movement_timeout_callback(self):
        # 只执行一次
        self.destroy_timer(self.timeout_timer)

        # 检查是否已经收到位置到达消息
        if not self.motor3_reached or not self.motor4_reached:
            self.get_logger().warn('位置到达超时！强制继续执行...')

            # 记录当前状态
            self.get_logger().info(f'电机3到达状态: {self.motor3_reached}, 电机4到达状态: {self.motor4_reached}')

            # 强制设置到位状态
            self.motor3_reached = True
            self.motor4_reached = True

            # 继续执行下一步
            self.next_movement()


def main(args=None):
    # 确保 ROS 2 环境只初始化一次
    try:
        rclpy.init(args=args)

        # 创建节点
        node = Motors34Tester()

        # 打印启动消息
        node.get_logger().info('电机测试节点已启动，开始测试循环...')

        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，停止测试...')
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f'发生错误: {str(e)}')
        else:
            print(f'初始化错误: {str(e)}')
    finally:
        # 清理资源
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()