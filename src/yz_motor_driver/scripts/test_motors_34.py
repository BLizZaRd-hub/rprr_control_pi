#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, UInt16
from std_srvs.srv import Trigger, SetBool
import time
import math
from enum import Enum, auto

# 预设与常量
NODE_IDS = [3, 4]  # 要控制的两个节点
STEP_DEG = 3600.0  # 每段旋转角度（正负交替）
PU_PER_REV = 32768.0  # 从电子齿轮读取（32768）
EPS_DEG = 0.2  # 误差阈值（度）
CONFIRM_N = 4  # 连续 N 次 TPDO 满足条件才算"到位"
TIMEOUT_FACTOR = 1.2  # 超时时间 = 预计时间 * TIMEOUT_FACTOR

# 状态机状态
class MotorState(Enum):
    IDLE = auto()
    MOVING = auto()

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

        # 订阅状态字话题
        self.motor3_status_sub = self.create_subscription(
            UInt16, '/motor3_status', self.motor3_status_callback, 10)
        self.motor4_status_sub = self.create_subscription(
            UInt16, '/motor4_status', self.motor4_status_callback, 10)

        # 初始化状态
        self.motor_states = {3: MotorState.IDLE, 4: MotorState.IDLE}
        self.motor_reached = {3: False, 4: False}
        self.motor_current_pos = {3: 0.0, 4: 0.0}
        self.motor_target_pos = {3: 0.0, 4: 0.0}
        self.motor_status = {3: 0, 4: 0}
        self.motor_confirm_count = {3: 0, 4: 0}
        self.direction = 1  # 初始正转
        self.cycle_count = 0  # 循环计数
        self.heartbeat_missing_count = {3: 0, 4: 0}
        self.max_heartbeat_missing = 3  # 最大允许丢失心跳次数

        # 创建心跳监控定时器
        self.heartbeat_timer = self.create_timer(0.1, self.check_heartbeat)

        # 创建定时器，等待一段时间后开始测试
        # 这给电机初始化节点一些时间来完成初始化
        self.get_logger().info('等待3秒后开始测试序列...')
        self.create_timer(3.0, self.start_test_sequence)

    def deg2pu(self, deg):
        """角度转换为编码器脉冲"""
        return deg / 360.0 * PU_PER_REV

    def check_heartbeat(self):
        """检查心跳状态"""
        for node_id in NODE_IDS:
            # 如果状态字为0，可能是心跳丢失
            if self.motor_status[node_id] == 0:
                self.heartbeat_missing_count[node_id] += 1
                if self.heartbeat_missing_count[node_id] >= self.max_heartbeat_missing:
                    self.get_logger().warn(f'电机{node_id}心跳丢失超过{self.max_heartbeat_missing}次')
            else:
                # 收到心跳，重置计数
                self.heartbeat_missing_count[node_id] = 0

    def motor3_status_callback(self, msg):
        """电机3状态字回调"""
        self.motor_status[3] = msg.data
        self.check_motor_status(3, msg.data)

    def motor4_status_callback(self, msg):
        """电机4状态字回调"""
        self.motor_status[4] = msg.data
        self.check_motor_status(4, msg.data)

    def check_motor_status(self, node_id, status):
        """检查电机状态字"""
        # 检查Fault位 (bit 3)
        if status & (1 << 3):
            self.get_logger().warn(f'电机{node_id}报告故障，状态字: 0x{status:04X}')

        # 检查目标到达位 (bit 10)
        target_reached = bool(status & (1 << 10))

        # 如果电机正在移动，检查是否到达目标位置
        if self.motor_states[node_id] == MotorState.MOVING:
            if target_reached and abs(self.motor_current_pos[node_id] - self.motor_target_pos[node_id]) <= EPS_DEG:
                self.motor_confirm_count[node_id] += 1
                if self.motor_confirm_count[node_id] >= CONFIRM_N:
                    self.get_logger().info(f'电机{node_id}到达目标位置 (连续{CONFIRM_N}次确认)')
                    self.motor_reached[node_id] = True
                    self.motor_states[node_id] = MotorState.IDLE
                    self.check_both_motors_reached()
            else:
                # 重置确认计数
                self.motor_confirm_count[node_id] = 0

    def motor3_pos_callback(self, msg):
        """电机3位置回调"""
        # 记录上一次位置，用于检测运动
        old_pos = self.motor_current_pos[3]
        self.motor_current_pos[3] = msg.data

        # 检测位置变化（仅在调试级别记录）
        if abs(self.motor_current_pos[3] - old_pos) > 0.1:
            self.get_logger().debug(f'电机3位置变化: {old_pos:.2f}° -> {self.motor_current_pos[3]:.2f}°')

    def motor4_pos_callback(self, msg):
        """电机4位置回调"""
        # 记录上一次位置，用于检测运动
        old_pos = self.motor_current_pos[4]
        self.motor_current_pos[4] = msg.data

        # 检测位置变化（仅在调试级别记录）
        if abs(self.motor_current_pos[4] - old_pos) > 0.1:
            self.get_logger().debug(f'电机4位置变化: {old_pos:.2f}° -> {self.motor_current_pos[4]:.2f}°')

    def start_test_sequence(self):
        """开始测试序列"""
        self.get_logger().info('开始±3600°往返循环测试')
        self.get_logger().info(f'当前位置 - 电机3: {self.motor_current_pos[3]:.2f}°, 电机4: {self.motor_current_pos[4]:.2f}°')

        # 开始第一次移动
        self.cycle_count = 1
        self.execute_next_movement()

    def motor3_reached_callback(self, msg):
        """电机3位置到达回调"""
        if msg.data:
            self.get_logger().info('收到电机3位置到达消息')
            # 注意：我们现在主要依靠状态字和位置检测来判断到位，这个回调作为辅助

    def motor4_reached_callback(self, msg):
        """电机4位置到达回调"""
        if msg.data:
            self.get_logger().info('收到电机4位置到达消息')
            # 注意：我们现在主要依靠状态字和位置检测来判断到位，这个回调作为辅助

    def check_both_motors_reached(self):
        """检查两个电机是否都到达目标位置"""
        if self.motor_reached[3] and self.motor_reached[4]:
            self.get_logger().info('两个电机都已到达目标位置')

            # 立即执行下一个动作（无延时）
            self.execute_next_movement()

    def execute_next_movement(self):
        """执行下一个动作"""
        # 计算相对位置增量
        delta_pos = self.direction * STEP_DEG

        # 记录当前循环信息
        direction_str = "正向" if self.direction > 0 else "反向"
        self.get_logger().info(f'===== 循环 {self.cycle_count} - {direction_str}旋转 {abs(STEP_DEG)}° =====')
        self.get_logger().info(f'当前位置 - 电机3: {self.motor_current_pos[3]:.2f}°, 电机4: {self.motor_current_pos[4]:.2f}°')

        # 计算目标位置（当前位置 + 方向*步长）- 仅用于日志记录
        for node_id in NODE_IDS:
            self.motor_target_pos[node_id] = self.motor_current_pos[node_id] + delta_pos

        self.get_logger().info(f'目标位置 - 电机3: {self.motor_target_pos[3]:.2f}°, 电机4: {self.motor_target_pos[4]:.2f}°')

        # 发送相对位置命令 - 直接发送相对增量
        self.send_position_command(
            self.motor_current_pos[3] + delta_pos,
            self.motor_current_pos[4] + delta_pos
        )

        # 反转方向，为下一次移动做准备
        self.direction = -self.direction

        # 如果方向变为正，增加循环计数
        if self.direction > 0:
            self.cycle_count += 1

    def send_position_command(self, position_3, position_4):
        """发送位置命令到两个电机"""
        # 重置到位状态和确认计数
        for node_id in NODE_IDS:
            self.motor_reached[node_id] = False
            self.motor_confirm_count[node_id] = 0
            self.motor_states[node_id] = MotorState.MOVING

        # 创建位置命令消息并发送到电机3
        msg3 = Float32()
        msg3.data = float(position_3)
        self.motor3_pub.publish(msg3)

        # 创建位置命令消息并发送到电机4
        msg4 = Float32()
        msg4.data = float(position_4)
        self.motor4_pub.publish(msg4)

        self.get_logger().info(f'发送位置命令 - 电机3: {position_3:.2f}°, 电机4: {position_4:.2f}°')

        # 不设置超时定时器，完全依靠状态字和位置检测来判断到位
        self.get_logger().info('等待电机到达目标位置...')


def main(args=None):
    # 确保 ROS 2 环境只初始化一次
    try:
        rclpy.init(args=args)

        # 创建节点
        node = Motors34Tester()

        # 打印启动消息
        node.get_logger().info('电机测试节点已启动，开始±3600°往返循环测试...')

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