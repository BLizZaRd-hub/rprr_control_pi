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
        
        # 初始化状态
        self.motor3_reached = False
        self.motor4_reached = False
        
        # 测试序列 - 可以根据需要修改
        self.test_positions = [0.0, 45.0, 90.0, 0.0, -45.0, -90.0, 0.0]
        self.current_position_index = 0
        
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
    
    def start_test_sequence(self):
        # 开始第一个位置命令
        self.send_position_command(self.test_positions[0])
    
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
            
            # 移动到下一个位置
            self.current_position_index += 1
            if self.current_position_index < len(self.test_positions):
                next_position = self.test_positions[self.current_position_index]
                self.get_logger().info(f'移动到下一个位置: {next_position}°')
                self.send_position_command(next_position)
            else:
                self.get_logger().info('测试序列完成')
    
    def send_position_command(self, position):
        # 创建位置命令消息
        msg = Float32()
        msg.data = float(position)
        
        # 发送到两个电机
       