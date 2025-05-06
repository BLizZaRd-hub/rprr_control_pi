#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, SetBool
import time

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        
        # 创建位置命令发布者
        self.motor1_pub = self.create_publisher(Float32, '/motor1_cmd_deg', 10)
        self.motor2_pub = self.create_publisher(Float32, '/motor2_cmd_deg', 10)
        
        # 创建服务客户端
        self.motor1_enable_client = self.create_client(Trigger, '/motor1/enable')
        self.motor1_position_mode_client = self.create_client(SetBool, '/motor1/position_mode')
        
        self.motor2_enable_client = self.create_client(Trigger, '/motor2/enable')
        self.motor2_position_mode_client = self.create_client(SetBool, '/motor2/position_mode')
        
        # 订阅位置到达话题
        self.motor1_reached_sub = self.create_subscription(
            Bool, '/motor1/position_reached', self.motor1_reached_callback, 10)
        self.motor2_reached_sub = self.create_subscription(
            Bool, '/motor2/position_reached', self.motor2_reached_callback, 10)
        
        # 初始化状态
        self.motor1_ready = False
        self.motor2_ready = False
        
        # 测试序列
        self.test_sequence = [
            (90.0, -90.0),   # 电机1正转90度，电机2反转90度
            (0.0, 0.0),      # 两个电机都回到0度
            (180.0, 180.0),  # 两个电机都转到180度
            (0.0, 0.0)       # 两个电机都回到0度
        ]
        
        self.current_step = 0
        
        # 等待服务可用
        self.wait_for_services()
        
        # 初始化电机
        self.initialize_motors()
    
    def wait_for_services(self):
        self.get_logger().info('等待服务可用...')
        self.motor1_enable_client.wait_for_service()
        self.motor1_position_mode_client.wait_for_service()
        self.motor2_enable_client.wait_for_service()
        self.motor2_position_mode_client.wait_for_service()
        self.get_logger().info('所有服务已可用')
    
    def initialize_motors(self):
        # 使能电机
        self.get_logger().info('使能电机1...')
        req = Trigger.Request()
        self.motor1_enable_client.call_async(req).add_done_callback(
            lambda future: self.enable_callback(future, 1))
        
        self.get_logger().info('使能电机2...')
        req = Trigger.Request()
        self.motor2_enable_client.call_async(req).add_done_callback(
            lambda future: self.enable_callback(future, 2))
    
    def enable_callback(self, future, motor_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'电机{motor_id}使能成功')
                
                # 设置位置模式
                req = SetBool.Request()
                req.data = True
                
                if motor_id == 1:
                    self.motor1_position_mode_client.call_async(req).add_done_callback(
                        lambda future: self.position_mode_callback(future, 1))
                else:
                    self.motor2_position_mode_client.call_async(req).add_done_callback(
                        lambda future: self.position_mode_callback(future, 2))
            else:
                self.get_logger().error(f'电机{motor_id}使能失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'电机{motor_id}使能调用异常: {str(e)}')
    
    def position_mode_callback(self, future, motor_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'电机{motor_id}设置位置模式成功')
                
                # 标记电机准备就绪
                if motor_id == 1:
                    self.motor1_ready = True
                else:
                    self.motor2_ready = True
                
                # 如果两个电机都准备好了，开始测试
                if self.motor1_ready and self.motor2_ready:
                    self.get_logger().info('两个电机都已准备就绪，开始测试序列')
                    self.execute_next_step()
            else:
                self.get_logger().error(f'电机{motor_id}设置位置模式失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'电机{motor_id}设置位置模式调用异常: {str(e)}')
    
    def motor1_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('电机1到达目标位置')
            self.check_both_motors_reached()
    
    def motor2_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('电机2到达目标位置')
            self.check_both_motors_reached()
    
    def check_both_motors_reached(self):
        # 如果两个电机都到达目标位置，执行下一步
        if self.motor1_ready and self.motor2_ready:
            self.current_step += 1
            if self.current_step < len(self.test_sequence):
                self.get_logger().info(f'执行测试步骤 {self.current_step + 1}/{len(self.test_sequence)}')
                self.execute_next_step()
            else:
                self.get_logger().info('测试序列完成')
    
    def execute_next_step(self):
        if self.current_step < len(self.test_sequence):
            pos1, pos2 = self.test_sequence[self.current_step]
            
            self.get_logger().info(f'发送命令: 电机1 -> {pos1}度, 电机2 -> {pos2}度')
            
            # 发送位置命令
            msg1 = Float32()
            msg1.data = float(pos1)
            self.motor1_pub.publish(msg1)
            
            msg2 = Float32()
            msg2.data = float(pos2)
            self.motor2_pub.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
