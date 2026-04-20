#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')
        
        # 参数初始化
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value
        self.base_radius = self.declare_parameter('base_radius', 0.125).value
        self.max_wheel_velocity = self.declare_parameter('max_wheel_velocity', 3.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value # 建议缩短超时
        self.ramp_rate = self.declare_parameter('ramp_rate', 5.0).value # 加速度限制
        self.control_freq = self.declare_parameter('control_freq', 60.0).value
        
        # 状态变量
        self.target_wheel_velocities = np.array([0.0, 0.0, 0.0])
        self.current_wheel_velocities = np.array([0.0, 0.0, 0.0])
        self.last_cmd_time = time.time()
        self.is_stopped = True # 初始状态为停止
        
        # 运动学矩阵计算
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        self.kinematic_matrix = np.array([
            [np.cos(angle), np.sin(angle), self.base_radius] 
            for angle in angles_rad
        ])
        
        # 通信
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_vel_pub = self.create_publisher(Float64MultiArray, '/lekiwi_wheel_controller/commands', 10)
        
        # 唯一的控制定时器
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_loop)
        self.get_logger().info("Holonomic controller (Ramp Optimized) started")

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        # 运动学解算
        body_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        target_ws = self.kinematic_matrix.dot(body_vel) / self.wheel_radius
        
        # 限速
        max_v = np.max(np.abs(target_ws))
        if max_v > self.max_wheel_velocity:
            target_ws = target_ws * (self.max_wheel_velocity / max_v)
            
        self.target_wheel_velocities = target_ws
        self.is_stopped = False

    def control_loop(self):
        now = time.time()
        dt = 1.0 / self.control_freq
        
        # 1. 超时检测：如果没收到指令，目标设为 0
        if (now - self.last_cmd_time) > self.cmd_timeout:
            if not self.is_stopped:
                self.get_logger().debug("Safety Timeout: Stopping robot")
                self.is_stopped = True
            self.target_wheel_velocities = np.array([0.0, 0.0, 0.0])

        # 2. 平滑滤波 (Ramp)
        diff = self.target_wheel_velocities - self.current_wheel_velocities
        max_change = self.ramp_rate * dt
        
        # clip 限制步进大小
        step = np.clip(diff, -max_change, max_change)
        self.current_wheel_velocities += step
        
        # 3. 发布消息
        msg = Float64MultiArray()
        msg.data = self.current_wheel_velocities.tolist()
        self.wheel_vel_pub.publish(msg)

def main():
    rclpy.init()
    controller = HolonomicController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0, 0.0, 0.0]
        controller.wheel_vel_pub.publish(stop_cmd)
        controller.get_logger().info("Sent stop command")
        
        rclpy.shutdown()

if __name__ == '__main__':
    main() 