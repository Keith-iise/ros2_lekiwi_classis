#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import numpy as np
import math

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0; aj /= 2.0; ak /= 2.0
        ci = math.cos(ai); si = math.sin(ai)
        cj = math.cos(aj); sj = math.sin(aj)
        ck = math.cos(ak); sk = math.sin(ak)
        return [si*cj*ck - ci*sj*sk, ci*sj*ck + si*cj*sk, ci*cj*sk - si*sj*sk, ci*cj*ck + si*sj*sk]

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # 参数获取
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        
        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        self.last_time = self.get_clock().now()
        
        # 订阅控制指令替代 JointStates，用于验证逻辑
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        
        # 主循环定时器 (50Hz)
        self.timer = self.create_timer(0.02, self.update_and_publish) 
        self.get_logger().info("Command-based Odometry (Debug Mode) started")

    def cmd_vel_callback(self, msg):
        """直接把指令作为当前速度（理想状态）"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0.0 or dt > 0.5:
            self.last_time = current_time
            return

        # --- 核心积分逻辑 ---
        # 如果你只发了 vx，那么 delta_theta 必须为 0
        delta_theta = self.vtheta * dt
        
        # 将局部坐标系的速度转换到全局坐标系
        # 哪怕 vtheta 为 0，如果 vx/vy 很大，theta 也会影响 x/y 的走向
        avg_theta = self.theta + delta_theta / 2.0  # 使用中值角度增加精度
        
        delta_x = (self.vx * math.cos(avg_theta) - self.vy * math.sin(avg_theta)) * dt
        delta_y = (self.vx * math.sin(avg_theta) + self.vy * math.cos(avg_theta)) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 角度归一化 (-pi, pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- 发布消息 (同前) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, \
        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q
        
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta
        
        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x, t.transform.rotation.y, \
            t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_broadcaster.sendTransform(t)
            
        self.last_time = current_time

def main():
    rclpy.init()
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 