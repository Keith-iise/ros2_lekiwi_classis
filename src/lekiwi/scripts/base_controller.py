#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64MultiArray
import tf2_ros
import numpy as np
import math

# 欧拉角转四元数辅助函数
try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0; aj /= 2.0; ak /= 2.0
        ci = math.cos(ai); si = math.sin(ai)
        cj = math.cos(aj); sj = math.sin(aj)
        ck = math.cos(ak); sk = math.sin(ak)
        return [si*cj*ck - ci*sj*sk, ci*sj*ck + si*cj*sk, ci*cj*sk - si*sj*sk, ci*cj*ck + si*sj*sk]

class LekiwiIntegratedNode(Node):
    def __init__(self):
        super().__init__('lekiwi_integrated_node')
        
        # === 1. 参数声明 ===
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value
        self.base_radius = self.declare_parameter('base_radius', 0.125).value
        self.max_wheel_velocity = self.declare_parameter('max_wheel_velocity', 3.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.ramp_rate = self.declare_parameter('ramp_rate', 5.0).value
        self.control_freq = self.declare_parameter('control_freq', 50.0).value
        
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        # === 2. 状态变量 ===
        # 控制相关
        self.target_body_vel = np.array([0.0, 0.0, 0.0]) # vx, vy, wz
        self.current_wheel_vels = np.array([0.0, 0.0, 0.0])
        self.last_cmd_time = self.get_clock().now()
        
        # 里程计相关
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_step_time = self.get_clock().now()

        # === 3. 运动学矩阵 ===
        # 三轮全向轮布局
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        self.kinematic_matrix = np.array([
            [np.cos(angle), np.sin(angle), self.base_radius] 
            for angle in angles_rad
        ])

        # === 4. 通信接口 ===
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_vel_pub = self.create_publisher(Float64MultiArray, '/lekiwi_wheel_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # === 5. 主循环定时器 ===
        self.timer = self.create_timer(1.0/self.control_freq, self.main_loop)
        
        self.get_logger().info("Lekiwi Integrated Controller & Odom started")

    def cmd_vel_callback(self, msg):
        """处理输入的控制指令"""
        self.last_cmd_time = self.get_clock().now()
        self.target_body_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def main_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_step_time).nanoseconds / 1e9
        if dt <= 0: return

        # --- A. 超时逻辑与目标设定 ---
        time_since_last_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last_cmd > self.cmd_timeout:
            active_target_body_vel = np.array([0.0, 0.0, 0.0])
        else:
            active_target_body_vel = self.target_body_vel

        # --- B. 运动控制逻辑 (Ramp + Kinematics) ---
        # 1. 解算目标轮速
        target_wheel_vels = self.kinematic_matrix.dot(active_target_body_vel) / self.wheel_radius
        
        # 2. 幅值限速
        max_v = np.max(np.abs(target_wheel_vels))
        if max_v > self.max_wheel_velocity:
            target_wheel_vels *= (self.max_wheel_velocity / max_v)

        # 3. 平滑演进 (Ramp)
        diff = target_wheel_vels - self.current_wheel_vels
        max_change = self.ramp_rate * dt
        self.current_wheel_vels += np.clip(diff, -max_change, max_change)

        # 4. 发布轮子指令
        wheel_msg = Float64MultiArray()
        wheel_msg.data = self.current_wheel_vels.tolist()
        self.wheel_vel_pub.publish(wheel_msg)

        # --- C. 里程计逻辑 (使用当前实际速度积分) ---
        # 注意：这里我们使用 active_target_body_vel 来积分，这反映了“理想”的命令轨迹
        # 如果你想反映 Ramp 后的轨迹，可以根据 current_wheel_vels 反解出 body_vel
        vx, vy, wz = active_target_body_vel
        
        # 即使轮子停了，只要命令变 0，这里的 vx/vy/wz 也会变 0，停止漂移
        delta_theta = wz * dt
        avg_theta = self.theta + delta_theta / 2.0
        
        self.x += (vx * math.cos(avg_theta) - vy * math.sin(avg_theta)) * dt
        self.y += (vx * math.sin(avg_theta) + vy * math.cos(avg_theta)) * dt
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- D. 发布里程计与 TF ---
        self.publish_odom_and_tf(now)

        self.last_step_time = now

    def publish_odom_and_tf(self, now):
        q = quaternion_from_euler(0, 0, self.theta)
        now_msg = now.to_msg()

        # Odom 消息
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        
        # 这里的速度反映的是指令速度
        odom.twist.twist.linear.x = self.target_body_vel[0]
        odom.twist.twist.linear.y = self.target_body_vel[1]
        odom.twist.twist.angular.z = self.target_body_vel[2]
        self.odom_pub.publish(odom)

        # TF 变换
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x, t.transform.rotation.y, \
            t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LekiwiIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停机安全指令
        stop_node = rclpy.create_node('stop_node')
        pub = stop_node.create_publisher(Float64MultiArray, '/lekiwi_wheel_controller/commands', 10)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        pub.publish(msg)
        node.get_logger().info("Shutting down... Sent zero velocity.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()