#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np
import math

class LekiwiRLDeployNode(Node):
    def __init__(self):
        super().__init__('lekiwi_rl_deploy')

        # 1. 加载模型 (建议先在 Isaac 环境用 torch.jit.save 导出的模型)
        # model_path 建议通过参数传入
        self.declare_parameter('model_path', 'lekiwi_model.pt')
        model_path = self.get_parameter('model_path').value
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        try:
            # 这里的模型应该是从 IsaacLab 导出的 TorchScript 文件
            self.policy = torch.jit.load(model_path).to(self.device)
            self.get_logger().info(f"Model loaded from {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

        # 2. 状态存储 (对齐 ObservationsCfg 的顺序)
        self.obs_base_lin_vel = np.zeros(3)  # x, y, z
        self.obs_base_ang_vel = np.zeros(3)  # wx, wy, wz
        self.obs_joint_pos = np.zeros(3)     # 3个轮子的相对角度
        self.obs_joint_vel = np.zeros(3)     # 3个轮子的角速度
        self.obs_command = np.zeros(3)       # 用户下发的 x, y, yaw 目标

        # 3. 订阅真实传感器
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        # 获取里程计用于获取线性速度（真机通常由EKF提供）
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 4. 发布力矩给 ros2_control
        # 注意：Topic 需要对应你的 effort_controller
        self.effort_pub = self.create_publisher(
            Float64MultiArray, 
            '/lekiwi_effort_controller/commands', 
            10
        )

        # 5. 控制定时器 (120Hz / 4 = 30Hz)
        self.control_freq = 30.0 
        self.timer = self.create_timer(1.0/self.control_freq, self.inference_loop)
        
        self.effort_scale = 15.0 # 对应你的 ActionsCfg 中的 scale

    # --- 数据回调函数 ---
    def imu_callback(self, msg):
        # 获取机体角速度
        self.obs_base_ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def js_callback(self, msg):
        # 匹配关节顺序：lekiwi 通常是按照配置里的正则顺序
        # 注意：这里可能需要根据你的 URDF 映射索引
        self.obs_joint_pos = np.array(msg.position)
        self.obs_joint_vel = np.array(msg.velocity)

    def odom_callback(self, msg):
        # 获取当前线速度 (应为 base_link 系)
        self.obs_base_lin_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

    def cmd_callback(self, msg):
        self.obs_command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    # --- 核心推理循环 ---
    def inference_loop(self):
        # 1. 拼接 Observation (必须严格对齐训练时的 PolicyCfg 顺序)
        # 顺序：lin_vel(3) + ang_vel(3) + joint_pos(3) + joint_vel(3) + command(3) = 15 dim
        input_obs = np.concatenate([
            self.obs_base_lin_vel,
            self.obs_base_ang_vel,
            self.obs_joint_pos,
            self.obs_joint_vel,
            self.obs_command
        ]).astype(np.float32)

        # 2. 转换为 Tensor 并推理
        obs_tensor = torch.from_numpy(input_obs).to(self.device).unsqueeze(0)
        
        with torch.no_grad():
            # 获取 Action (-1 到 1 之间)
            action = self.policy(obs_tensor).cpu().numpy().flatten()

        # 3. 映射到实际力矩
        target_effort = action * self.effort_scale
        
        # 4. 发布
        msg = Float64MultiArray()
        msg.data = target_effort.tolist()
        self.effort_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LekiwiRLDeployNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()