#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import time

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom')

        # -----------------------------
        # Parameters (initial pose)
        # -----------------------------
        self.declare_parameter('initial_x', 14.0)
        self.declare_parameter('initial_y', 6.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('track_width', 1.925)
        self.declare_parameter('forbid_in_place_spin', True)
        self.declare_parameter('min_linear_for_turn', 0.12)
        self.declare_parameter('spin_angular_threshold', 0.08)
        self.declare_parameter('min_turning_radius', 5.0)
        
        # -----------------------------
        # State variables
        # -----------------------------
        self.x: float = self.get_parameter('initial_x').get_parameter_value().double_value
        self.y: float = self.get_parameter('initial_y').get_parameter_value().double_value
        self.theta: float = self.get_parameter('initial_theta').get_parameter_value().double_value
        self.track_width: float = self.get_parameter('track_width').get_parameter_value().double_value
        self.forbid_in_place_spin: bool = self.get_parameter('forbid_in_place_spin').get_parameter_value().bool_value
        self.min_linear_for_turn: float = self.get_parameter('min_linear_for_turn').get_parameter_value().double_value
        self.spin_angular_threshold: float = self.get_parameter('spin_angular_threshold').get_parameter_value().double_value
        self.min_turning_radius: float = self.get_parameter('min_turning_radius').get_parameter_value().double_value

        # 速度变量 (新增)
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.last_linear_sign = 1.0

        # -----------------------------
        # TF broadcasters
        # -----------------------------
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # -----------------------------
        # Subscribers (新增：订阅速度指令)
        # -----------------------------
        self.create_subscription(
            Twist,
            '/cmd_vel',  # 确保这里的名字和 nav2 发出的一致
            self.cmd_vel_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/fake_odom', 10)

        # -----------------------------
        # Timers
        # -----------------------------
        # 1. 动态 TF 更新频率提高到 20Hz (0.05s)，让移动更丝滑
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_pose_and_tf)
        
        # 2. 静态 map->odom 发布 (保持不变，用于假定位)
        self.broadcast_static_tf() 
        self.static_timer = self.create_timer(1.0, self.broadcast_static_tf)

        self.get_logger().info(
            f'FakeOdom initialized at ({self.x:.2f}, {self.y:.2f}) with cmd_vel listener.'
        )

    def cmd_vel_callback(self, msg):
        """接收速度指令"""
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        if abs(v) > 1e-3:
            self.last_linear_sign = 1.0 if v > 0.0 else -1.0

        if self.forbid_in_place_spin:
            if abs(v) < self.min_linear_for_turn and abs(w) > self.spin_angular_threshold:
                v = self.last_linear_sign * self.min_linear_for_turn

            if abs(v) < 1e-4:
                w = 0.0

            radius_limit = abs(v) / max(1e-3, self.min_turning_radius)
            same_direction_limit = (2.0 * abs(v) / max(1e-3, self.track_width)) * 0.98
            w_limit = min(radius_limit, same_direction_limit)
            if abs(w) > w_limit:
                w = w_limit if w >= 0.0 else -w_limit

        self.vx = v
        self.vth = w

    def broadcast_static_tf(self):
        """Publish map -> odom transform"""
        # 这里模拟定位完全准确，map和odom重合
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def update_pose_and_tf(self):
        """核心逻辑：计算位移并发布 odom -> base_link"""
        
        # 1. 计算时间差 (更严谨的做法是用当前时间减去上次时间，这里为了简单直接用定时器周期)
        current_time = self.get_clock().now()
        # dt = (current_time - self.last_time).nanoseconds / 1e9
        # self.last_time = current_time
        dt = self.dt # 简单近似

        # 2. 运动学积分 (简单的差分驱动/履带模型)
        # x += v * cos(theta) * dt
        # y += v * sin(theta) * dt
        # theta += omega * dt
        if self.vx != 0.0 or self.vth != 0.0:
            delta_x = (self.vx * math.cos(self.theta)) * dt
            delta_y = (self.vx * math.sin(self.theta)) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_th

        # 3. 构建并发布 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # 欧拉角转四元数
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # 4. 发布 fake_odom 里程计，供 HMI 等节点订阅
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
