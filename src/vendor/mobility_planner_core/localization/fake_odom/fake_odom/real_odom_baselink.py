#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math

class RealOdomBaseLink(Node):
    def __init__(self):
        super().__init__('real_odom_base_link')

        # 状态变量：x, y, 偏航角theta
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅由 Nav2 或遥控器发出的速度指令
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 20Hz 更新频率，保证 Nav2 运行平滑
        self.timer = self.create_timer(0.05, self.update_and_publish)
        self.get_logger().info("Odom -> BaseLink (Pure Integration) Node Started")

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0: return

        # 简单的运动学积分（假设是差分或履带模型）
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vth * dt

        # 发布 TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(RealOdomBaseLink())
    rclpy.shutdown()