#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import math
import numpy as np
import time  # <--- 修复了之前漏掉的导入
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer
from tf_transformations import quaternion_from_euler

class RTKOdomNode(Node):
    def __init__(self):
        super().__init__('rtk_to_odom')

        # --- 1. 参数配置 ---
        self.declare_parameter('rtk_host', '192.168.2.136')
        self.declare_parameter('rtk_port', 9904)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # RTK原点：建议设置为机器人启动时的参考点
        self.declare_parameter('origin_lat', 37.75845658)
        self.declare_parameter('origin_lon', 112.59322883)
        self.declare_parameter('origin_alt', 755.265)

        self.host = self.get_parameter('rtk_host').value
        self.port = self.get_parameter('rtk_port').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # --- 2. 坐标转换初始化 ---
        self.wgs84 = CRS("EPSG:4326")
        self.ecef = CRS("EPSG:4978")
        self.transformer = Transformer.from_crs(self.wgs84, self.ecef, always_xy=True)
        
        # 设置初始原点 ECEF 坐标
        lat0 = self.get_parameter('origin_lat').value
        lon0 = self.get_parameter('origin_lon').value
        alt0 = self.get_parameter('origin_alt').value
        self.origin_ecef = self.transformer.transform(lon0, lat0, alt0)
        self.origin_lla = (lat0, lon0)

        # --- 3. 状态变量 ---
        self.socket = None
        self.connected = False
        self.running = True
        
        # --- 4. ROS 发布者与 TF ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # 禁用 TF 广播，由 EKF 负责发布 TF
        # self.tf_broadcaster = TransformBroadcaster(self)

        # 启动接收线程
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f"RTK Odom 节点启动成功，尝试连接 {self.host}:{self.port}")

    def lla_to_enu(self, lat, lon, alt):
        """经纬高转局部 ENU 坐标"""
        # 1. 转换为 ECEF
        x, y, z = self.transformer.transform(lon, lat, alt)
        
        dx = x - self.origin_ecef[0]
        dy = y - self.origin_ecef[1]
        dz = z - self.origin_ecef[2]

        lat0_rad = math.radians(self.origin_lla[0])
        lon0_rad = math.radians(self.origin_lla[1])

        # 2. ENU 旋转矩阵 (从 ECEF 到 ENU)
        # ECEF 到 ENU 的投影公式
        e = -math.sin(lon0_rad) * dx + math.cos(lon0_rad) * dy
        n = -math.sin(lat0_rad) * math.cos(lon0_rad) * dx - math.sin(lat0_rad) * math.sin(lon0_rad) * dy + math.cos(lat0_rad) * dz
        u =  math.cos(lat0_rad) * math.cos(lon0_rad) * dx + math.cos(lat0_rad) * math.sin(lon0_rad) * dy + math.sin(lat0_rad) * dz
        
        return e, n, u

    def parse_rtk_line(self, line):
        """解析 $GPCHC 或类似 NMEA 数据"""
        try:
            if '$GPCHC' not in line: return None
            data = line.strip().split(',')
            # 基本长度检查
            if len(data) < 20: return None

            parsed = {
                'heading': float(data[6]) if data[6] else 0.0,
                'lat': float(data[16]) if data[16] else None,
                'lon': float(data[17]) if data[17] else None,
                'alt': float(data[18]) if data[18] else None,
                'status': data[1] if data[1] else '0'
            }
            return parsed
        except (ValueError, IndexError) as e:
            # 过滤掉不完整的行数据
            return None

    def receive_loop(self):
        """TCP 接收循环（带自动重连）"""
        buffer = ""
        while self.running:
            if not self.connected:
                try:
                    # 每次重连前先关闭旧 Socket
                    if self.socket:
                        self.socket.close()
                    
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.settimeout(5.0)  # 增加超时容忍度
                    self.socket.connect((self.host, self.port))
                    self.connected = True
                    self.get_logger().info("RTK 网络连接已建立")
                except Exception as e:
                    self.get_logger().warn(f"RTK 连接失败: {e}, 5秒后重试...")
                    time.sleep(5)
                    continue

            try:
                # 接收原始数据
                raw_data = self.socket.recv(4096)
                if not raw_data:
                    self.get_logger().error("RTK 连接被远程主机断开")
                    self.connected = False
                    continue
                
                buffer += raw_data.decode('utf-8', errors='ignore')
                
                # 处理完整行
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.startswith('$GPCHC'):
                        parsed = self.parse_rtk_line(line)
                        if parsed and parsed['lat'] is not None:
                            self.publish_odom(parsed)
                            
            except socket.timeout:
                continue # 正常超时，继续监听
            except Exception as e:
                self.get_logger().error(f"接收线程异常: {e}")
                self.connected = False
                time.sleep(2)

    def publish_odom(self, data):
        """发布 Odometry 消息和 TF 变换"""
        # 1. 经纬度转 ENU
        enu_x, enu_y, enu_z = self.lla_to_enu(data['lat'], data['lon'], data['alt'])
        self.get_logger().info(f"实时位置 ENU -> X: {enu_x:.3f}, Y: {enu_y:.3f}")
        
        # 2. 航向角转换 (RTK 北偏东 0-360 -> ROS 正东逆时针)
        # 假设 RTK 0度指向正北，90度正东
        heading_deg = data['heading']
        heading_ros = math.radians(90.0 - heading_deg)
        
        # 3. 安装偏移补偿 (Offset Compensation)
        # 1.79319 是天线中心相对于机器中心中心的纵向偏移，0.6 是横向偏移
        cos_h = math.cos(heading_ros)
        sin_h = math.sin(heading_ros)
        
        # 根据你的坐标系定义：
        # 如果天线在中心前方 1.79m, 右侧 0.6m
        final_x = enu_x - (1.79319 * cos_h - 0.6 * sin_h)
        final_y = enu_y - (1.79319 * sin_h + 0.6 * cos_h)

        now = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, heading_ros)

        # 4. 发布 Odometry 
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = final_x
        odom.pose.pose.position.y = final_y
        odom.pose.pose.position.z = 0.0 # 假设平面运行
        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # 简单赋予一个协方差（可选）
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        
        self.odom_pub.publish(odom)

        # 5. 不再发布 TF 变换，由 EKF 负责
        # t = TransformStamped()
        # t.header.stamp = now
        # t.header.frame_id = self.odom_frame
        # t.child_frame_id = self.base_frame
        # t.transform.translation.x = final_x
        # t.transform.translation.y = final_y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = RTKOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()