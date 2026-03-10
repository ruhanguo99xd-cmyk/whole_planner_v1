#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import math
import numpy as np
import time
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
        
        # 新增：真正读取是否发布 TF 的开关
        self.declare_parameter('publish_tf', False) 
        self.should_publish_tf = self.get_parameter('publish_tf').value

        # RTK原点
        self.declare_parameter('origin_lat', 37.75844849)
        self.declare_parameter('origin_lon', 112.59322195)
        self.declare_parameter('origin_alt', 755.265)

        self.host = self.get_parameter('rtk_host').value
        self.port = self.get_parameter('rtk_port').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # --- 2. 坐标转换初始化 ---
        self.wgs84 = CRS("EPSG:4326")
        self.ecef = CRS("EPSG:4978")
        self.transformer = Transformer.from_crs(self.wgs84, self.ecef, always_xy=True)
        
        # 设置初始原点
        lat0 = self.get_parameter('origin_lat').value
        lon0 = self.get_parameter('origin_lon').value
        alt0 = self.get_parameter('origin_alt').value
        self.origin_ecef = self.transformer.transform(lon0, lat0, alt0)
        self.origin_lla = (lat0, lon0)

        # --- 3. 状态变量 ---
        self.socket = None
        self.connected = False
        self.running = True
        
        # --- 4. ROS 发布者 ---
        # 修改：为了避免和 EKF 的输出话题混淆，这里建议改为 gps_odom
        self.odom_pub = self.create_publisher(Odometry, 'gps_odom', 10)
        
        # 只有当允许发布 TF 时，才创建广播器
        if self.should_publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info("调试模式：节点将发布 TF (odom->base_link)")
        else:
            self.tf_broadcaster = None
            self.get_logger().info("标准模式：TF 发布已禁用 (交给 EKF 处理)")

        # 启动接收线程
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f"RTK Odom 节点启动成功，连接 {self.host}:{self.port}")

    def lla_to_enu(self, lat, lon, alt):
        """经纬高转局部 ENU 坐标"""
        x, y, z = self.transformer.transform(lon, lat, alt)
        dx = x - self.origin_ecef[0]
        dy = y - self.origin_ecef[1]
        dz = z - self.origin_ecef[2]

        lat0_rad = math.radians(self.origin_lla[0])
        lon0_rad = math.radians(self.origin_lla[1])

        e = -math.sin(lon0_rad) * dx + math.cos(lon0_rad) * dy
        n = -math.sin(lat0_rad) * math.cos(lon0_rad) * dx - math.sin(lat0_rad) * math.sin(lon0_rad) * dy + math.cos(lat0_rad) * dz
        u =  math.cos(lat0_rad) * math.cos(lon0_rad) * dx + math.cos(lat0_rad) * math.sin(lon0_rad) * dy + math.sin(lat0_rad) * dz
        
        return e, n, u

    def parse_rtk_line(self, line):
        """
        解析 $GPCHC 数据
        参考手册：1.4.1 GPCHC 数据协议 (Page 10)
        格式: $GPCHC,Week,Time,Heading,Pitch,Roll,gx,gy,gz,ax,ay,az,Lat,Lon,Alt,Ve,Vn,Vu,V...
        """
        try:
            if not line.startswith('$GPCHC'): 
                return None
            
            # 移除校验位(*hh)和换行符，再分割
            if '*' in line:
                line = line.split('*')[0]
            
            data = line.strip().split(',')
            
            # GPCHC 协议通常至少有 24 个字段
            if len(data) < 20: 
                return None

            # --- 关键修正区域 ---
            
            # 1. 状态解析 (Index 21)
            # 手册定义：系统状态(低半字节) + 卫星状态(高半字节)
            try:
                raw_status = int(data[21]) 
            except:
                raw_status = 0
            
            # 2. 航向角 (Index 3)
            heading = float(data[3]) if data[3] else 0.0
            
            # 3. 经纬高 (Index 12, 13, 14)
            lat = float(data[12]) if data[12] else 0.0
            lon = float(data[13]) if data[13] else 0.0
            alt = float(data[14]) if data[14] else 0.0

            # 简单的有效性过滤：如果经纬度全为0，视为无效
            if abs(lat) < 0.1 and abs(lon) < 0.1:
                return None

            parsed = {
                'heading': heading,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'status': raw_status, 
                'warning': data[23] if len(data) > 23 else '0'
            }
            
            # 打印调试信息，验证读取是否正确
            # self.get_logger().info(f"解析成功: Lat={lat:.6f}, Lon={lon:.6f}, Head={heading:.2f}")
            
            return parsed

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"数据解析异常: {e} | 数据: {line}")
            return None

    def receive_loop(self):
        """TCP 接收循环（带自动重连与Buffer保护）"""
        buffer = ""
        MAX_BUFFER_SIZE = 81920  # 防止 buffer 无限增长
        
        while self.running:
            # --- 1. 连接管理 ---
            if not self.connected:
                try:
                    if self.socket: 
                        self.socket.close()
                    
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.settimeout(5.0)
                    self.socket.connect((self.host, self.port))
                    self.connected = True
                    self.get_logger().info("RTK 网络连接已建立")
                    buffer = "" # 重连时清空旧缓存
                except Exception as e:
                    self.get_logger().warn(f"RTK 连接失败: {e}, 5秒后重试...")
                    time.sleep(5)
                    continue

            # --- 2. 数据接收 ---
            try:
                raw_data = self.socket.recv(4096)
                if not raw_data:
                    self.get_logger().error("RTK 连接被远程断开")
                    self.connected = False
                    continue
                
                # 解码并追加到 buffer
                buffer += raw_data.decode('utf-8', errors='ignore')
                
                # --- 3. 粘包处理循环 ---
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    # 校验协议头（确保是完整的一行）
                    if line.startswith('$GPCHC'):
                        # 调用我们刚才修正过的解析函数
                        parsed = self.parse_rtk_line(line)
                        if parsed and parsed['status'] != 0: # 稍微加个状态判断更安全
                            self.publish_odom(parsed)
                
                # --- 4. Buffer 溢出保护 ---
                if len(buffer) > MAX_BUFFER_SIZE:
                    self.get_logger().warn("Buffer 过大，强制清空（可能收到无效数据）")
                    buffer = ""
                            
            except socket.timeout:
                continue # 正常超时，维持心跳
            except Exception as e:
                self.get_logger().error(f"接收线程异常: {e}")
                self.connected = False
                time.sleep(2)

    def publish_odom(self, data):
        # 1. 经纬度转 ENU
        enu_x, enu_y, enu_z = self.lla_to_enu(data['lat'], data['lon'], data['alt'])
        # self.get_logger().info(f"实时位置 ENU -> X: {enu_x:.3f}, Y: {enu_y:.3f}")
        
        # 2. 航向角转换
        heading_deg = data['heading']
        heading_ros = math.radians(90.0 - heading_deg)
        
        # 3. 安装偏移补偿 (保留这里！因为你是在计算 base_link 的位置)
        # cos_h = math.cos(heading_ros)
        # sin_h = math.sin(heading_ros)
        
        # 这里计算的是 base_link 在 map/odom 中的坐标
        final_x = enu_x
        final_y = enu_y
        
        # final_x = enu_x - (1.79319 * cos_h - 0.6 * sin_h)
        # final_y = enu_y - (1.79319 * sin_h + 0.6 * cos_h)

        now = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, heading_ros)

        # 4. 发布 Odometry 消息
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame  # 这里应该修改为gps_antenna_link?
        # odom.child_frame_id = self.base_frame # 这里填 base_link 是对的，因为上面已经补偿了偏置
        
        odom.pose.pose.position.x = final_x
        odom.pose.pose.position.y = final_y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # 协方差
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        
        self.odom_pub.publish(odom)

        # 5. 条件发布 TF
        if self.should_publish_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = 'base_link'
            t.transform.translation.x = final_x
            t.transform.translation.y = final_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except: pass
def main(args=None):
    # 移除或注释掉重复的 rclpy.init
    if not rclpy.ok():
        rclpy.init(args=args)
        
    node = RTKOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 在 shutdown 之前先销毁节点
        node.destroy_node()
        # 增加判断，防止重复 shutdown
        if rclpy.ok():
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()
    