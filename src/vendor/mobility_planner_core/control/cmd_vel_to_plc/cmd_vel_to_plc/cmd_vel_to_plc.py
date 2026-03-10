#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import snap7
from snap7 import util
import threading
import time
from mode_enable.msg import MachineModeState

class SafePlcBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_plc')
        
        # --- 1. 参数声明与获取 ---
        self.declare_parameter('plc_ip', '192.168.2.20')
        self.declare_parameter('wheel_base', 1.925)      # 轮距 L (米)
        self.declare_parameter('read_interval', 0.1)  # 读取间隔
        self.declare_parameter('max_speed', 1.0)      # 最大物理速度限制 (m/s)
        self.declare_parameter('forbid_in_place_spin', True)    # 禁止原地自转
        self.declare_parameter('min_linear_for_turn', 0.12)     # 转弯时最小线速度
        self.declare_parameter('spin_angular_threshold', 0.08)  # 判定原地自转的角速度阈值
        self.declare_parameter('min_turning_radius', 5.0)       # 最小允许转弯半径
        
        self.plc_ip = self.get_parameter('plc_ip').value
        self.L = self.get_parameter('wheel_base').value
        self.read_interval = self.get_parameter('read_interval').value
        self.max_speed = self.get_parameter('max_speed').value
        self.forbid_in_place_spin = bool(self.get_parameter('forbid_in_place_spin').value)
        self.min_linear_for_turn = float(self.get_parameter('min_linear_for_turn').value)
        self.spin_angular_threshold = float(self.get_parameter('spin_angular_threshold').value)
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.last_linear_sign = 1.0

        # --- 2. 核心变量初始化 (必须在调用任何函数前) ---
        self.scaling_factor = 16384.0       #16384.0        # PLC 速度缩放基数
        self.write_lock = threading.Lock()    # 线程锁，防止指令冲突
        self.control_enabled = False         # 控制就绪标志
        self.hardware_ready = False         # 硬件就绪标志
        # self.machine_mode = None


        # --- 3. 硬件连接 ---
        self.plc = snap7.client.Client()
        self.connect_plc()
        # --- 3.5 订阅 /machine_mode_state ---
        self.mode_sub = self.create_subscription(
            MachineModeState,
            '/machine_mode_state',
            self.mode_callback,
            10
        )
        

        # --- 4. 执行硬件初始化序列 ---
        # 此时 safe_write_speed 所需的 max_speed 和 scaling_factor 已定义
        self.initial_sequence()

        # --- 5. 开启 ROS 订阅 ---
        # 最后开启订阅，确保只有在硬件初始化完成后才处理控制指令
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.get_logger().info('✅ 系统初始化完成，控制循环已激活')

    def connect_plc(self):
        """连接或重连 PLC"""
        try:
            if not self.plc.get_connected():
                self.plc.connect(self.plc_ip, 0, 1) 
                self.get_logger().info(f'成功连接到PLC: {self.plc_ip}')
        except Exception as e:
            self.get_logger().error(f"PLC 连接失败: {e}")
            
            
    def mode_callback(self, msg: MachineModeState):
        self.machine_mode = msg

        walking_allowed = (
            msg.mode == 2 and
            msg.walking_active and
            msg.mode not in [3, 4, 5]
        )

        if not self.hardware_ready or not walking_allowed:
            if self.control_enabled:
               self.get_logger().warn(
                   f"🚫 行走被互锁，mode={msg.mode}, hardware_ready={self.hardware_ready}"
               )
            self.control_enabled = False
            self.safe_write_speed(0.0, 0.0)
        else:
            if not self.control_enabled:
                self.get_logger().info("✅ 行走互锁解除，允许速度控制")
            self.control_enabled = True



    def initial_sequence(self):
        """硬件初始化：速度归零 -> 复位脉冲 -> 松闸脉冲"""
        self.get_logger().info("正在执行硬件初始化序列...")
        
        try:
            # 1. 初始速度强制设为 0
            self.safe_write_speed(0.0, 0.0)
            
            # 2. 发送复位和松闸脉冲 (根据 DB1301 地址逻辑)
            db_num = 1301
            addr = 12040
            
            # 复位脉冲 (Bit 1)
            self._send_pulse(db_num, addr, 1)
            time.sleep(0.5)
            
            # 松闸脉冲 (Bit 6, 7)
            self._send_pulse(db_num, addr, [6, 7])
            
            # 3. 标记硬件就绪
            # self.control_enabled = True
            self.hardware_ready = True
            self.get_logger().info("硬件就绪，等待模式许可")
        except Exception as e:
            self.get_logger().error(f"初始化序列执行崩溃: {e}")

    def kinematics_conversion(self, v, w):
        """差速运动学换算：将线速度和角速度转为左右轮速度"""
        if abs(v) > 1e-3:
            self.last_linear_sign = 1.0 if v > 0.0 else -1.0

        if self.forbid_in_place_spin:
            # 1) 原地自转命令改成最小弧线转向（不允许左右履带反向）
            if abs(v) < self.min_linear_for_turn and abs(w) > self.spin_angular_threshold:
                v = self.last_linear_sign * self.min_linear_for_turn
                self.get_logger().warn("检测到原地自转指令，已改为弧形转向")

            # 2) 若线速度几乎为零，直接禁止角速度，防止静止扭转
            if abs(v) < 1e-4:
                w = 0.0

            # 3) 按最小转弯半径限制角速度：|w| <= |v| / R_min
            if self.min_turning_radius > 1e-3:
                w_limit_radius = abs(v) / self.min_turning_radius
                if abs(w) > w_limit_radius:
                    w = w_limit_radius if w >= 0.0 else -w_limit_radius

            # 4) 按履带同向约束限制角速度，确保左右履带不会一正一负
            if self.L > 1e-3 and abs(v) > 1e-4:
                w_limit_same_direction = (2.0 * abs(v) / self.L) * 0.98
                if abs(w) > w_limit_same_direction:
                    w = w_limit_same_direction if w >= 0.0 else -w_limit_same_direction

        # 标准差速公式
        v_l = v - (w * self.L / 2.0)
        v_r = v + (w * self.L / 2.0)

        # 最终保护：若仍出现反向履带，则降级为纯直行速度
        if self.forbid_in_place_spin and (v_l * v_r < 0.0):
            avg_v = 0.5 * (v_l + v_r)
            v_l = avg_v
            v_r = avg_v

        return float(v_l), float(v_r)

    def cmd_vel_callback(self, msg):
        """ROS 速度指令回调"""
        self.get_logger().info(f"收到cmd_vel: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        self.get_logger().info(f"control_enabled状态: {self.control_enabled}")
        
        if not self.hardware_ready or not self.control_enabled:
            self.get_logger().warn("❌ 行走被互锁或硬件未就绪")
            return


        l_speed, r_speed = self.kinematics_conversion(msg.linear.x, msg.angular.z)
        self.get_logger().info(f"转换后速度: left={l_speed:.3f}, right={r_speed:.3f}")
        self.safe_write_speed(l_speed, r_speed)
       

    def safe_write_speed(self, left, right):
        """将物理速度(m/s)转换为PLC整数并写入"""
        with self.write_lock:
            if not self.plc.get_connected():
                self.get_logger().error("❌ PLC连接失败，速度写入被跳过！")
                self.connect_plc()
            
            try:
                # 换算公式: (当前速度 / 最大速度) * 16384
                left_scaled = int((left / self.max_speed) * self.scaling_factor)
                right_scaled = int((right / self.max_speed) * self.scaling_factor)
                
                # === 旧方案注释保留 ===
                # 原方案：使用DINT格式写入DB151.DBD12/16
                # 边界溢出限制
                left_scaled = max(min(left_scaled, 16384), -16384)
                right_scaled = max(min(right_scaled, 16384), -16384)    #旧程序
                
                # 写入 PLC (假设为 DINT 4字节类型)
                buf = bytearray(8)
                util.set_real(buf, 0, left_scaled) 
                util.set_real(buf, 4, right_scaled)
                # self.control_enabled = True
                self.plc.db_write(151, 12, buf)
                
                #---------------------------------------
                
                # left_scaled = max(min(left_scaled, 32767), -32768)
                # right_scaled = max(min(right_scaled, 32767), -32768)
                
                # # === 新写入方案：使用WORD格式分别写入两个DB块 ===
                # # 左转速写入 DB15.DBW6
                # left_buf = bytearray(2)
                # util.set_int(left_buf, 0, left_scaled)  # set_int用于16位整数
                # self.plc.db_write(15, 6, left_buf)
                
                # # 右转速写入 DB16.DBW6  
                # right_buf = bytearray(2)
                # util.set_int(right_buf, 0, right_scaled)  # set_int用于16位整数
                # self.plc.db_write(16, 6, right_buf)


            except Exception as e:
                self.get_logger().error(f"写入速度异常: {e}")

    def _send_pulse(self, db, byte, bits):
        """内部辅助：向指定位发送电平脉冲"""
        if isinstance(bits, int): bits = [bits]
        try:
            # 置位 (True)
            data = self.plc.read_area(snap7.type.Areas.DB, db, byte, 1)
            for b in bits: util.set_bool(data, 0, b, True)
            self.plc.write_area(snap7.type.Areas.DB, db, byte, data)
            
            time.sleep(0.5) # 维持脉冲宽度
            
            # 复位 (False)
            data = self.plc.read_area(snap7.type.Areas.DB, db, byte, 1)
            for b in bits: util.set_bool(data, 0, b, False)
            self.plc.write_area(snap7.type.Areas.DB, db, byte, data)
        except Exception as e:
            self.get_logger().error(f"脉冲发送失败: {e}")

    def on_shutdown(self):
        """安全关闭逻辑"""
        self.get_logger().info("正在安全停车并释放硬件连接...")
        self.control_enabled = False
        try:
            # 尝试发送 0 速度
            self.safe_write_speed(0.0, 0.0)
            self.plc.disconnect()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SafePlcBridge()
    try:
        # 主线程运行ROS2 spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保在完全关闭前执行停车逻辑
        node.on_shutdown()
        node.destroy_node()
        # 注意：这里如果报错 rcl_shutdown already called，是因为
        # context 已经在外部或信号处理中被关闭，属于 ROS2 Humble 正常现象
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
    
    # ⚠️ 注意：
# PLC 中 walking_active 与 walking_enable 为同一物理点位
# 互锁逻辑仅在 ROS 中完成，PLC 不承担模式裁决
