# Author: pipixuan
# 固定轨迹动作（省长参观那一套)
# 要使用时需要修改launch.py，替换plc_control_test1 节点为 plc_control_test
# 同时注释掉 launch.py中动作客户端节点 prsdata_client
# 可以注释掉 load、return、perceive_truck_server节点，避免干扰 
import snap7
from snap7 import util
import time
import numpy as np
from pathlib import Path

from typing import Callable, Optional

# ros2通讯相关
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # 用Bool类型传递启动/完成信号

"代码抽象为三层："
"1. 业务逻辑层：do_one_dig_cycle()等函数，描述挖掘动作的业务逻辑流程"
"2. 执行层：wait_until()函数的通用机制，描述等待PLC完成的通用逻辑"
"3. 接口层：PerceptionCommNode类和plc_done()函数，描述具体怎么和ROS2/PLC交互"

# ======================== 新增ROS2节点类 ========================
class PerceptionCommNode(Node):
    def __init__(self):
        super().__init__('plc_control')
        # 1. 创建发布者：向规划系统发送"准许感知启动计算"信号（话题名可根据实际ROS2系统修改）
        self.start_pub = self.create_publisher(Bool, 'digging/perception_start', 10)
        # 1.1 创建发布者：向装载/复位节点发送“挖掘开始”信号
        self.excavation_start_pub = self.create_publisher(Bool, 'digging/excavation_start', 10)
        # 2. 创建订阅者：接收规划系统"感知计算完成"信号（话题名需与感知系统一致）
        self.finish_sub = self.create_subscription(
            Bool, 'digging/perception_finish', self.finish_callback, 10
        )
        # 3. 标志位：标记是否收到感知完成信号
        self.perception_finished = False
        self.get_logger().info("ROS2动作逻辑控制节点已启动")

    # 感知完成信号回调函数
    def finish_callback(self, msg):
        if msg.data:  # 若收到True信号，说明感知计算完成
            self.perception_finished = True
            self.get_logger().info("感知系统计算完成")

    # 发布"启动感知"信号的函数
    def publish_start_signal(self):
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        self.get_logger().info("向挖掘节点发送允许感知计算信号")

    # 发布“挖掘开始”信号
    def publish_excavation_start(self):
        start_msg = Bool()
        start_msg.data = True
        self.excavation_start_pub.publish(start_msg)
        self.get_logger().info("向挖掘/装载节点发送开始执行挖掘动作信号")

def call_perception_and_wait(perception_node: PerceptionCommNode):
    # ① 先清零，保证是“新一轮”
    perception_node.perception_finished = False

    print("发送感知启动信号...")
    # ② 再发 start
    perception_node.publish_start_signal()

    print("等待规划/感知系统计算完成...")
    # ③ 等待期间，只允许回调把它从 False -> True
    start_time = time.time()
    timeout = 10.0  # 设置超时时间为10秒
    while not perception_node.perception_finished and rclpy.ok():
        if time.time() - start_time > timeout:
            perception_node.get_logger().error("感知任务超时！")
            break
        rclpy.spin_once(perception_node, timeout_sec=0.05)

    if perception_node.perception_finished:
        print("感知系统计算完成。")
    else:
        print("感知等待过程中 rclpy 已退出或异常。")


def plc_done(plc, db: int = 1300, byte_off: int = 116, bit: int = 0) -> bool:
    """目前PLC执行动作数组完成的通用读取地址都是DB1300.DBX116.0"""
    b = plc.db_read(db, byte_off, 1)
    return not util.get_bool(b, 0, bit)

def wait_until(
    *,
    name: str,   # 名字：日志/报错定位用
    node: Optional[Node],     # ROS2节点：用于 spin_once，让订阅回调能执行
    is_done: Callable[[], bool],   # 完成判据：每轮循环调用一次；返回True则结束wait_until
    on_enter: Optional[Callable[[], None]] = None,  # 进入时调用，只执行一次（触发动作/发信号），不参与循环
    on_tick: Optional[Callable[[], None]] = None,  # 每轮循环执行一次：心跳/重发/故障急停检查等
    timeout_sec: float = 60.0,   # 超时：超过则抛 TimeoutError，防止永远等待
    poll_sec: float = 0.05,     # 每轮循环末尾sleep时间；实际周期 = 循环体耗时 + poll_sec
    log_every_sec: float = 2.0, # 等待日志打印间隔
):
    """
    通用等待框架：
    - 进入时执行 on_enter（可选）
    - 循环中：spin_once(可选) + on_tick(可选) + is_done
    - 超时抛 TimeoutError
    """
    print(f"\n--- {name} ---")
    if on_enter:
        on_enter()

    t0 = time.time()
    last_log = 0.0

    while True:
        # 1) 处理ROS2回调（让perception_finish等回调不会饿死）
        if node is not None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            # ！！！这是wait_until这个函数的重要作用：能够让程序在等待 PLC 完成位的期间，也不断让 ROS2 回调有机会执行。
            # 通俗的理解就是动作执行期间也能接收 ROS2 消息

        # 2) 每轮循环可以插入“重发/心跳/安全检查”等，这里
        if on_tick:
            on_tick()

        # 3) 完成判据
        if is_done():
            print(f"[OK] {name} 完成")
            return

        # 4) 超时
        now = time.time()
        if now - t0 >= timeout_sec:
            raise TimeoutError(f"[TIMEOUT] {name} 超时（{timeout_sec:.1f}s）")

        # 5) 日志节流
        if now - last_log >= log_every_sec:
            print(f"[WAIT] {name} ... {now - t0:.1f}s")
            last_log = now

        time.sleep(poll_sec)

# =====================PLC通讯相关================================
# 通用函数：写入REAL数组
def write_real_array(client, db_number, start_offset, array_values):
    """写入REAL数组到PLC指定DB块"""
    array_size = len(array_values)
    size = array_size * 4  # 每个REAL占4字节
    data = bytearray(size)
    
    for i, value in enumerate(array_values):
        util.set_real(data, i * 4, value)
    
    client.db_write(db_number, start_offset, data)

# 通用函数：初始化执行点位
def initialize_points(plc):
    """重置DB1301.DBX12026相关控制位"""
    byte_bool = plc.db_read(1301, 12026, 1)  # 读取控制字节
    # 重置所有控制位为0
    util.set_bool(byte_bool, 0, 0, 0)  # 位置数组_执行开始信号
    util.set_bool(byte_bool, 0, 1, 0)  # 定点复位_执行开始信号
    util.set_bool(byte_bool, 0, 2, 0)  # 提升自动允许信号
    util.set_bool(byte_bool, 0, 3, 0)  # 推压自动允许信号
    util.set_bool(byte_bool, 0, 4, 0)  # 回转自动允许信号
    util.set_bool(byte_bool, 0, 5, 0)  # 预限屏蔽信号关闭
    plc.db_write(1301, 12026, byte_bool)  # 写回PLC
    time.sleep(0.5)
    print("初始化完成")

# 通用函数：模拟上位机脉冲函数
def pulse_bit(plc, db_number: int, byte_offset: int, bit_idx: int, pulse_ms: int = 300):
    """
    上位机模拟上升沿：对 DB[byte_offset].bit_idx 执行 0->1->0 脉冲
    注意：保证脉宽 >= 2×OB1周期，并留有通信延迟裕量
    """
    # 先读该字节，避免覆盖同字节其它控制位
    b = bytearray(plc.db_read(db_number, byte_offset, 1))

    # 确保先为0
    util.set_bool(b, 0, bit_idx, 0)
    plc.db_write(db_number, byte_offset, b)
    time.sleep(0.05)

    # 拉高（上升沿）
    util.set_bool(b, 0, bit_idx, 1)
    plc.db_write(db_number, byte_offset, b)
    time.sleep(pulse_ms / 1000.0)

    # 复位
    util.set_bool(b, 0, bit_idx, 0)
    plc.db_write(db_number, byte_offset, b)

# 五次多项式轨迹规划函数
def quintic_trajectory_planning(s0, st, T):
    """
    五次多项式轨迹规划
    s0: 起始位置
    st: 目标位置
    T: 总运动时间(s)
    返回: 规划后的位置数组
    """
    # 计算采样点数（保证0.1s间隔，N必须为整数）
    delta_t = 0.1  # 采样间隔（秒）
    N = int(T / delta_t) + 1  # 强制转换为整数，避免浮点数错误

    # 时间序列（0到T秒，共N个点）
    t = np.linspace(0, T, N)
    
    # 五次多项式系数计算（满足起点和终点速度、加速度为0的边界条件）
    a0 = s0
    a1 = 0  # 初始速度为0
    a2 = 0  # 初始加速度为0
    a3 = (20 * (st - s0) - 8 * a1 * T - 3 * a2 * T**2) / (2 * T**3)
    a4 = (-30 * (st - s0) + 14 * a1 * T + 5 * a2 * T**2) / (2 * T**4)
    a5 = (12 * (st - s0) - 6 * a1 * T - 2 * a2 * T**2) / (2 * T**5)
    
    # 计算每个时刻的位置
    position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    return position.tolist()  # 转换为Python列表

# ============================核心动作=============================
# 挖掘功能
def dig_function(plc):
    PROJECT_ROOT = Path(__file__).resolve().parents[3]
    CSV_DIR = PROJECT_ROOT.parent / "csv_created" / "trajectory_planner"
    gan_path = CSV_DIR / 'gan_enc.csv'
    rope_path = CSV_DIR / 'rope_enc.csv'
    huizhuan_path = CSV_DIR / 'huizhuan_enc.csv'

    try:
        # 1. 初始化点位
        initialize_points(plc)
        
        # 2. 读取各数组数据
        try:
            with open(gan_path, 'r') as f:
                gan_vals = [float(x) for x in f.readline().split(',') if x.strip()]  #gan_enc.csv,

            with open(rope_path, 'r') as f:
                rope_vals = [float(x) for x in f.readline().split(',') if x.strip()]  #rope_enc.csv  ， rope_enc_test.csv 
            
            with open(huizhuan_path, 'r') as f:
                huizhuan_vals = [float(x) for x in f.readline().split(',') if x.strip()]
        except Exception as e:
            raise Exception(f"读取挖掘数据文件失败：{str(e)}")
        
        # 3. 统一数组长度（取最小长度）
        # n = 151
        n = min(len(gan_vals), len(rope_vals), len(huizhuan_vals))
        if n == 0:
            raise Exception("数组为空，无法执行挖掘动作")
        
        gan_vals = gan_vals[:n] 
        rope_vals = rope_vals[:n] 
        huizhuan_vals = huizhuan_vals[:n]
        print(f"已统一数组长度为：{n}")
        
        # 4. 写入数组数量
        byte_int = bytearray(2)
        util.set_int(byte_int, 0, n)
        plc.db_write(1301, 0, byte_int)
        print(f"已写入数组数量：{n}")
        
        # 5. 写入各数组数据
        write_real_array(plc, 1301, 2006, rope_vals)    # 提升编码器
        write_real_array(plc, 1301, 6014, gan_vals)     # 推压编码器
        write_real_array(plc, 1301, 10022, huizhuan_vals)  # 回转角度
        print("已写入提升、推压、回转数组数据")
        
        # 6. 允许所有自动信号
        byte_bool = plc.db_read(1301, 12026, 1)
        util.set_bool(byte_bool, 0, 2, 1)  # 提升自动允许
        util.set_bool(byte_bool, 0, 3, 1)  # 推压自动允许
        util.set_bool(byte_bool, 0, 4, 0)  # 回转自动允许
        util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭，关闭可以保证在预限位处不受PLC内部减速影响
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 7. 触发位置数组执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("挖掘动作已执行")
        
    except Exception as e:
        print(f"挖掘功能出错：{str(e)}")

# 复位规划功能
def reset_planning_function(plc):
    try:
        # 1. 初始化点位
        initialize_points(plc)
        
        # 2. 读取当前提升和推压编码器数值
        # 提升当前值（DB1300.92）
        tisheng_current_data = plc.db_read(1300, 92, 4)
        tisheng_current = util.get_real(tisheng_current_data, 0)
        
        # 推压当前值（DB1300.96）
        tuiya_current_data = plc.db_read(1300, 96, 4)
        tuiya_current = util.get_real(tuiya_current_data, 0)
        
        # 回转当前值（DB1300.88）
        huizhuan_current_data = plc.db_read(1300, 88, 4)
        huizhuan_current = util.get_real(huizhuan_current_data, 0)

        print(f"当前提升编码器值：{tisheng_current:.2f}")
        print(f"当前推压编码器值：{tuiya_current:.2f}")
        print(f"当前回转角度值：{huizhuan_current:.2f}")
        
        # 3. 获取目标位置
        target_tisheng = 11578
        # target_tuiya = 4428
        target_tuiya = 2828

        print(f"目标提升编码器值：{target_tisheng}")
        print(f"目标推压编码器值：{target_tuiya}")
        
        target_huizhuan = 0.0  # 复位目标回转角度为0度
        print(f"目标回转角度值：{target_huizhuan}")

        # 4. 轨迹规划参数设置
        total_time = 12.0  # 总运动时间（秒）
        sample_points = 121  # 采样点数（越多越平滑）
        print(f"开始五次多项式规划：总时间{total_time}秒，采样点{sample_points}个")

        # ===== 添加用户确认环节 =====
        # while True:
        #     user_input = input("请确认目标位置是否正确，确认执行请输入'y'，取消请输入'n'：").strip().lower()
        #     if user_input in ['y', 'yes']:
        #         print("用户已确认，开始执行复位规划...")
        #         break
        #     elif user_input in ['n', 'no']:
        #         print("用户取消执行，复位规划功能终止。")
        #         return  # 退出函数，不再执行
        #     else:
        #         print("输入无效，请重新输入（'y'确认 / 'n'取消）：")
        
        # 5. 生成轨迹数组
        tisheng_array = quintic_trajectory_planning(
            tisheng_current, target_tisheng, total_time
        )
        tuiya_array = quintic_trajectory_planning(
            tuiya_current, target_tuiya, total_time
        )

        # 回转角度复位特殊处理：如果当前角度与目标角度差值过大，则不执行回转动作
        if abs(huizhuan_current - target_huizhuan) > 5.0:
            print("当前回转角度与目标角度相差较大，复位规划不执行回转动作,请注意检查。")
            huizhuan_array = [huizhuan_current] * sample_points  # 保持当前角度不变
        else:
            huizhuan_array = quintic_trajectory_planning(
                huizhuan_current, target_huizhuan, total_time
            )

        # 6. 写入数组数量
        byte_int = bytearray(2)
        util.set_int(byte_int, 0, sample_points)
        plc.db_write(1301, 0, byte_int)
        print(f"已写入数组数量：{sample_points}")
        
        # 7. 写入规划数组到PLC（与挖掘功能共用数组地址）
        write_real_array(plc, 1301, 2006, tisheng_array)  # 提升数组
        write_real_array(plc, 1301, 6014, tuiya_array)    # 推压数组
        write_real_array(plc, 1301, 10022, huizhuan_array)  # 回转数组
        print("已写入提升/推压/回转规划轨迹数据")
        
        # 8. 允许自动执行
        byte_bool = plc.db_read(1301, 12026, 1)
        util.set_bool(byte_bool, 0, 2, 1)  # 提升自动允许
        util.set_bool(byte_bool, 0, 3, 1)  # 推压自动允许
        util.set_bool(byte_bool, 0, 4, 1)  # 回转自动允许
        util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 9. 触发执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("复位规划轨迹已开始执行")
        
    except Exception as e:
        print(f"复位规划功能出错：{str(e)}")

# 卸载前回收规划功能
def xiezai_qian_planning(plc):
    try:
        # 1. 初始化点位
        initialize_points(plc)
        
        # 2. 读取当前推压/提升数值
        # 推压当前值（DB1300.96）
        tuiya_current_data = plc.db_read(1300, 96, 4)
        tuiya_current = util.get_real(tuiya_current_data, 0)

        # 提升当前值（DB1300.92）
        tisheng_current_data = plc.db_read(1300, 92, 4)
        tisheng_current = util.get_real(tisheng_current_data, 0)
                
        print(f"当前推压对应编码器值：{tuiya_current:.2f}")
        print(f"当前提升对应编码器值：{tisheng_current:.2f}")
        
        # 3. 轨迹规划参数设置
        total_time = 4.0  # 总运动时间（秒）
        sample_points = 41  # 采样点数（越多越平滑）
        print(f"开始五次多项式规划：总时间{total_time}秒，采样点{sample_points}个")
        
        # tuiya = tuiya_current-4000

        # tisheng = tisheng_current-1500

        tuiya = 15353
        tisheng = 3330

        print(f"回收后推压对应编码器值：{tuiya:.2f}")
        print(f"回收后提升对应编码器值：{tisheng:.2f}")

        # 5. 生成轨迹数组
        tuiya_array = quintic_trajectory_planning(
            tuiya_current, tuiya, total_time
        )

        tisheng_array = quintic_trajectory_planning(
            tisheng_current, tisheng, total_time
        )
        
        # 6. 写入数组数量
        byte_int = bytearray(2)
        util.set_int(byte_int, 0, sample_points)
        plc.db_write(1301, 0, byte_int)
        print(f"已写入数组数量：{sample_points}")
        
        # 7. 写入规划数组到PLC（与挖掘功能共用数组地址）
        write_real_array(plc, 1301, 6014, tuiya_array)  # 推压数组
        write_real_array(plc, 1301, 2006, tisheng_array)  # 提升数组
        print("已写入推压/提升编码器数据")
        
        # 8. 允许自动执行
        byte_bool = plc.db_read(1301, 12026, 1)
        util.set_bool(byte_bool, 0, 2, 1)  # 提升自动允许
        util.set_bool(byte_bool, 0, 3, 1)  # 推压自动允许
        util.set_bool(byte_bool, 0, 4, 0)  # 回转自动允许
        # util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 9. 触发执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("回转规划轨迹已开始执行")
        
    except Exception as e:
        print(f"回转规划功能出错：{str(e)}")

# 回转规划功能
def rotate_planning_function(plc,angle):
    try:
        # 1. 初始化点位
        initialize_points(plc)
        
        # 2. 读取当前回转角度数值
        # 回转角度当前值（DB1300.88）
        huizhuan_current_data = plc.db_read(1300, 88, 4)
        huizhuan_current = util.get_real(huizhuan_current_data, 0)
                
        print(f"当前回转角度值：{huizhuan_current:.2f}")
        
        # 3. 轨迹规划参数设置
        total_time = 10.0  # 总运动时间（秒）
        sample_points = 101  # 采样点数（越多越平滑）
        print(f"开始五次多项式规划：总时间{total_time}秒，采样点{sample_points}个")
        
        # 5. 生成轨迹数组
        huizhuan_array = quintic_trajectory_planning(
            huizhuan_current, angle, total_time
        )
        
        # 6. 写入数组数量
        byte_int = bytearray(2)
        util.set_int(byte_int, 0, sample_points)
        plc.db_write(1301, 0, byte_int)
        print(f"已写入数组数量：{sample_points}")
        
        # 7. 写入规划数组到PLC（与挖掘功能共用数组地址）
        write_real_array(plc, 1301, 10022, huizhuan_array)  # 回转数组
        print("已写入回转角度数据")
        
        # 8. 允许自动执行
        byte_bool = plc.db_read(1301, 12026, 1)
        util.set_bool(byte_bool, 0, 2, 0)  # 提升自动允许
        util.set_bool(byte_bool, 0, 3, 0)  # 推压自动允许
        util.set_bool(byte_bool, 0, 4, 1)  # 回转自动允许
        # util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 9. 触发执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("回转规划轨迹已开始执行")
        
    except Exception as e:
        print(f"回转规划功能出错：{str(e)}")

"""单次循环: 回转0°→复位→挖掘→回收→卸载→开斗→(通知感知)→ 完整一轮"""

def do_one_dig_cycle(plc, perception_node: Optional[PerceptionCommNode] = None):

    # ！！！！！每次循环前开斗都需要清0，避免上一次程序中断未清0导致第一斗出现关不上斗的问题
    status_byte = plc.db_read(1301, 12040, 1)
    util.set_bool(status_byte, 0, 0, 0)
    plc.db_write(1301, 12040, status_byte)
    print("关闭开斗状态")
    time.sleep(0.5)

    """单次循环: 回转0°→复位→挖掘→回收→卸载→开斗→(通知感知)→ 完整一轮"""
    # 1）回转0°
    wait_until(
        name="回转0°准备复位",
        node=perception_node,
        on_enter=lambda: rotate_planning_function(plc, 0),
        is_done=lambda: plc_done(plc),
        timeout_sec=60.0,
        poll_sec=0.05,
    )
    time.sleep(0.5)

    # 2）复位规划
    wait_until(
        name="复位规划执行并完成",
        node=perception_node,
        on_enter=lambda: reset_planning_function(plc),
        is_done=lambda: plc_done(plc),
        timeout_sec=90.0,
        poll_sec=0.05,
    )
    time.sleep(0.5)

    # 3）挖掘动作
    def start_dig():
        if perception_node is not None:
            perception_node.publish_excavation_start()
        dig_function(plc)

    wait_until(
        name="挖掘动作执行并完成",
        node=perception_node,
        on_enter=start_dig,
        is_done=lambda: plc_done(plc),
        timeout_sec=180.0,
        poll_sec=0.05,
    )
    time.sleep(0.5)

    # 4）卸载前回收
    wait_until(
        name="卸载前回收执行并完成",
        node=perception_node,
        on_enter=lambda: xiezai_qian_planning(plc),
        is_done=lambda: plc_done(plc),
        timeout_sec=60.0,
        poll_sec=0.05,
    )
    time.sleep(0.5)

    # 5）卸载（回转90°）
    wait_until(
        name="卸载：回转90°并完成",
        node=perception_node,
        on_enter=lambda: rotate_planning_function(plc, 90),
        is_done=lambda: plc_done(plc),
        timeout_sec=60.0,
        poll_sec=0.05,
    )

    # 6）开斗动作
    print("\n--- 执行开斗动作 ---")
    status_byte = plc.db_read(1301, 12040, 1)
    util.set_bool(status_byte, 0, 0, 1)  # 开斗启动
    plc.db_write(1301, 12040, status_byte)
    time.sleep(3.0)
    print("开斗动作已完成")

    status_byte = plc.db_read(1301, 12040, 1)
    util.set_bool(status_byte, 0, 0, 0)
    plc.db_write(1301, 12040, status_byte)
    print("关闭开斗状态")

# ===========================================================
# ====================== 主程序（核心修改环节） ======================
def main():
    # 1. 初始化ROS2节点
    rclpy.init()
    perception_node = PerceptionCommNode()  # 创建启动感知通讯节点
    
    plc = None

    try:
        print("="*50)
        print("ROS2：||回转90-扫料-感知-循环(回转0-复位-挖掘-回收-卸载-开斗-感知)|| 调试")
        print("="*50)

        # # 2. 连接PLC
        plc = snap7.client.Client()
        plc.connect('192.168.2.20', 0, 1)   # 需修改为实际PLC IP
        if not plc.get_connected():
            raise Exception("PLC连接失败")
        print(f"PLC连接状态：{plc.get_connected()}")

        # -------------------------- 连上PLC启动后需要用户确认在运行 --------------------------
        # while True:
        #     user_input = input("\n请确认是否开始执行连续动作流程？（输入 'y' 执行 / 'n' 退出）：").strip().lower()
        #     if user_input == 'y':
        #         print("用户已确认，开始执行动作流程...")
        #         break  # 确认后退出循环，继续执行后续动作
        #     elif user_input == 'n':
        #         print("用户取消执行，程序即将退出")
        #         return  # 取消则直接退出main函数，不执行后续动作
        #     else:
        #         print("输入无效，请重新输入（仅支持 'y' 或 'n'）")
        # ----------------------------------------------------------------------
        
        # ===================== 第一次：回转90° + 感知 =====================
        wait_until(
            name="预动作：回转90°扫描物料",
            node=perception_node,
            on_enter=lambda: rotate_planning_function(plc, 90),
            is_done=lambda: plc_done(plc),
            timeout_sec=60.0,
            poll_sec=0.05,
        )

        print("\n--- 步骤2：第一次启动感知（回转90°后） ---")
        call_perception_and_wait(perception_node)

        # ===================== 启动循环：机械动作 + 感知 =====================
        loop_idx = 1
        while rclpy.ok():
            print("\n" + "="*50)
            print(f"开始第 {loop_idx} 轮：回转0-复位-挖掘-回收-卸载-开斗-感知")
            print("="*50)

            # 一轮完整机械动作：回转0°→复位→挖掘→回收→卸载→开斗→回0°→闭斗→空挖准备
            do_one_dig_cycle(plc, perception_node)

            # 开斗动作结束后，第二次/第三次/... 通知感知
            print("\n--- 本轮开斗结束，通知感知 ---")
            call_perception_and_wait(perception_node)

            loop_idx += 1

        print("\n循环结束（rclpy 不再可用）")

    except KeyboardInterrupt:
        print("检测到用户中断（Ctrl+C），程序即将退出")
    except Exception as e:
        print(f"操作失败：{str(e)}")
    finally:
        # 断开PLC连接 + 销毁ROS2节点
        if plc and plc.get_connected():
            plc.disconnect()
            print("已断开PLC连接")

        perception_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
