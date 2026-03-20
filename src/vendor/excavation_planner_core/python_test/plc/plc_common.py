# 通用函数

import snap7
from snap7 import util
import time
import numpy as np
from pathlib import Path


def resolve_csv_dir(subdir: str) -> Path:
    current = Path(__file__).resolve()
    for ancestor in current.parents:
        candidate = ancestor / "src" / "vendor" / "excavation_planner_core" / "csv_created" / subdir
        if candidate.exists():
            return candidate
        legacy_candidate = ancestor / "csv_created" / subdir
        if legacy_candidate.exists():
            return legacy_candidate
    return current.parents[3] / "csv_created" / subdir

# ===========================================================
# 通用函数
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
def quintic_trajectory_planning(s0, st, T, delta_t=0.1):
    """
    五次多项式轨迹规划
    s0: 起始位置
    st: 目标位置
    T: 总运动时间(s)
    delta_t: 采样间隔(s)
    返回: 规划后的位置数组
    """
    # 计算采样点数（保证固定采样间隔，N必须为整数）
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


# ===========================================================
# 核心动作
# 挖掘功能
def dig_function(plc):

    CSV_DIR = resolve_csv_dir("trajectory_planner")
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

# 卸料功能
def xieliao_function(plc):

    CSV_DIR = resolve_csv_dir("load")
    gan_path = CSV_DIR / 'load_gan_enc.csv'
    rope_path = CSV_DIR / 'load_rope_enc.csv'
    huizhuan_path = CSV_DIR / 'load_fix_rotation_deg.csv'

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
            raise Exception(f"读取卸料数据文件失败：{str(e)}")
        
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
        util.set_bool(byte_bool, 0, 4, 1)  # 回转自动允许
        util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭，关闭可以保证在预限位处不受PLC内部减速影响
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 7. 触发位置数组执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("卸料动作已执行")
        
    except Exception as e:
        print(f"卸料功能出错：{str(e)}")

# 卸料复位功能
def fuwei_function(plc):

    CSV_DIR = resolve_csv_dir("return")
    gan_path = CSV_DIR / 'return_gan_enc.csv'
    rope_path = CSV_DIR / 'return_rope_enc.csv'
    huizhuan_path = CSV_DIR / 'return_fix_rotation_deg.csv'

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
            raise Exception(f"读取复位数据文件失败：{str(e)}")
        
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
        util.set_bool(byte_bool, 0, 4, 1)  # 回转自动允许
        util.set_bool(byte_bool, 0, 5, 1)  # 预限屏蔽关闭，关闭可以保证在预限位处不受PLC内部减速影响
        plc.db_write(1301, 12026, byte_bool)
        time.sleep(0.1)
        
        # 7. 触发位置数组执行（上升沿）
        pulse_bit(plc, 1301, 12026, 0, pulse_ms=300)
        print("复位动作已执行")
        
    except Exception as e:
        print(f"复位功能出错：{str(e)}")

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
        target_tisheng = 11538
        target_tuiya = 2817

        # target_tisheng = 4839
        # target_tuiya = 13530

        print(f"目标提升编码器值：{target_tisheng}")
        print(f"目标推压编码器值：{target_tuiya}")
        
        target_huizhuan = 0.0  # 复位目标回转角度为0度
        print(f"目标回转角度值：{target_huizhuan}")

        # 4. 轨迹规划参数设置
        total_time = 12.0  # 总运动时间（秒）
        sample_interval = 0.1  # 采样间隔（秒）
        sample_points = int(total_time / sample_interval) + 1
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
            tisheng_current, target_tisheng, total_time, sample_interval
        )
        tuiya_array = quintic_trajectory_planning(
            tuiya_current, target_tuiya, total_time, sample_interval
        )

        # 回转角度复位特殊处理：如果当前角度与目标角度差值过大，则不执行回转动作
        if abs(huizhuan_current - target_huizhuan) > 5.0:
            print("当前回转角度与目标角度相差较大，复位规划不执行回转动作,请注意检查。")
            huizhuan_array = [huizhuan_current] * sample_points  # 保持当前角度不变
        else:
            huizhuan_array = quintic_trajectory_planning(
                huizhuan_current, target_huizhuan, total_time, sample_interval
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
        total_time = 15.0  # 总运动时间（秒）
        sample_interval = 0.1  # 采样间隔（秒）
        sample_points = int(total_time / sample_interval) + 1
        print(f"开始五次多项式规划：总时间{total_time}秒，采样点{sample_points}个")
        
        # 5. 生成轨迹数组
        huizhuan_array = quintic_trajectory_planning(
            huizhuan_current, angle, total_time, sample_interval
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


# 上位机读取当前状态
def duqu(plc):
    try:
        # 1. 初始化点位
        initialize_points(plc)
        
        # 2. 读取当前提升和推压编码器数值、回转角度
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
        
        # 3. 读取状态字节，判断各机构松闸状态
        status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
        tisheng_done = util.get_bool(status_byte, 0, 4)  # 提升松闸状态位 DB1300.DBX116.4
        tuiya_done = util.get_bool(status_byte, 0, 5)    # 推压松闸状态位 DB1300.DBX116.5
        huizhuan_done = util.get_bool(status_byte, 0, 6) # 回转松闸状态位 DB1300.DBX116.6

        if tisheng_done:
            print("提升松闸")
        else:
            print("提升报闸")
        
        if tuiya_done:
            print("推压松闸")
        else:
            print("推压报闸")

        if huizhuan_done:
            print("回转松闸")
        else:
            print("回转报闸")
        
        
    except Exception as e:
        print(f"复位规划功能出错：{str(e)}")
