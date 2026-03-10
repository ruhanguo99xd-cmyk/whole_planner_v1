# 连续循环动作测试程序
# 包含：定点复位、定点回转、挖掘规划、复位规划功能

import snap7
from snap7 import util
import time
import numpy as np

from plc_common import (
    write_real_array,
    initialize_points,
    pulse_bit,
    quintic_trajectory_planning,
    reset_planning_function,
)

# 复位规划功能
def reset_planning_function(plc,tuiya_value):
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
        # target_tisheng = 11578
        target_tisheng = 7000
        # target_tuiya = 2828
        target_tuiya = tuiya_value

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


# ===========================================================
# 主程序
def main():
    plc = None
    try:
        # 显示菜单 - 增加正转和反转选项
        print("="*50)
        print("控制调试一直运动功能测试程序")
        print("="*50)
        
      
        # 连接PLC
        plc = snap7.client.Client()
        plc.connect('192.168.2.20', 0, 1)   # 修改为实际PLC IP地址
        if not plc.get_connected():
            raise Exception("PLC连接失败")
        print(f"PLC连接状态：{plc.get_connected()}")
        
        tisheng_targets = [ 10000,5000]
        tuiya_targets = [ 20000,6000]
        huizhuan_targets = [100.0,0.0]
        target_idx = 0

        while plc.get_connected():
            tuiya_target = tuiya_targets[target_idx]
            reset_planning_function(plc, tuiya_target)

            # tisheng_target = tisheng_targets[target_idx]
            # reset_planning_function(plc, tisheng_target)

            # huizhuan_target = huizhuan_targets[target_idx]
            # rotate_planning_function(plc, huizhuan_target)

            # 监测复位规划完成
            print("正在监测回转动作完成状态...")
            while True:
                status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
                fuwei_exec = util.get_bool(status_byte, 0, 0)  # 执行状态位 DB1300.DBX116.0（1执行中/0完成）
                if not fuwei_exec:
                    print("一轮复位规划动作已完成")
                    break
                time.sleep(0.1) # 每0.1秒检查一次状态

            print("="*50)

            target_idx = 1 - target_idx
            time.sleep(0.5)


    except Exception as e:
        print(f"操作失败：{str(e)}")
    finally:
        # 断开连接
        if plc and plc.get_connected():
            plc.disconnect()
            print("已断开PLC连接")

if __name__ == "__main__":
    main()
