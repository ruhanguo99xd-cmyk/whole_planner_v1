# 分步调试功能测试程序
# 包含：定点复位、定点回转、挖掘规划、复位规划功能

import snap7
from snap7 import util
import time
import numpy as np
from pathlib import Path

from plc_common import (
    write_real_array,
    initialize_points,
    pulse_bit,
    quintic_trajectory_planning,
    dig_function,
    reset_planning_function,
    xieliao_function,
    fuwei_function,
    rotate_planning_function,
    duqu,
)


# ===========================================================
# 主程序
def main():
    plc = None
    try:
        # 显示菜单 - 增加正转和反转选项
        print("="*50)
        print("上位机分步调试功能测试")
        print("1. 执行回转规划（回转）功能")
        print("2. 执行挖掘规划（推压/提升）功能")
        print("3. 执行复位规划（推压/提升）功能")
        print("4. 执行卸料规划（推压/提升/回转）功能")
        print("5. 执行卸料复位规划（推压/提升/回转）功能")
        print("6. 执行开斗动作")
        print("7. 急停复位")
        print("8. 提升松闸")
        print("9. 推压松闸")
        print("10. 回转松闸")
        print("11. 回转角度清零")
        print("12. 读取当前状态")
        print("0. 退出程序")
        print("="*50)
        
        # 获取用户选择
        choice = input("请输入功能编号（0-10）：").strip()
        if choice not in ['0', '1', '2', '3', '4' ,'5','6','7','8','9','10','11','12']:
            print("输入错误，请输入0-11之间的数字")
            return
        
        if choice == '0':
            print("程序已退出")
            return
        
        # 连接PLC
        plc = snap7.client.Client()
        plc.connect('192.168.2.20', 0, 1)   # 修改为实际PLC IP地址
        if not plc.get_connected():
            raise Exception("PLC连接失败")
        print(f"PLC连接状态：{plc.get_connected()}")
        
        # 根据选择执行对应功能
        if choice == '1':
            huizhuan_current_data = plc.db_read(1300, 88, 4)
            huizhuan_current = util.get_real(huizhuan_current_data, 0)
                    
            print(f"当前回转角度值：{huizhuan_current:.2f}")

            while True:
                angle_input = input("请输入回转角度：").strip()
                try:
                    angle = float(angle_input)  # 支持整数和小数角度
                    break
                except ValueError:
                    print("输入无效")
            rotate_planning_function(plc, angle)  
        elif choice == '2':
            dig_function(plc)
        elif choice == '3':
            reset_planning_function(plc)
        elif choice == '4':
            xieliao_function(plc)
        elif choice == '5':
            fuwei_function(plc)
        elif choice == '6':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 0, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(2.0)  # 开斗电机持续2s中
            print("开斗动作已完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 0, 0)
            plc.db_write(1301, 12040, status_byte)
            print("关闭开斗状态")

        elif choice == '7':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 1, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)  # 开斗电机持续2s中
            print("急停复位完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 1, 0)
            plc.db_write(1301, 12040, status_byte)
            print("关闭急停复位")
        
        elif choice == '8':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 2, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)  
            print("提升松闸完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 2, 0)
            plc.db_write(1301, 12040, status_byte)
            print("提升松闸复位")

        elif choice == '9':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 3, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)  
            print("推压松闸完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 3, 0)
            plc.db_write(1301, 12040, status_byte)
            print("推压松闸复位")

        elif choice == '10':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 4, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)  
            print("回转松闸完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 4, 0)
            plc.db_write(1301, 12040, status_byte)
            print("回转松闸复位")

        elif choice == '11':
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 5, 1)  # 开斗启动
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)  
            print("回转清零完成")

            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 5, 0)
            plc.db_write(1301, 12040, status_byte)
            print("回转清零复位")

        elif choice == '12':
            duqu(plc)

    except Exception as e:
        print(f"操作失败：{str(e)}")
    finally:
        # 断开连接
        if plc and plc.get_connected():
            plc.disconnect()
            print("已断开PLC连接")

if __name__ == "__main__":
    main()