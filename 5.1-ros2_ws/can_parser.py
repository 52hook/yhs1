import can
import struct
import csv
import time  # 用于记录时间戳

def parse_chassis_feedback(data):
    """解析底盘控制反馈帧（ID=0x18C4D2EF）"""
    # 1. 目标档位
    gear_raw = data[0] & 0x0F
    gear_map = {0: "disable", 1: "P档", 2: "R档", 3: "N档", 4: "D档"}
    gear = gear_map.get(gear_raw, f"未知档位({gear_raw})")

    # 2. 当前车体速度（m/s）
    speed_raw = ((data[0] >> 4) << 8) | data[1]
    speed = speed_raw * 1

    # 3. 当前车体转向角（°）
    angle_raw = struct.unpack('<h', bytes([data[2], data[3]]))[0]
    angle = angle_raw * 1

    # 4. 当前车辆运行模式
    mode_raw = data[5] & 0x03
    mode_map = {0: "auto", 1: "remote", 2: "stop"}
    mode = mode_map.get(mode_raw, f"未知模式({mode_raw})")

    # 5. Alive心跳信号
    alive_raw = ((data[6] << 4) | (data[7] >> 4)) & 0x0F

    return {
        "目标档位": gear,
        "当前速度(m/s)": round(speed, 3),
        "转向角(°)": round(angle, 2),
        "运行模式": mode,
        "心跳计数": alive_raw
    }

def save_to_csv(data, filename="can_feedback_data.csv"):
    """保存解析后的数据到CSV文件"""
    fieldnames = ["时间戳", "目标档位", "当前速度(m/s)", "转向角(°)", "运行模式", "心跳计数"]
    
    with open(filename, mode='a+', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        
        # 首次写入时添加表头
        file.seek(0)
        if not file.read(1):
            writer.writeheader()
        
        writer.writerow(data)

def main():
    bus = can.interface.Bus(
        bustype='socketcan',
        channel='can0',
        bitrate=500000
    )
    print("开始监听CAN总线，按Ctrl+C退出...")
    print("逻辑：速度首次不为0后，持续保存所有数据（包括速度再次为0的情况）")
    
    # 状态标志：是否已出现过速度非零（一旦为True，后续持续保存）
    has_non_zero_speed = False  # 初始为False
    
    try:
        while True:
            msg = bus.recv(1)
            if msg and msg.arbitration_id == 0x18C4D2EF:
                parsed = parse_chassis_feedback(msg.data)
                print(f"ID=0x{msg.arbitration_id:X} 反馈：{parsed}")
                
                # 核心逻辑：更新状态标志并判断是否保存
                current_speed = parsed["当前速度(m/s)"]
                
                # 第一步：如果还没出现过非零速度，检查当前是否为非零
                if not has_non_zero_speed:
                    if current_speed != 0:
                        has_non_zero_speed = True  # 标记为已出现非零速度
                        print("检测到速度非零，开始持续保存数据...")
                
                # 第二步：如果已出现过非零速度，无论当前速度是否为0，都保存
                if has_non_zero_speed:
                    parsed_with_timestamp = {
                        "时间戳": time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                        **parsed
                    }
                    save_to_csv(parsed_with_timestamp)
                    
    except KeyboardInterrupt:
        print("\n程序退出")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()

