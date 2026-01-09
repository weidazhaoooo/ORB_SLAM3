#!/usr/bin/env python3
"""
WitMotion IMU数据解析
解析维特智能IMU的标准协议数据
"""

import serial
import time
import struct

class WitMotionIMU:
    """维特智能IMU解析器"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # 数据存储
        self.acc = [0.0, 0.0, 0.0]  # 加速度 m/s²
        self.gyro = [0.0, 0.0, 0.0]  # 角速度 rad/s
        self.angle = [0.0, 0.0, 0.0]  # 角度 degree
        self.mag = [0.0, 0.0, 0.0]  # 磁场
        
    def open(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(0.1)
            print(f"✓ 成功打开IMU: {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"✗ 无法打开IMU: {e}")
            return False
    
    def close(self):
        """关闭串口"""
        if self.ser:
            self.ser.close()
    
    def parse_packet(self, data):
        """解析数据包
        
        WitMotion协议格式:
        0x55 0x51: 加速度数据
        0x55 0x52: 角速度数据  
        0x55 0x53: 角度数据
        0x55 0x54: 磁场数据
        """
        if len(data) < 11:
            return None
        
        if data[0] != 0x55:
            return None
        
        packet_type = data[1]
        
        # 提取数据 (小端序)
        ax_raw = struct.unpack('<h', bytes(data[2:4]))[0]
        ay_raw = struct.unpack('<h', bytes(data[4:6]))[0]
        az_raw = struct.unpack('<h', bytes(data[6:8]))[0]
        
        if packet_type == 0x51:  # 加速度
            # 量程 ±16g
            self.acc[0] = ax_raw / 32768.0 * 16 * 9.8  # 转换为 m/s²
            self.acc[1] = ay_raw / 32768.0 * 16 * 9.8
            self.acc[2] = az_raw / 32768.0 * 16 * 9.8
            return 'acc'
            
        elif packet_type == 0x52:  # 角速度
            # 量程 ±2000°/s
            self.gyro[0] = ax_raw / 32768.0 * 2000 * 3.14159 / 180  # 转换为 rad/s
            self.gyro[1] = ay_raw / 32768.0 * 2000 * 3.14159 / 180
            self.gyro[2] = az_raw / 32768.0 * 2000 * 3.14159 / 180
            return 'gyro'
            
        elif packet_type == 0x53:  # 角度
            # 量程 ±180°
            self.angle[0] = ax_raw / 32768.0 * 180
            self.angle[1] = ay_raw / 32768.0 * 180
            self.angle[2] = az_raw / 32768.0 * 180
            return 'angle'
            
        elif packet_type == 0x54:  # 磁场
            self.mag[0] = ax_raw
            self.mag[1] = ay_raw
            self.mag[2] = az_raw
            return 'mag'
        
        return None
    
    def read_data(self):
        """读取并解析数据"""
        if not self.ser or not self.ser.is_open:
            return None
        
        buffer = []
        
        while self.ser.in_waiting > 0 or len(buffer) < 11:
            byte = self.ser.read(1)
            if len(byte) == 0:
                break
            
            buffer.append(byte[0])
            
            # 查找包头 0x55
            if len(buffer) >= 2 and buffer[-2] == 0x55:
                # 等待完整包
                while len(buffer) < 11 and self.ser.in_waiting > 0:
                    buffer.append(self.ser.read(1)[0])
                
                if len(buffer) >= 11:
                    # 从倒数第11个字节开始取包
                    packet = buffer[-11:]
                    data_type = self.parse_packet(packet)
                    buffer = []
                    return data_type
        
        return None

def main():
    print("=" * 70)
    print("WitMotion IMU 数据读取测试")
    print("=" * 70)
    
    imu = WitMotionIMU('/dev/ttyUSB0', 230400)
    
    if not imu.open():
        return
    
    print("\n开始读取IMU数据... (按Ctrl+C停止)\n")
    
    acc_count = 0
    gyro_count = 0
    angle_count = 0
    
    start_time = time.time()
    
    try:
        while True:
            data_type = imu.read_data()
            
            if data_type == 'acc':
                acc_count += 1
                if acc_count % 10 == 0:  # 每10个包显示一次
                    print(f"加速度 Acc: X={imu.acc[0]:7.3f} Y={imu.acc[1]:7.3f} Z={imu.acc[2]:7.3f} m/s²")
                    
            elif data_type == 'gyro':
                gyro_count += 1
                if gyro_count % 10 == 0:
                    print(f"角速度 Gyro: X={imu.gyro[0]:7.3f} Y={imu.gyro[1]:7.3f} Z={imu.gyro[2]:7.3f} rad/s")
                    
            elif data_type == 'angle':
                angle_count += 1
                if angle_count % 10 == 0:
                    print(f"角度 Angle: Roll={imu.angle[0]:7.2f} Pitch={imu.angle[1]:7.2f} Yaw={imu.angle[2]:7.2f} °")
            
            # 每秒显示统计
            elapsed = time.time() - start_time
            if elapsed >= 5:
                print(f"\n--- 统计 (5秒) ---")
                print(f"加速度包: {acc_count} ({acc_count/elapsed:.1f} Hz)")
                print(f"角速度包: {gyro_count} ({gyro_count/elapsed:.1f} Hz)")
                print(f"角度包: {angle_count} ({angle_count/elapsed:.1f} Hz)")
                print()
                acc_count = gyro_count = angle_count = 0
                start_time = time.time()
    
    except KeyboardInterrupt:
        print("\n\n正在停止...")
    
    finally:
        imu.close()
        print("已关闭IMU连接")

if __name__ == "__main__":
    main()
