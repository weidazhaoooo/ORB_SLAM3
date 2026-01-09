#!/usr/bin/env python3
"""
分析IMU数据格式和波特率
Analyze IMU data format and baudrate
"""

import serial
import time
import struct

def test_baudrate(port, baudrate, duration=3):
    """测试指定波特率"""
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
        time.sleep(0.5)
        
        data_bytes = []
        text_lines = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                # 读取原始字节
                raw = ser.read(ser.in_waiting)
                data_bytes.extend(raw)
                
                # 尝试解码为文本
                try:
                    text = raw.decode('utf-8', errors='ignore')
                    if text.strip():
                        text_lines.append(text.strip())
                except:
                    pass
        
        ser.close()
        
        return data_bytes, text_lines
        
    except Exception as e:
        return None, None

def analyze_data(data_bytes):
    """分析数据特征"""
    if not data_bytes:
        return {}
    
    # 统计字节分布
    printable = 0
    control = 0
    high_bytes = 0
    
    for b in data_bytes:
        if 32 <= b <= 126:  # 可打印ASCII
            printable += 1
        elif b < 32:  # 控制字符
            control += 1
        else:  # 高位字节
            high_bytes += 1
    
    total = len(data_bytes)
    
    return {
        'total_bytes': total,
        'printable_pct': printable / total * 100 if total > 0 else 0,
        'control_pct': control / total * 100 if total > 0 else 0,
        'high_bytes_pct': high_bytes / total * 100 if total > 0 else 0
    }

def find_patterns(data_bytes):
    """查找可能的数据包模式"""
    if len(data_bytes) < 20:
        return []
    
    # 查找重复的字节序列
    patterns = []
    for start_byte in [0x55, 0xAA, 0xFF, 0x00, 0x24, 0x23]:  # 常见起始字节
        positions = [i for i, b in enumerate(data_bytes) if b == start_byte]
        if len(positions) >= 3:
            # 计算间隔
            intervals = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
            # 检查是否有规律
            if intervals:
                avg_interval = sum(intervals) / len(intervals)
                if 10 <= avg_interval <= 100:  # 合理的包长度
                    patterns.append({
                        'start_byte': f'0x{start_byte:02X}',
                        'occurrences': len(positions),
                        'avg_interval': avg_interval
                    })
    
    return patterns

def main():
    port = '/dev/ttyUSB0'
    baudrates = [115200, 57600, 38400, 19200, 9600, 4800, 230400, 460800]
    
    print("=" * 70)
    print("IMU 数据分析 / IMU Data Analysis")
    print("=" * 70)
    
    results = []
    
    for baudrate in baudrates:
        print(f"\n测试波特率 / Testing baudrate: {baudrate}")
        
        data_bytes, text_lines = test_baudrate(port, baudrate, duration=3)
        
        if data_bytes is None:
            print(f"  ✗ 无法打开串口 / Cannot open port")
            continue
        
        if len(data_bytes) == 0:
            print(f"  ✗ 无数据 / No data received")
            continue
        
        analysis = analyze_data(data_bytes)
        patterns = find_patterns(data_bytes)
        
        print(f"  ✓ 接收到 {analysis['total_bytes']} 字节")
        print(f"    - 可打印字符: {analysis['printable_pct']:.1f}%")
        print(f"    - 控制字符: {analysis['control_pct']:.1f}%")
        print(f"    - 高位字节: {analysis['high_bytes_pct']:.1f}%")
        
        if patterns:
            print(f"    - 发现 {len(patterns)} 个可能的数据包模式:")
            for p in patterns:
                print(f"      起始字节 {p['start_byte']}: {p['occurrences']}次, 平均间隔 {p['avg_interval']:.1f} 字节")
        
        # 显示前100个字节的十六进制
        print(f"    - 前100字节 (十六进制):")
        hex_str = ' '.join([f'{b:02X}' for b in data_bytes[:100]])
        for i in range(0, len(hex_str), 72):
            print(f"      {hex_str[i:i+72]}")
        
        # 如果有文本行，显示
        if text_lines:
            print(f"    - 文本行样本 ({len(text_lines)} 行):")
            for line in text_lines[:3]:
                print(f"      {line[:80]}")
        
        results.append({
            'baudrate': baudrate,
            'analysis': analysis,
            'patterns': patterns,
            'has_text': len(text_lines) > 0
        })
    
    # 推荐最佳波特率
    print("\n" + "=" * 70)
    print("分析结果 / Analysis Results")
    print("=" * 70)
    
    if not results:
        print("未能从IMU读取任何数据")
        return
    
    # 找到数据量最大的
    best = max(results, key=lambda x: x['analysis']['total_bytes'])
    
    print(f"\n推荐波特率 / Recommended baudrate: {best['baudrate']}")
    print(f"数据特征 / Data characteristics:")
    print(f"  - 总字节数: {best['analysis']['total_bytes']}")
    print(f"  - 可打印字符占比: {best['analysis']['printable_pct']:.1f}%")
    
    if best['analysis']['printable_pct'] > 70:
        print(f"\n✓ 数据主要是ASCII文本格式")
        print(f"  建议使用文本解析")
    elif best['patterns']:
        print(f"\n✓ 数据可能是二进制包格式")
        print(f"  发现了规律性的数据包结构")
        print(f"  建议使用二进制协议解析")
    else:
        print(f"\n! 数据格式不明确")
        print(f"  请查阅IMU的通信协议文档")
    
    print("\n建议下一步:")
    print("1. 查看IMU型号和数据手册")
    print("2. 确认IMU的通信协议 (文本/二进制)")
    print("3. 如果是特定传感器(如MPU6050, BNO055等)，使用对应的库")

if __name__ == "__main__":
    main()
