#!/usr/bin/env python3
import sys
import time
import threading
from datetime import datetime
import os


sys.path.append('./my_lidar/lidar_src')
from sdk_lib import sdk_py

def get_version_info():
    """获取SDK版本信息"""
    try:
        version = sdk_py.get_version()
        print(f"SDK版本: {version}")
        return True
    except Exception as e:
        print(f"获取SDK版本失败: {e}")
        return False

def initialize_device(com_id=0, baudrate=115200):
    """初始化设备"""
    try:
        # 创建设备对象
        device = sdk_py.Dev()
        
        # 构造串口路径
        if sys.platform.startswith('win'):
            port_path = f"//./com{com_id}"
        else:
            port_path = f"/dev/ttyUSB{com_id}"
        
        # 初始化设备
        result = device.Initialize(port_path, baudrate, True)
        if result != 0:
            print("设备初始化失败")
            return None
        
        print(f"设备初始化成功，端口: {port_path}")
        return device
    except Exception as e:
        print(f"设备初始化异常: {e}")
        return None

def get_scan_data_callback(device, is_reverse=True):
    """回调模式获取扫描数据"""
    try:
        # 设置回调函数处理点云数据
        def point_cloud_callback(point_list):
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 接收到点云数据，点数: {len(point_list)}")
            
            # 保存到文件
            with open("pointcloud_data.csv", "a") as f:
                for point in point_list:
                    line = f"{point.timestamp_ns},{point.angle:.3f},{point.angle_raw:.3f},{point.distance},{point.is_valid},{point.speed},{point.gray}\n"
                    f.write(line)
        
        # 注册回调函数；如果 SDK 未提供回调接口，则用轮询线程模拟回调
        if hasattr(device, 'set_point_cloud_callback'):
            try:
                device.set_point_cloud_callback(point_cloud_callback)
                print("开始接收扫描数据（回调模式 via SDK 回调）...")
                input("按回车键停止...")
            except Exception as e:
                print(f"注册SDK回调失败: {e}")
        else:
            stop_event = threading.Event()

            def poll_loop():
                while not stop_event.is_set():
                    try:
                        data_list = device.GetScanData(is_reverse)
                        if data_list:
                            point_cloud_callback(data_list)
                        time.sleep(0.001)
                    except Exception:
                        time.sleep(0.01)
    
            t = threading.Thread(target=poll_loop, daemon=True)
            t.start()
            print("开始接收扫描数据（回调模式 via 轮询线程）...")
            input("按回车键停止...")
            stop_event.set()
            t.join(timeout=1)
        
    except Exception as e:
        print(f"回调模式数据获取失败: {e}")

def get_scan_data_polling(device, is_reverse=True):
    """轮询模式获取扫描数据"""
    try:
        print("开始接收扫描数据（轮询模式）...")
        print("按Ctrl+C停止...")
        
        valid_count = 0
        invalid_count = 0
        total_points = 0
        
        with open("scan_data.csv", "w") as f:
            f.write("angle,distance,is_valid,quality\n")
            
            while True:
                try:
                    # 获取扫描数据（从Python绑定中接收返回的列表）
                    data_list = device.GetScanData(is_reverse)

                    if not data_list:
                        time.sleep(0.01)  # 10ms延迟
                        continue
                    
                    # 处理数据
                    batch_valid = 0
                    batch_invalid = 0
                    
                    for point in data_list:
                        # 写入CSV文件
                        line = f"{point.angle_q6_checkbit/64.0:.4f},{point.distance_q2/4},{point.is_valid},{point.syn_quality}\n"
                        f.write(line)
                        
                        # 统计有效/无效点
                        if point.is_valid == 1:
                            batch_valid += 1
                        else:
                            batch_invalid += 1
                    
                    valid_count += batch_valid
                    invalid_count += batch_invalid
                    total_points += len(data_list)
                    
                    # 打印统计信息
                    timestamp = int(time.time())
                    print(f"ts={timestamp}, 数据量={len(data_list)}, 有效={batch_valid}, 无效={batch_invalid}")
                    
                    f.write("\n")  # 每批次后空行分隔
                    f.flush()  # 立即写入磁盘
                    
                    time.sleep(0.001)  # 1ms延迟
                    
                except KeyboardInterrupt:
                    print("\n用户中断程序")
                    break
                except Exception as e:
                    print(f"数据获取错误: {e}")
                    error_code = device.GetLastErrCode()
                    if error_code != 0:  # 假设0表示成功
                        print(f"错误代码: {error_code}")
                    time.sleep(0.01)
        
        print(f"\n统计信息:")
        print(f"总点数: {total_points}")
        print(f"有效点: {valid_count}")
        print(f"无效点: {invalid_count}")
        print(f"有效率: {valid_count/total_points*100:.2f}%")
        
    except Exception as e:
        print(f"轮询模式数据获取失败: {e}")

def main():
    """主函数"""
    print("=== 激光雷达扫描数据采集程序 ===")
    
    # 检查SDK版本
    if not get_version_info():
        return
    
    # 显示可用类
    print("可用的类:", [attr for attr in dir(sdk_py) if not attr.startswith('_')])
    
    # 初始化设备
    com_id = 0  # 默认使用ttyUSB0
    device = initialize_device(com_id)
    if not device:
        return
    
    try:
        # 选择运行模式
        print("\n请选择运行模式:")
        print("1. 回调模式")
        print("2. 轮询模式")
        
        choice = input("请输入选择 (1/2): ").strip()
        
        if choice == "1":
            get_scan_data_callback(device)
        elif choice == "2":
            get_scan_data_polling(device)
        else:
            print("无效选择，使用默认轮询模式")
            get_scan_data_polling(device)
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序执行出错: {e}")
    finally:
        # 清理资源
        try:
            device.Uninit()
            print("设备已关闭")
        except:
            pass

if __name__ == "__main__":
    main()