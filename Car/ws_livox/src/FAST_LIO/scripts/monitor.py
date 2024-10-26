#!/usr/bin/python3
import psutil
import time
import csv
from datetime import datetime

# 监测间隔时间（秒）
interval = 5

# 要监测的进程名
process_name = "fastlio_mapping"

# 输出 CSV 文件名
output_file = f"process_monitor@{datetime.now().strftime('%Y.%m.%d.%H.%M.%S')}.csv"

def get_process_info(name):
    """返回进程名为 name 的进程的 CPU 和内存占用情况"""
    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_info']):
        if proc.info['name'] == name:
            mem_info = proc.info['memory_info'].rss / (1024 * 1024)  # 将内存从字节转换为MB
            return {
                'pid': proc.info['pid'],
                'cpu_percent': proc.info['cpu_percent'],
                'memory_mb': mem_info
            }
    return None

def monitor_process(interval, output_file):
    """监测进程 CPU 和内存占用情况，并将数据记录到 CSV 文件中"""
    start_time = time.time()  # 获取脚本启动时的时间戳
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Elapsed Time (s)', 'PID', 'CPU Percent', 'Memory (MB)'])

        while True:
            process_info = get_process_info(process_name)
            if process_info:
                elapsed_time = round(time.time() - start_time, 2)  # 计算自脚本启动以来的秒数
                writer.writerow([elapsed_time, process_info['pid'], process_info['cpu_percent'], process_info['memory_mb']])
                print(f"{elapsed_time} seconds - PID: {process_info['pid']}, CPU: {process_info['cpu_percent']}%, Memory: {process_info['memory_mb']} MB")
            else:
                print(f"{round(time.time() - start_time, 2)} seconds - Process '{process_name}' not found")

            time.sleep(interval)

if __name__ == "__main__":
    monitor_process(interval, output_file)
