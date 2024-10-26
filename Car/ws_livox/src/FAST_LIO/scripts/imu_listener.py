#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import math
from collections import deque

# 滑动窗口的大小
WINDOW_SIZE = 100

# 存储加速度分量的滑动窗口
ax_buffer = deque(maxlen=WINDOW_SIZE)
ay_buffer = deque(maxlen=WINDOW_SIZE)
az_buffer = deque(maxlen=WINDOW_SIZE)

def apply_moving_average(buffer, new_value):
    buffer.append(new_value)
    return sum(buffer) / len(buffer)

def magnitude(x,y,z):
    return math.sqrt(x*x+y*y+z*z)

def imu_callback(msg):
    # 提取加速度分量
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z

    # 对加速度分量进行均值滤波
    ax_filtered = apply_moving_average(ax_buffer, ax)
    ay_filtered = apply_moving_average(ay_buffer, ay)
    az_filtered = apply_moving_average(az_buffer, az)

    # 计算滤波后的俯仰角（pitch）和横滚角（roll）
    pitch = math.asin(-ax_filtered / magnitude(ax, ay, az))  # 以g = 9.81 m/s²为基准
    roll = math.atan2(ay_filtered, az_filtered)

    # 输出结果
    rospy.loginfo(f"Pitch: {math.degrees(pitch):.2f}°, Roll: {math.degrees(roll):.2f}°")

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)

    # 订阅 /livox/imu 话题
    rospy.Subscriber("/livox/imu", Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    imu_listener()
