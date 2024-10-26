#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from collections import deque

# 全局变量存储上一次的位姿和时间
last_position = None
last_time = None
last_yaw = None

# 创建一个发布器
velocity_publisher = None

# 滤波窗口大小
WINDOW_SIZE = 5

# 存储过去的线速度分量和角速度
vx_buffer = deque(maxlen=WINDOW_SIZE)
vy_buffer = deque(maxlen=WINDOW_SIZE)
vz_buffer = deque(maxlen=WINDOW_SIZE)
angular_velocity_buffer = deque(maxlen=WINDOW_SIZE)

def quaternion_to_yaw(orientation):
    """将四元数转换为偏航角（yaw）"""
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def apply_moving_average(buffer, new_value):
    """应用移动平均滤波器"""
    buffer.append(new_value)
    return sum(buffer) / len(buffer)

def odometry_callback(msg):
    global last_position, last_time, last_yaw

    # 提取当前位置和姿态
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    current_time = rospy.Time.now().to_sec()

    # 如果这是第一次接收到消息，直接保存并返回
    if last_position is None:
        last_position = position
        last_time = current_time
        last_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        return

    # 计算时间差
    dt = current_time - last_time  # typically 0.1s
    if dt == 0:
        return

    # 计算线速度的三个分量
    vx = (position.x - last_position.x) / dt
    vy = (position.y - last_position.y) / dt
    vz = (position.z - last_position.z) / dt

    # 对每个线速度分量应用移动平均滤波器
    vx_filtered = apply_moving_average(vx_buffer, vx)
    vy_filtered = apply_moving_average(vy_buffer, vy)
    vz_filtered = apply_moving_average(vz_buffer, vz)

    # 使用滤波后的线速度分量计算线速度的模
    linear_velocity_magnitude = math.sqrt(vx_filtered**2 + vy_filtered**2 + vz_filtered**2)

    # 计算角速度
    current_yaw = quaternion_to_yaw(orientation)
    angular_velocity_z = (current_yaw - last_yaw) / dt

    # 对角速度应用滤波
    angular_velocity_z_filtered = apply_moving_average(angular_velocity_buffer, angular_velocity_z)

    # 创建Twist消息
    velocity_msg = Twist()
    velocity_msg.linear.x = vx_filtered  # 线速度模存放在linear.x
    velocity_msg.linear.y = vy_filtered  # 滤波后的线速度分量
    velocity_msg.linear.z = vz_filtered
    velocity_msg.angular.z = angular_velocity_z_filtered

    # 发布速度消息
    velocity_publisher.publish(velocity_msg)

    # 更新上一次的位姿和时间
    last_position = position
    last_time = current_time
    last_yaw = current_yaw

def odometry_listener():
    global velocity_publisher
    rospy.init_node('odometry_listener', anonymous=True)

    # 创建一个发布器，发布到 /calculated_velocity 话题，消息类型为 geometry_msgs/Twist
    velocity_publisher = rospy.Publisher('/calculated_velocity', Twist, queue_size=10)

    # 订阅 /Odometry 话题
    rospy.Subscriber("/Odometry", Odometry, odometry_callback)

    rospy.spin()

if __name__ == '__main__':
    odometry_listener()
