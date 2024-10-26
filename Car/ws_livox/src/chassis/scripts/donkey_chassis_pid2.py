#!/usr/bin/env python
from donkeycar.parts import pins
import numpy as np, time, rospy, math, tf
from select import select
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32  
from simple_pid import PID
# using dynamic reconfig from https://github.com/pal-robotics/ddynamic_reconfigure_python/tree/master
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

### configs
forward_range = [399, 430]
backward_range = [300, 354]
throttle_neutral = 380

left_range = [400, 475]
right_range = [320, 400]
steering_neutral = 400
###

def apply_recursive_filter(new_val: float, last_val: float, alpha=0.1):
    # `alpha` controls the weight of the new value in the filter.
    # Higher `alpha` means faster response, less smoothing.
    # Lower `alpha` means slower response, more smoothing.
    last_val = alpha * new_val + (1 - alpha) * last_val
    return last_val
import rospy
import math
import numpy as np
import time
from simple_pid import PID
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class MotorController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("motor_controller_node")

        self.target_velocity = 0  # 记录目标速度
        self.last_angle = 0
        self.maesured_velocity = 0
        self.last_velocity_ll = 0
        self.last_output = 0

        self.throttle_pin = pins.pwm_pin_by_id("PCA9685.1:40.1")
        self.throttle_pin.start()

        self.steering_pin = pins.pwm_pin_by_id("PCA9685.1:40.0")
        self.steering_pin.start()

        # 初始化PID控制器
        self.vpid = PID(Kp=0.1, Ki=2.0, Kd=0, setpoint=0, sample_time=None, output_limits=(-1,2), proportional_on_measurement=False)
        self.vpid.set_auto_mode(False, 0.0)

        ddynrec = DDynamicReconfigure("donkey_chassis_pid")
        ddynrec.add_variable("kp", "kp", 0.1, 0.0, 10.0)
        ddynrec.add_variable("ki", "ki", 2.0, 0.0, 10.0)
        ddynrec.add_variable("kd", "kd", 0.0, 0.0, 10.0)
        ddynrec.start(self.reconfigure_callback)

        # Subscribe to topics after everything initialized
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self.odometry_callback)
        self.velocity_pub = rospy.Publisher("/maesured_velocity", Float32, queue_size=10)

        rospy.spin()

    def reconfigure_callback(self, config, level):
        # 动态更新PID参数
        self.vpid.Kp = config.kp
        self.vpid.Ki = config.ki
        self.vpid.Kd = config.kd
        rospy.loginfo(f"Reconfigure Request: Kp={config.kp}, Ki={config.ki}, Kd={config.kd}")
        return config

    def set_vel_ll(self, vel: float):
        if vel < 0:
            pulse = np.interp(vel, [-1 * 3, 0], backward_range)
        elif vel > 0:
            pulse = np.interp(vel, [0, 5], forward_range)
        else:
            pulse = throttle_neutral
        if vel * self.last_velocity_ll <= 0 and vel < 0 and abs(vel - self.last_velocity_ll) > 0.1:
            self.throttle_pin.duty_cycle(pulse / 4096)
            time.sleep(0.05)
            self.throttle_pin.duty_cycle(throttle_neutral / 4096)
            time.sleep(0.05)
            rospy.loginfo(f"opposite direction with v={vel}, lastv={self.last_velocity_ll}")
        print("pulse: ", pulse)
        self.throttle_pin.duty_cycle(pulse / 4096)
        self.last_velocity_ll = vel

    def set_deg(self, deg: float):
        if deg < 0:
            pulse = np.interp(deg, [-17, 0], right_range)
        elif deg > 0:
            pulse = np.interp(deg, [0, 17], left_range)
        else:
            pulse = steering_neutral
        # print(pulse)
        self.steering_pin.duty_cycle(pulse / 4096)

    def set_vel_pid(self, vel: float):
        if vel != 0:
            self.vpid.setpoint = vel
            output = self.vpid(self.maesured_velocity)
            outputs = f"{output:.3f}" if output else "None"
            rospy.loginfo(f"vel:{vel:.3f}, out:{outputs}, int:{self.vpid._integral:.3f}, vel_ll: {output + vel:.3f}")
        else:
            output = self.vpid(self.maesured_velocity)
        self.set_vel_ll(vel + output)

    def cmd_vel_callback(self, twist_msg: Twist):
        self.target_velocity = twist_msg.linear.x
        omega = twist_msg.angular.z

        l = 0.175  # wheelbase, in meters
        if self.target_velocity != 0:
            self.last_angle = math.degrees(math.atan(omega * l / self.target_velocity))

        self.set_deg(self.last_angle)

    def update_velocity(self, event):
        # 以10Hz频率调用set_vel_pid
        self.set_vel_pid(self.target_velocity)

    def odometry_callback(self, msg):
        if not self.vpid.auto_mode:
            rospy.logwarn("enabling pid...")
            # 设置一个定时器，以10Hz频率调用set_vel_pid函数
            rospy.Timer(rospy.Duration(0.1), self.update_velocity)
            self.vpid.set_auto_mode(True, 0.0)

        # 获取世界坐标系下的线速度分量
        linear_velocity_x = msg.twist.twist.linear.x
        linear_velocity_y = msg.twist.twist.linear.y

        # 获取小车的位姿（orientation），用于进行tf变换
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

        # 将四元数转换为欧拉角，以获取yaw角度（绕z轴的旋转）
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # 构造旋转矩阵，将世界坐标系下的速度转换到车体坐标系下
        rotation_matrix = np.array(
            [[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]
        )

        # 世界坐标系下的速度向量
        velocity_world = np.array([linear_velocity_x, linear_velocity_y])

        # 车体坐标系下的速度向量
        velocity_body = np.dot(rotation_matrix, velocity_world)

        # 车体坐标系下y轴方向的速度，即为前进/后退速度
        self.maesured_velocity = apply_recursive_filter(-velocity_body[1], self.maesured_velocity, 0.7)
        self.velocity_pub.publish(self.maesured_velocity)

    def __del__(self):
        self.set_deg(0)
        self.set_vel_ll(0)
        print("exiting: setting pwm to 0")


if __name__ == "__main__":
    try:
        MotorController()
    except rospy.ROSInterruptException:
        pass
