#!/usr/bin/env python
from donkeycar.parts import pins
import numpy as np, time, rospy, math, atexit
from select import select
from geometry_msgs.msg import Twist

### configs
forward_range = [399, 430]
backward_range = [310, 357]
throttle_neutral = 387

steering_neutral = 390
left_range = [steering_neutral, 440]
right_range = [330, steering_neutral]
###

throttle_pin = pins.pwm_pin_by_id("PCA9685.1:40.1")
throttle_pin.start()

steering_pin = pins.pwm_pin_by_id("PCA9685.1:40.0")
steering_pin.start()

def vel_to_pwm(vel: float):
    # apply velocity limit
    if vel != 0.0 and abs(vel) < 0.10:
        vel = math.copysign(0.10, vel)
    MAX_VEL = 0.40
    vel = np.interp(vel, [-MAX_VEL, MAX_VEL], [-MAX_VEL, MAX_VEL])
    if vel < 0:
        return np.interp(vel, [-1, 0], backward_range)
    elif vel > 0:
        return np.interp(vel, [0, 1], forward_range)
    else:
        return throttle_neutral

def deg_to_pwm(deg: float):
    if deg < 0:
        return np.interp(deg, [-27, 0], right_range)
    elif deg > 0:
        return np.interp(deg, [0, 20], left_range)
    else:
        return steering_neutral

def set_vel(vel: float):
    pulse = vel_to_pwm(vel)
    print("pulse:", pulse)
    throttle_pin.duty_cycle(pulse / 4096)
    
def set_deg(deg: float):
    pulse = deg_to_pwm(deg)
    # print(pulse)
    steering_pin.duty_cycle(pulse / 4096)

class MotorController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("motor_controller_node")

        # 订阅/cmd_vel主题
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.deg = 0
        self.v = 0

        rospy.spin()

    def cmd_vel_callback(self, twist_msg: Twist):
        v = twist_msg.linear.x
        omega = twist_msg.angular.z
        
        # print(v,omega)

        l = 0.175  # unit: meters
        if v != 0:
            self.deg = math.degrees(math.atan(omega * l / v))
        # else unchanged

        set_deg(self.deg)
        
        if v * self.v <= 0:
            # opposite direction
            set_vel(v)
            time.sleep(0.05)
            set_vel(0)
            time.sleep(0.05)
            self.v = v
            # rospy.loginfo("opposite dircetion")
        else:
            self.v = v

        set_vel(self.v)
        rospy.loginfo(f"deg={self.deg} v={v}")

@atexit.register
def cleanup():
    set_deg(0)
    set_vel(0)
    print("exiting: setting pwm to 0")

if __name__ == "__main__":
    try:
        MotorController()
    except rospy.ROSInterruptException:
        pass
