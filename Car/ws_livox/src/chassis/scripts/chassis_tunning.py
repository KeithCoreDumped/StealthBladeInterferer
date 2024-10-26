#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import signal
import sys

class VelocityMonitor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('velocity_monitor', anonymous=True)

        # Initialize variables to store maximum and minimum values
        self.cmd_vel_max = -float('inf')
        self.cmd_vel_min = float('inf')
        self.maesured_vel_max = -float('inf')
        self.maesured_vel_min = float('inf')

        # Subscribe to the /cmd_vel and /maesured_velocity topics
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/maesured_velocity', Float32, self.maesured_vel_callback)

        # Register a signal handler to handle Ctrl+C (SIGINT)
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.spin()

    def cmd_vel_callback(self, msg):
        # Update max and min values for cmd_vel linear.x
        self.cmd_vel_max = max(self.cmd_vel_max, msg.linear.x)
        self.cmd_vel_min = min(self.cmd_vel_min, msg.linear.x)

    def maesured_vel_callback(self, msg):
        # Update max and min values for maesured_velocity
        self.maesured_vel_max = max(self.maesured_vel_max, msg.data)
        self.maesured_vel_min = min(self.maesured_vel_min, msg.data)

    def signal_handler(self, sig, frame):
        # Output the maximum and minimum values when Ctrl+C is pressed
        print("\nReceived Ctrl+C, stopping...")
        print(f"command\tmaesured")
        print(f"{self.cmd_vel_max},{self.maesured_vel_max}")
        print(f"{self.cmd_vel_min},{self.maesured_vel_min}")
        sys.exit(0)

if __name__ == '__main__':
    try:
        monitor = VelocityMonitor()
    except rospy.ROSInterruptException:
        pass
