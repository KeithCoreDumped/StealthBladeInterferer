#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import atexit

def pythonize_goal(goal: Pose):
    return f"Pose(position=Point(x={goal.position.x}, y={goal.position.y}, z={goal.position.z}), " \
                f"orientation=Quaternion(x={goal.orientation.x}, y={goal.orientation.y}, " \
                f"z={goal.orientation.z}, w={goal.orientation.w})),\n"

def callback(goal: PoseStamped):
    print(pythonize_goal(goal.pose))

def listener():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
