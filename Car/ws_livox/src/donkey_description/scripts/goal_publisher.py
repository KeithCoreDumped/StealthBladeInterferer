#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry

from poses import GOALS

class WaypointPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("waypoint_publisher")

        # 发布目标点的Publisher
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10, latch=True)

        # 订阅里程计信息，获取当前位置
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)

        # 创建tf监听器
        self.tf_listener = tf.TransformListener()

        # 路径点索引
        self.current_goal_index = 0

        # 路径点的距离阈值
        self.goal_tolerance = 1.0

        # 机器人当前位置
        self.current_position = None

        # 开始发布第一个路径点
        self.publish_goal()

    def odom_callback(self, msg):
        try:
            # 获取里程计坐标（camera_init frame），转换到map frame
            position_camera_init = msg.pose.pose.position

            # 等待从camera_init到map的变换
            self.tf_listener.waitForTransform('map', 'camera_init', rospy.Time(0), rospy.Duration(4.0))

            # 获取camera_init相对于map的变换
            (trans, rot) = self.tf_listener.lookupTransform('map', 'camera_init', rospy.Time(0))

            # 使用变换将位置从camera_init坐标系转换到map坐标系
            self.current_position = Point()
            self.current_position.x = position_camera_init.x + trans[0]
            self.current_position.y = position_camera_init.y + trans[1]
            self.current_position.z = position_camera_init.z + trans[2]

            # 检查是否到达当前路径点
            if self.current_position is not None and self.check_goal_reached():
                rospy.loginfo(f"Goal {self.current_goal_index} reached")
                # 如果到达当前路径点，发布下一个路径点
                self.current_goal_index += 1
                if self.current_goal_index < len(GOALS):
                    rospy.sleep(1.0)
                    self.publish_goal()
                    rospy.sleep(1.0)
                    self.publish_goal()
                else:
                    rospy.loginfo("All goals have been reached")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed")

    def publish_goal(self):
        # 获取下一个路径点
        goal_pose = GOALS[self.current_goal_index]
        
        # 创建PoseStamped消息
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = goal_pose
        
        # 发布目标
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Published goal {self.current_goal_index}")
        rospy.sleep(1.5)

    def check_goal_reached(self):
        # 获取当前路径点的位置
        goal_position = GOALS[self.current_goal_index].position

        # 计算当前位置和目标位置的距离
        distance = math.sqrt(
            (self.current_position.x - goal_position.x) ** 2 +
            (self.current_position.y - goal_position.y) ** 2
        )
        
        # rospy.loginfo(f"Distance to goal: {distance}")

        return distance < self.goal_tolerance

if __name__ == "__main__":
    try:
        WaypointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
