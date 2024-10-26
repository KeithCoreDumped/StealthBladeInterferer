#!/usr/bin/env python3

import rospy
import math
import tf
import serial  # 用于串口通信
import actionlib  # 用于取消和重新发布导航目标
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import threading  # 用于并行处理串口监听

from poses import GOALS

class WaypointPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("waypoint_publisher")
        
        # 标记导航是否暂停
        self.paused = False
        # 路径点索引
        self.current_goal_index = 0
        # 路径点的距离阈值
        self.goal_tolerance = 1.5
        # 机器人当前位置
        self.current_position = None
        # 当前目标的状态
        self.current_goal_status = None
        
        self.xiaoba_done = False

        # 发布目标点的Publisher
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10, latch=True)

        # 订阅里程计信息，获取当前位置
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)

        # 订阅 /move_base/status 话题以获取当前目标状态
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

        # 创建tf监听器
        self.tf_listener = tf.TransformListener()

        # 创建 move_base 的 action 客户端，用于取消目标点
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()  # 等待 action 服务器就绪

        # 串口监听线程
        self.serial_thread = threading.Thread(target=self.serial_listener)
        self.serial_thread.daemon = True  # 守护线程，主程序退出时线程自动结束
        self.serial_thread.start()
        
        # 开始发布第一个路径点
        self.publish_goal()

    def serial_listener(self):
        # 打开串口
        ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
        rospy.loginfo("Listening on /dev/ttyAMA1 for messages")

        while not rospy.is_shutdown():
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                rospy.loginfo(f"Received from serial: {line}")

                # 如果接收到的是特定消息，则暂停导航，执行do_somework
                if line.startswith("xiaoba") and not self.xiaoba_done:
                    # 暂停导航并取消当前目标点
                    self.cancel_navigation()
                    rospy.logwarn("Navigation paused due to 'xiaoba' detection")
                    
                    # 执行任务
                    self.do_somework()

                    # 恢复导航
                    rospy.logwarn("Resuming navigation")
                    self.paused = False
                    self.xiaoba_done = True
                    self.publish_goal()  # 重新发布取消前的目标点

    def cancel_navigation(self):
        # 设置暂停标志位
        self.paused = True

        # 取消当前 move_base 的目标点
        self.move_base_client.cancel_all_goals()
        rospy.loginfo("Current goal canceled")

    def do_somework(self):
        # 这个函数中添加需要执行的任务逻辑
        rospy.loginfo("正在执行干扰...")
        rospy.sleep(2)  # 模拟一些耗时操作

    def status_callback(self, msg):
        # 检查是否有有效的目标状态
        if len(msg.status_list) > 0:
            # 获取最新的目标状态（假设我们只关心最新的目标）
            self.current_goal_status = msg.status_list[-1].status
            #rospy.loginfo(f"Current goal status: {self.current_goal_status}")
        else:
            self.current_goal_status = None

    def odom_callback(self, msg):
        if self.paused:
            return  # 如果暂停，则跳过里程计回调的处理

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
            if self.current_position is not None:
                if self.check_goal_reached():
                    rospy.loginfo(f"Goal {self.current_goal_index} reached")
                    # 如果到达当前路径点，发布下一个路径点
                    self.current_goal_index += 1
                    if self.current_goal_index < len(GOALS):
                        rospy.sleep(1.0)
                        self.publish_goal()
                    else:
                        rospy.logwarn("All goals have been reached, exiting")
                        rospy.signal_shutdown("All goals have been reached")
                else:
                    if self.current_goal_status == GoalStatus.SUCCEEDED:
                        rospy.sleep(1.0)
                        rospy.logwarn("Incorrect state, republishing...")
                        self.publish_goal()
                        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed")

    def publish_goal(self):
        if self.paused:
            return  # 如果暂停，则不发布新的目标点

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
        rospy.sleep(1.0)

    def check_goal_reached(self):
        # 获取当前路径点的位置
        goal_position = GOALS[self.current_goal_index].position

        # 计算当前位置和目标位置的距离
        distance = math.sqrt(
            (self.current_position.x - goal_position.x) ** 2 +
            (self.current_position.y - goal_position.y) ** 2
        )
        
        return distance < self.goal_tolerance

if __name__ == "__main__":
    try:
        WaypointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
