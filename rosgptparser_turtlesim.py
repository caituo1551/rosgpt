#!/usr/bin/env python3
# 用於實體 AMR，控制 move_base 並訂閱 hdl_localization 提供的 /nav_pose

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import json
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class AMRNavigator:
    def __init__(self):
        rospy.init_node('amr_navigator', anonymous=True)

        self.nav_pose = None
        rospy.Subscriber('/voice_cmd', String, self.voice_cmd_callback)
        rospy.Subscriber('/nav_pose', PoseStamped, self.nav_pose_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待連接 move_base action server ...")
        self.client.wait_for_server()
        rospy.loginfo("已連接 move_base！")

    def nav_pose_callback(self, msg):
        self.nav_pose = msg

    def voice_cmd_callback(self, msg):
        try:
            commands = json.loads(msg.data)
            if isinstance(commands, dict):
                commands = [commands]

            rospy.loginfo(f"接收到指令：{commands}")

            for command in commands:
                action = command.get('action')
                params = command.get('params', {})

                if action == 'goto':
                    self.goto_target(
                        x=params.get('x', 0.0),
                        y=params.get('y', 0.0),
                        yaw=params.get('yaw', 0.0)
                    )

                elif action == 'move':
                    self.move_forward_distance(
                        distance=params.get('distance', 1.0),
                        is_forward=params.get('is_forward', True)
                    )

                elif action == 'rotate':
                    self.rotate_in_place(
                        angle=params.get('angle', 90),
                        is_clockwise=params.get('is_clockwise', True)
                    )

                else:
                    rospy.logwarn(f"未知的 action: {action}")

        except Exception as e:
            rospy.logerr(f"解析或執行指令時發生錯誤：{e}")

    def goto_target(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"前往目標點：x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("導航任務完成！")

    def move_forward_distance(self, distance, is_forward=True):
        if self.nav_pose is None:
            rospy.logwarn("尚未收到 /nav_pose，無法執行 move")
            return

        direction = 1.0 if is_forward else -1.0
        current_x = self.nav_pose.pose.position.x
        current_y = self.nav_pose.pose.position.y

        q = self.nav_pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 根據方向與角度推算目標座標
        target_x = current_x + direction * distance * math.cos(yaw)
        target_y = current_y + direction * distance * math.sin(yaw)

        rospy.loginfo(f"移動距離 {distance} 公尺，目標座標為 x={target_x}, y={target_y}")
        self.goto_target(target_x, target_y, yaw)

    def rotate_in_place(self, angle, is_clockwise=True):
        if self.nav_pose is None:
            rospy.logwarn("尚未收到 /nav_pose，無法執行 rotate")
            return

        q = self.nav_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        angle_rad = math.radians(angle)
        if is_clockwise:
            target_yaw = current_yaw - angle_rad
        else:
            target_yaw = current_yaw + angle_rad

        rospy.loginfo(f"原地旋轉 {angle} 度（{'順時針' if is_clockwise else '逆時針'}），目標 yaw={target_yaw}")
        self.goto_target(self.nav_pose.pose.position.x, self.nav_pose.pose.position.y, target_yaw)


if __name__ == '__main__':
    try:
        nav = AMRNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
