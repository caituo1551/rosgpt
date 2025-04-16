#!/usr/bin/env python3
# ROSGPT AMR Navigator - 使用 /pose_nav 控制 cmd_vel 並支援 move_base

import rospy
import actionlib
import math
import json
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

class AMRNavigator:
    def __init__(self):
        rospy.init_node('amr_navigator', anonymous=True)
        self.pose_nav = None

        rospy.sleep(1.0)  # 🔧 確保 ROS network 初始化完成

        rospy.Subscriber('/voice_cmd', String, self.voice_cmd_callback)
        rospy.Subscriber('/pose_nav', Odometry, self.pose_nav_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Timer(rospy.Duration(1.0), self.check_move_base_connection, oneshot=True)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def check_move_base_connection(self, event):
        if self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.loginfo("✅ 成功連接 move_base action server！")
        else:
            rospy.logwarn("⚠️ 無法連接 move_base action server！請確認是否已啟動")

    def pose_nav_callback(self, msg):
        rospy.loginfo("[DEBUG] pose_nav_callback() 被呼叫")
        self.pose_nav = msg
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        rospy.loginfo(f"[pose_nav] x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}, yaw: {math.degrees(yaw):.1f}°")

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
                    self.goto_target(params.get('x', 0.0), params.get('y', 0.0), params.get('yaw', 0.0))
                elif action in ['move', 'rotate']:
                    self.cmd_vel_control(action, params)
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
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"前往目標點：x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("導航任務完成！")

    def get_distance(self, p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def cmd_vel_control(self, action, params):
        twist = Twist()
        rate = rospy.Rate(10)

        if self.pose_nav is None:
            rospy.logwarn("尚未收到任何 /pose_nav 資料，請確認訂閱是否生效")
            return

        start_pos = copy.deepcopy(self.pose_nav.pose.pose.position)

        if action == 'move':
            linear_speed = params.get('linear_speed', 0.3)
            distance = params.get('distance', 1.0)
            is_forward = params.get('is_forward', True)
            twist.linear.x = linear_speed if is_forward else -linear_speed
            rospy.loginfo(f"執行 move：目標距離 {distance} 公尺")

            while not rospy.is_shutdown():
                current_pos = self.pose_nav.pose.pose.position
                if self.get_distance(start_pos, current_pos) >= distance:
                    break
                self.vel_pub.publish(twist)
                rate.sleep()

        elif action == 'rotate':
            angular_velocity = params.get('angular_velocity', 0.8)
            angle = params.get('angle', 90)
            is_clockwise = params.get('is_clockwise', True)
            angle_rad = math.radians(angle)
            q = self.pose_nav.pose.pose.orientation
            _, _, start_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            twist.angular.z = -angular_velocity if is_clockwise else angular_velocity
            rospy.loginfo(f"執行 rotate：目標角度 {angle} 度")

            while not rospy.is_shutdown():
                q_now = self.pose_nav.pose.pose.orientation
                _, _, current_yaw = euler_from_quaternion([q_now.x, q_now.y, q_now.z, q_now.w])
                angle_diff = abs(self.normalize_angle(current_yaw - start_yaw))
                if angle_diff >= angle_rad:
                    break
                self.vel_pub.publish(twist)
                rate.sleep()

        self.vel_pub.publish(Twist())
        rospy.loginfo("動作完成，AMR 停止")

if __name__ == '__main__':
    try:
        nav = AMRNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
