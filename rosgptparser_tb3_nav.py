#!/usr/bin/env python3
# This file is part of rosgpt package.

# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.

# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rospy
import json
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Sample JSON database for locations
locations_json = """
[
    {"name": "kitchen", "x": 6.5, "y": 1.0, "theta": 0.0},
    {"name": "living_room", "x": 0.0, "y": 1.0, "theta": 0.0},
    {"name": "bedroom", "x": -3.6, "y": -1, "theta": 0.0}
]
"""

# Load the locations from JSON
locations = json.loads(locations_json)

class LocationNavigationNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('location_navigation_node', anonymous=True)

        # Subscriber to voice_cmd topic
        self.subscription = rospy.Subscriber(
            'voice_cmd', String, self.voice_cmd_callback, queue_size=10
        )

        # Action client for move_base
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Location Navigation Node is ready')

    def voice_cmd_callback(self, msg):
        # Convert the received message to lowercase for matching
        voice_command = msg.data.lower()

        # Match the command to a location
        for location in locations:
            if location["name"] in voice_command:
                self.send_navigation_goal(location)
                break
        else:
            rospy.logwarn("No location found in the received JSON command")

    def send_navigation_goal(self, location):
        """
        Sends a navigation goal to the move_base action server.

        :param location: A dictionary containing the location information (name, x, y, theta).
        """
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = location["x"]
        goal_msg.target_pose.pose.position.y = location["y"]

        # Set orientation (assuming quaternion format)
        theta = location["theta"]
        goal_msg.target_pose.pose.orientation.z = theta

        rospy.loginfo(f'Sending navigation goal to {location["name"]}')
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal_msg, done_cb=self.navigation_goal_done_callback)

    def navigation_goal_done_callback(self, status, result):
        """
        Callback function that is called when a navigation goal is done.

        :param status: Status of the goal.
        :param result: Result of the goal.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Goal failed with status: " + str(status))

    def read_keyboard_input(self):
        """
        Reads keyboard input from the user and sends a navigation goal accordingly.
        """
        while not rospy.is_shutdown():
            try:
                location_number = int(input("Enter a location number (1 - kitchen, 2 - living room, 3 - bedroom): "))
                if 1 <= location_number <= len(locations):
                    self.send_navigation_goal(locations[location_number - 1])
                else:
                    rospy.loginfo("Invalid location number. Please try again.")
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a number.")

def main():
    # Initialize node
    node = LocationNavigationNode()

    # Start a separate thread for keyboard input
    keyboard_input_thread = threading.Thread(target=node.read_keyboard_input)
    keyboard_input_thread.daemon = True
    keyboard_input_thread.start()

    # Keep node running
    rospy.spin()

if __name__ == '__main__':
    main()
