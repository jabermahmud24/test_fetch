#!/usr/bin/env python3
"""
Navigate LoCoBot to a given (x, y) coordinate using move_base action server.
"""

import rospy
import actionlib
from math import sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys


class MoveBaseClientNS:
    def __init__(self, ns="/locobot", wait_sec=8.0):
        self.name = f"{ns}/move_base" if ns else "move_base"
        self.client = actionlib.SimpleActionClient(self.name, MoveBaseAction)
        rospy.loginfo(f"[LoCoBot] Waiting for action server {self.name} ...")
        ok = self.client.wait_for_server(rospy.Duration(wait_sec))
        if not ok:
            rospy.logerr(f"[LoCoBot] No action server at {self.name}. "
                         f"Ensure navigation stack (amcl/map_server/move_base) is running.")
            raise rospy.ROSException(f"move_base not available at {self.name}")

    def goto(self, x, y, yaw=0.0, frame="map", wait=True):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = cos(yaw / 2.0)
        self.client.send_goal(goal)
        if wait:
            self.client.wait_for_result()
            result = self.client.get_state()
            if result == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[LoCoBot] Successfully reached ({x}, {y})")
                return True
            else:
                rospy.logwarn(f"[LoCoBot] Failed to reach ({x}, {y})")
                return False
        return True


def navigate_to(x=0.0, y=0.0):
    rospy.init_node("locobot_navigate", anonymous=True)
    
    # Wait for the clock to ensure simulation time is available
    while rospy.Time.now().to_sec() == 0.0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    try:
        navigator = MoveBaseClientNS(ns="/locobot")
        rospy.loginfo(f"[LoCoBot] Navigating to ({x}, {y}) in map frame...")
        success = navigator.goto(x, y, yaw=0.0, frame="map", wait=True)
        if success:
            rospy.loginfo("[LoCoBot] Navigation completed successfully.")
        else:
            rospy.logwarn("[LoCoBot] Navigation failed.")
    except rospy.ROSException as e:
        rospy.logerr(f"[LoCoBot] Navigation error: {e}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[LoCoBot] Navigation interrupted.")


if __name__ == "__main__":
    try:
        # Get x, y coordinates from command-line arguments or use defaults
        if len(sys.argv) == 3:
            target_x = float(sys.argv[1])
            target_y = float(sys.argv[2])
        else:
            rospy.logwarn("No coordinates provided, using default (0, 0)")
            target_x, target_y = 0.0, 0.0
        navigate_to(target_x, target_y)
    except ValueError:
        rospy.logerr("Error: Please provide valid numeric coordinates (x, y).")
        sys.exit(1)