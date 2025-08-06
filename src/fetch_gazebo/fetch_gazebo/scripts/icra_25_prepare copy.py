#!/usr/bin/env python

import sys

import rospy
import actionlib
import numpy as np
import pandas as pd
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
'''
arm_intermediate_positions  = [0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0]

arm_joint_positions  = [0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0]
#Step 1
arm_joint_positions_1  = [0.0, 0, 0.0, 1.0, 0.0, -0.9, 1.57]

#Step 2

arm_joint_positions_2  = [0.4, -0.0, -1.7, 1.4, 0.0, -0.7, 0]


#STep 3
arm_joint_positions_3  = [0.4, 0.4, -1.7, 1.4, 0.5, -0.7, 0]



#Step 4
arm_joint_positions_4  = [0.0, 0.8, 0.0, -1.7, 0.0, 1.0, 1.57]
'''
#Step Gazebp
arm_joint_positions  = [0.0, 0.55, 0.0, -1.45, 0.0, 1.0, 1.57]
# arm_joint_positions = [0.1, 0.55, 0.2, -1.45, 0.0, 1.0, 1.45]

# po1
waypoint_PO_1 = np.array([0.1, 0.55, 0.2, -1.45, 0.0, 1, 1.45 ])
# waypoint_PO_1 = np.array(arm_joint_positions) + np.array([0.1, 0, 0.2, 0, 0, 0, -0.12])
waypoint_PO_1 = waypoint_PO_1.tolist()

#po2


waypoint_PO_2 = np.array([-0.15071344375610352, 0.5756263732910156, -0.2899223864078522, -1.4614996910095215, -0.12962138652801514, 0.9560532569885254, 1.876058578491211])
# waypoint_PO_2 = np.array([0.1, 0.7, 0.2, -1.60, 0.0, 1, 1.45])
# waypoint_PO_2 = np.array(waypoint_PO_1) + np.array([0, 0.15, 0, -0.15, 0, 0, 0])
waypoint_PO_2 = waypoint_PO_2.tolist()

# #po3
# # waypoint_PO_3 = np.array([-0.05, 0.7, -0.05, -1.6, 0.0, 1, 1.53])
# waypoint_PO_3 = np.array(waypoint_PO_2) + np.array([-0.15, 0, -0.25, 0, 0, 0, 0.08])
# waypoint_PO_3 = waypoint_PO_3.tolist()

# #po4
waypoint_PO_3= np.array([-0.3781261444091797, 0.47745180130004883, -0.7363107800483704, -1.4411749839782715, -0.11543205380439758, 0.9207720756530762, 2.2825639247894287])
# waypoint_PO_4 = np.array(waypoint_PO_3) + np.array([-0.1, 0, -0.25, 0, -0.24, 0, 0.37])
waypoint_PO_3 = waypoint_PO_3.tolist()


#po3
waypoint_PO_4 = np.array([-0.4425535202026367, 0.373140811920166, -0.9096506237983704, -1.4285197257995605, -0.0517718531191349, 1.0070581436157227, 2.3999130725860596])
# waypoint_PO_4 = np.array(waypoint_PO_3) + np.array([-0.15, 0, -0.25, 0, 0, 0, 0.08])
waypoint_PO_4= waypoint_PO_4.tolist()


###############################################

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0073164403438568115, 0.42031073570251465]


if __name__ == "__main__":
    rospy.init_node("prepare_simulated_robot")

    # Check robot serial number, we never want to run this on a real robot!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        #sys.exit(-1)

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for gripper_controller...")
    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper_client.wait_for_server()
    rospy.loginfo("...connected.")

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)


    ts = 0
    j=0
    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names 
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[j].positions = arm_joint_positions
    trajectory.points[j].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[j].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[j].time_from_start = rospy.Duration(ts+1.7)
    
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[j+1].positions = waypoint_PO_1
    trajectory.points[j+1].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[j+1].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[j+1].time_from_start = rospy.Duration(ts+3.4)
    
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[j+2].positions = waypoint_PO_2
    trajectory.points[j+2].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[j+2].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[j+2].time_from_start = rospy.Duration(ts+5.1)
    
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[j+3].positions = waypoint_PO_3
    trajectory.points[j+3].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[j+3].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[j+3].time_from_start = rospy.Duration(ts+7.8)
    
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[j+4].positions = waypoint_PO_4
    trajectory.points[j+4].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[j+4].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[j+4].time_from_start = rospy.Duration(ts+10.5)


    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.01
    rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)
    arm_client.send_goal(arm_goal)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(6.0))
    head_client.wait_for_result(rospy.Duration(6.0))

    rospy.loginfo("...done")