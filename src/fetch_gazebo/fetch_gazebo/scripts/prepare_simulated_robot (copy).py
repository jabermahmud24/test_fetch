#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import sys
# from gazebo_msgs.msg import LinkStates

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
#               "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


# arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
# arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

# arm_intermediate_positions  = [-0.28, 0.58, -0.5, -1.60,-0.24, 0.8, 2.1] 
arm_intermediate_positions  = [-0.55, 0.5, -1.0, -1.60, -0.4, 0.65, 2.9] 
# arm_joint_positions  = [0, 0, 0, 0, 0, 0, 0]


# arm_intermediate_positions  = [-1.1, 0.4, 1.5, 1.4, -0.5, -0.9, 0.2]
# arm_joint_positions  = [-1.1, 0.4, 1.5, 1.4, -0.5, -0.9, 0.2]


# arm_intermediate_positions#################

#################

# arm_joint_positions  = [0.23, 0.5, 0.45, -1.56, 0, 0.92, 1.05]  # PO = 1 right
# arm_joint_positions  = [0.1, 0.6, 0.2, -1.65, 0, 0.9, 1.45] # PO = 2


# arm_joint_positions  = [-0.05, 0.6, -0.05, -1.62, 0, 0.9, 1.53] # PO = 3 middle pic
# arm_joint_positions  = [-0.25, 0.58, -0.5, -1.60,-0.24, 0.95, 2.1] # PO = 4 left side


####################### VIDEO PO ###############

# arm_joint_positions  = [-0.05, 0.6, -0.05, -1.62, 0, 0.9, 1.53]

# arm_joint_positions  = [-0.05, -0.70, -0.05, 1.25, 0, -0.7, 1.53]

# arm_joint_positions  = [-0.55, 0.5, -1.0, -1.60, -0.4, 0.65, 2.9] 

# arm_joint_positions = [0.72, 0.24773788452148438, 0.7336263656616211, -2.193209171295166, 1.1121363639831543, 0.2826355993747711, 0.03719902038574219, 1.1144371032714844]
#################
#fig=1c
# arm_joint_positions  = [0.20, -0.18, -0.5, -0.68,-0.24, 0.95, 2.1] # PO = 4 left side
# arm_joint_positions  = [-0.35, 0.68, -0.5, -1.60,-0.24, 0.95, 2.1] # PO = 4 left side
# 
# arm_joint_positions  = [0, 0.5, 0, -1.5, 0, 0.9, 1.57]

# arm_joint_positions  = [0.3014039, -0.677831, -0.201364, 0.0132410, -0.26183867, 0.7160369, 0.35760139]
# arm_joint_positions  = [-0.55, 0.5, -1.0, -1.60, -0.4, 0.65, 2.9]
# arm_joint_positions  = [0.2, -0.18, -0.5, -0.68, -0.24, 0.95, 2.1]
# arm_joint_positions  = [0.2984024, -0.667248, -0.2392133, -0.01010590, -0.21816394, 0.72837132, 0.35240399]

# arm_joint_positions  = [0.1593387, 0.3884378, 0.582715, -0.96319146, 0.77580548, 0.41651449, 1.65812]


# arm_intermediate_positions  = [0, 0, 0, 0, 0.0, 0, 0.0]
# arm_joint_positions  = [1.0, 0.5, 1.57, -1.7, 0.4, 0.9, 0]



#electronics_paper

# arm_joint_positions  = [0.20, -0.18, -0.5, -0.68,-0.24, 0.95, 2.1] # PO = 4 left side
# arm_joint_positions  = [0.048, -0.188, -0.426, -0.634,-0.078,0.830,1.860] # PO = 4 left side
# arm_joint_positions  = [0.089, -0.244, -0.151, -0.644, -0.298, 0.821, 1.812] # PO = 4 left side
arm_joint_positions  = [0.063, -0.150, -0.304,-0.645, -0.256, 0.669, 1.959] # PO = 4 left side



head_joint_names = ["head_pan_joint", "head_tilt_joint"]


head_joint_positions = [0.0, 0.0]
# head_joint_positions = [0.2, 0.5]

torso_joint_name = ["torso_lift_joint"]
torso_joint_position = [0.0]
# torso_joint_position = [0.3]


if __name__ == "__main__":
    rospy.init_node("prepare_simulated_robot")

    # Check robot serial number, we never want to run this on a real robot!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for torso_controller...")
    torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    torso_client.wait_for_server()
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


    trajectory = JointTrajectory()
    trajectory.joint_names = torso_joint_name
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = torso_joint_position
    trajectory.points[0].velocities = [0.0] * len(torso_joint_position)
    trajectory.points[0].accelerations = [0.0] * len(torso_joint_position)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    torso_goal = FollowJointTrajectoryGoal()
    torso_goal.trajectory = trajectory
    torso_goal.goal_time_tolerance = rospy.Duration(0.0)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0] * len(arm_joint_positions)
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = arm_intermediate_positions
    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[1].time_from_start = rospy.Duration(3.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[2].positions = arm_joint_positions
    trajectory.points[2].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[2].time_from_start = rospy.Duration(6.0)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.01
    # gripper_goal.command.position = 0.1

    rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)
    torso_client.send_goal(torso_goal)
    arm_client.send_goal(arm_goal)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(6.0))
    torso_client.wait_for_result(rospy.Duration(6.0))
    head_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")



