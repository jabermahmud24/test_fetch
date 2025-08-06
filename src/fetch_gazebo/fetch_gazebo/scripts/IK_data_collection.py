#!/usr/bin/env python
import csv
import tf
import tf.transformations
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import numpy as np
import itertools
import time
from actionlib_msgs.msg import GoalStatus
import random

# Joint names for the Fetch robot's arm
arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "upperarm_roll_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]

# Joint names for the Fetch robot's head
head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]

def get_pose(link_name):
    """
    Retrieves the pose of the specified link from the /gazebo/link_states topic.
    """
    link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
    try:
        index = link_states.name.index(link_name)
        pose = link_states.pose[index]
        return pose
    except ValueError:
        print("Link {} not found in /gazebo/link_states".format(link_name))
        return None

def move_arm(angle_combination):
    """
    Sends a trajectory to the arm to move it to the specified joint angles.
    """
    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = angle_combination
    trajectory.points[0].velocities = [0.0] * len(angle_combination)
    trajectory.points[0].accelerations = [0.0] * len(angle_combination)
    trajectory.points[0].time_from_start = rospy.Duration(3)

    # Send the trajectory to the arm
    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result()  # Wait up to 10 seconds

    state = arm_client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Arm reached the goal position")
    else:
        rospy.logwarn("Arm failed to reach the goal position, state: {}".format(state))

def main():
    # start_time = time.time()
    # # number_points = 6
    # #     # Define random joint angles within specified ranges for each joint
    # # angle_shoulder_pan = [random.uniform(-1.57, 1.57) for _ in range(number_points)]
    # # angle_shoulder_lift = [random.uniform(-1.45, 1.45) for _ in range(number_points)]
    # # angle_upperarm_roll = [random.uniform(-1.57, 1.57) for _ in range(number_points)]
    # # angle_elbow_flex = [random.uniform(-2.1, 2.1) for _ in range(number_points)]
    # # angle_forearm_roll = [random.uniform(-1.57, 1.57) for _ in range(number_points)]
    # # angle_wrist_flex = [random.uniform(-2.1, 2.1) for _ in range(number_points)]
    # # angle_wrist_roll = [random.uniform(-1.57, 1.57) for _ in range(number_points)]


    # # Define the increment (gap) for each joint
    # increment = -0.2 # Adjust as needed

    # # Define joint angle ranges
    # angle_shoulder_pan = np.arange(0.57, -0.57 - increment, increment)
    # angle_shoulder_lift = np.arange(0.3, -0.7 - increment, increment)
    # angle_upperarm_roll = np.arange(1.0, -1.0 - increment, increment)
    # angle_elbow_flex = np.arange(1.1, -1.1 - increment, increment)
    # angle_forearm_roll = np.arange(1.0, -1.0 - increment, increment)
    # angle_wrist_flex = np.arange(0.2, -0.2 - increment, increment)
    # angle_wrist_roll = np.arange(1.0, -1.0 - increment, increment)

    # # Generate all combinations
    # all_combinations = list(itertools.product(
    #     angle_shoulder_pan,
    #     angle_shoulder_lift,
    #     angle_upperarm_roll,
    #     angle_elbow_flex,
    #     angle_forearm_roll,
    #     angle_wrist_flex,
    #     angle_wrist_roll
    # ))

    # total_combinations = len(all_combinations)
    # print("Total number of combinations:", total_combinations)

    # rospy.init_node("prepare_simulated_robot")

    # # Check robot serial number, ensure this script is not run on a real robot
    # if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
    #     rospy.logerr("This script should not be run on a real robot")
    #     return

    # rospy.loginfo("Waiting for head_controller...")
    # head_client = actionlib.SimpleActionClient(
    #     "head_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    # )
    # head_client.wait_for_server()
    # rospy.loginfo("...connected.")

    # rospy.loginfo("Waiting for arm_controller...")
    # global arm_client
    # arm_client = actionlib.SimpleActionClient(
    #     "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    # )
    # arm_client.wait_for_server()
    # rospy.loginfo("...connected.")

    # # Initialize the head position
    # trajectory = JointTrajectory()
    # trajectory.joint_names = head_joint_names
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[0].positions = head_joint_positions
    # trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    # trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    # trajectory.points[0].time_from_start = rospy.Duration(1.0)

    # head_goal = FollowJointTrajectoryGoal()
    # head_goal.trajectory = trajectory
    # head_goal.goal_time_tolerance = rospy.Duration(0.0)

    # # Send head goal
    # head_client.send_goal(head_goal)
    # head_client.wait_for_result(rospy.Duration(1.0))

    # data = []

    # try:
    #     # Main loop to move the arm and get the pose
    #     for i, angle_combination in enumerate(all_combinations):
    #         print("Iteration:", i + 1, "/", total_combinations)
    #         print("Angle combination:", angle_combination)

    #         move_arm(angle_combination)

    #         # Wait a moment for the arm to settle
    #         rospy.sleep(2.0)

    #         # Get the pose of the end effector
    #         link_name = "fetch::r_gripper_finger_link"  # Adjust as needed
    #         pose = get_pose(link_name)
    #         print("Link name:", link_name)

    #         if pose is not None:
    #             position_x = pose.position.x
    #             position_y = pose.position.y
    #             position_z = pose.position.z
    #             orientation_x = pose.orientation.x
    #             orientation_y = pose.orientation.y
    #             orientation_z = pose.orientation.z
    #             orientation_w = pose.orientation.w

    #             orientation_q = [
    #                 pose.orientation.x,
    #                 pose.orientation.y,
    #                 pose.orientation.z,
    #                 pose.orientation.w,
    #             ]

    #             roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)

    #             print("Pose:")
    #             print(
    #                 "Position: x={}, y={}, z={}".format(
    #                     position_x, position_y, position_z
    #                 )
    #             )
    #             print(
    #                 "Orientation: roll={}, pitch={}, yaw={}".format(
    #                     roll, pitch, yaw
    #                 )
    #             )

    #             within_tolerance = abs(pitch) <= 0.2 and abs(yaw) <= 0.2
    #             data_row = list(angle_combination) + [position_x, position_y, position_z, roll, pitch, yaw, within_tolerance]
    #             data.append(data_row)
    #         else:
    #             print("Pose not found")

    # except KeyboardInterrupt:
    #     rospy.logwarn("Process interrupted! Saving collected data...")
    # finally:
    #     # Save data to CSV file even when interrupted
    #     header = [
    #         "shoulder_pan_joint",
    #         "shoulder_lift_joint",
    #         "upperarm_roll_joint",
    #         "elbow_flex_joint",
    #         "forearm_roll_joint",
    #         "wrist_flex_joint",
    #         "wrist_roll_joint",
    #         "position_x",
    #         "position_y",
    #         "position_z",
    #         "roll",
    #         "pitch",
    #         "yaw",
    #         "within_tolerance"
    #     ]

    #     with open('robot_IK_data.csv', 'w', newline='') as csvfile:
    #         data_writer = csv.writer(csvfile)
    #         data_writer.writerow(header)
    #         data_writer.writerows(data)

    #     end_time = time.time()
    #     duration = end_time - start_time
    #     print("Total time: {} seconds".format(duration))
    #     print("Data saved to robot_IK_data.csv")


    start_time = time.time()
    
    # Define the number of random points you want to generate
    number_points = 1000  # Adjust as needed
    
    # Define the random joint angles within specified ranges for each joint
    angle_shoulder_pan = [random.uniform(-1.57, 1.57) for _ in range(number_points)]    # -90 to +90 degrees
    angle_shoulder_lift = [random.uniform(-1.05, 1.05) for _ in range(number_points)]   # -60 to +60 degrees
    angle_upperarm_roll = [random.uniform(-1.05, 1.05) for _ in range(number_points)]   # -60 to +60 degrees
    angle_elbow_flex = [random.uniform(-0.78, 0.78) for _ in range(number_points)]      # -45 to +45 degrees
    angle_forearm_roll = [random.uniform(-0.78, 0.78) for _ in range(number_points)]    # -45 to +45 degrees
    angle_wrist_flex = [random.uniform(-0.78, 0.78) for _ in range(number_points)]      # -45 to +45 degrees
    angle_wrist_roll = [random.uniform(-0.78, 0.78) for _ in range(number_points)]      # -45 to +45 degrees

    # Initialize data collection structures
    data = []
    
    rospy.init_node("prepare_simulated_robot")
    
    # Check robot serial number, ensure this script is not run on a real robot
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        return
    
    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient(
        "head_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    )
    head_client.wait_for_server()
    rospy.loginfo("...connected.")
    
    rospy.loginfo("Waiting for arm_controller...")
    global arm_client
    arm_client = actionlib.SimpleActionClient(
        "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    )
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")
    
    # Initialize the head position
    trajectory = JointTrajectory()
    head_joint_names = ["head_pan_joint", "head_tilt_joint"]  # Replace with your actual joint names
    head_joint_positions = [0.0, 0.0]  # Replace with your desired initial positions
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)
    
    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)
    
    # Send head goal
    head_client.send_goal(head_goal)
    head_client.wait_for_result(rospy.Duration(1.0))
    
    try:
        # Main loop to move the arm and get the pose
        for i in range(number_points):
            print(f"Iteration: {i + 1} / {number_points}")
            
            # Get the i-th angle for each joint
            angle_combination = (
                angle_shoulder_pan[i],
                angle_shoulder_lift[i],
                angle_upperarm_roll[i],
                angle_elbow_flex[i],
                angle_forearm_roll[i],
                angle_wrist_flex[i],
                angle_wrist_roll[i]
            )
            print("Angle combination (radians):", angle_combination)
            
            # Move the arm to the specified joint angles
            move_arm(angle_combination)
            
            # Wait a moment for the arm to settle
            rospy.sleep(2.0)
            
            # Get the pose of the end effector
            link_name = "fetch::r_gripper_finger_link"  # Adjust as needed
            pose = get_pose(link_name)
            print("Link name:", link_name)
            
            if pose is not None:
                position_x = pose.position.x
                position_y = pose.position.y
                position_z = pose.position.z
                orientation_x = pose.orientation.x
                orientation_y = pose.orientation.y
                orientation_z = pose.orientation.z
                orientation_w = pose.orientation.w
                
                orientation_q = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
                
                # Convert quaternion to Euler angles
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
                
                print("Pose:")
                print(f"Position: x={position_x}, y={position_y}, z={position_z}")
                print(f"Orientation: roll={roll}, pitch={pitch}, yaw={yaw}")
                
                # Check if the pose is within the specified tolerance
                within_tolerance = abs(pitch) <= 0.2 and abs(yaw) <= 0.2
                data_row = list(angle_combination) + [
                    position_x, position_y, position_z,
                    roll, pitch, yaw,
                    within_tolerance
                ]
                data.append(data_row)
            else:
                print("Pose not found")
    
    except KeyboardInterrupt:
        rospy.logwarn("Process interrupted! Saving collected data...")
    
    finally:
        # Save data to CSV file even when interrupted
        header = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "upperarm_roll_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "position_x",
            "position_y",
            "position_z",
            "roll",
            "pitch",
            "yaw",
            "within_tolerance"
        ]
        
        with open('robot_IK_data.csv', 'w', newline='') as csvfile:
            data_writer = csv.writer(csvfile)
            data_writer.writerow(header)
            data_writer.writerows(data)
        
        end_time = time.time()
        duration = end_time - start_time
        print(f"Total time: {duration} seconds")
        print("Data saved to robot_IK_data.csv")


if __name__ == "__main__":
    main()
