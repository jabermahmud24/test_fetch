#!/usr/bin/env python3


import copy
import math
import threading

import rospy
import actionlib

from math import sin, cos

# --- ROS msgs/actions -------------------------------------------------------
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              PointHeadAction, PointHeadGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from moveit_python import MoveGroupInterface, PlanningSceneInterface
# ---------------------------------------------------------------------------


# ========================== USER SETTINGS ===================================
FRAME   = "map"             # world frame
EE_LINK = "gripper_link"    # end-effector link for Fetch

def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    )

def make_ps(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, frame=FRAME):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    ps.pose.orientation = Quaternion(qx, qy, qz, qw)
    return ps

# <<< EDIT THESE >>>

# >>> Set these to the actual object pose (grasp pose) and the placement pose
PICK_POSE  = make_ps(3.7, 3.15, 0.83, roll=0.0, pitch=math.pi/2, yaw=0.0)   # example
PLACE_POSE = make_ps(5.8, -6.00, 0.43, roll=0.0, pitch=math.pi/2, yaw=0.0)  # example

RETREAT_Z  = 0.10  # retreat height after grasp/place
# ============================================================================


# # ----------------------------- UTILITIES ------------------------------------
# def wait_for_time(timeout=5.0):
#     """Block until /clock starts if /use_sim_time==true, else return immediately."""
#     if not rospy.get_param("/use_sim_time", False):
#         return
#     start = rospy.Time.now()
#     r = rospy.Rate(10)
#     while rospy.Time.now().to_sec() == 0.0 and not rospy.is_shutdown():
#         if (rospy.Time.now() - start).to_sec() > timeout:
#             rospy.logwarn("/clock never started; continuing anyway.")
#             break
#         r.sleep()
# ---------------------------------------------------------------------------


# --------------------------- MOVE_BASE CLIENT -------------------------------
class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("move_base action server not available")

    def goto(self, x, y, theta, frame="map"):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = sin(theta/2.0)
        goal.target_pose.pose.orientation.w = cos(theta/2.0)

        rospy.loginfo(f"[move_base] Goal: ({x:.2f},{y:.2f},{theta:.2f}) in {frame}")
        self.client.send_goal(goal)

        # ok = self.client.wait_for_result(rospy.Duration(timeout))
        # if not ok:
        #     rospy.logerr("move_base timeout, cancelling goal.")
        #     self.client.cancel_goal()
        #     return False

        state = self.client.get_state()  # 3=SUCCEEDED
        rospy.loginfo(f"[move_base] Result state = {state}")
        return state == actionlib.GoalStatus.SUCCEEDED
# ---------------------------------------------------------------------------


# --------------------- FOLLOW TRAJECTORY (TORSO/GRIPPER) --------------------
class FollowTrajectoryClient:
    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient(f"{name}/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo(f"Waiting for {name}...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError(f"{name} action server not available")
        self.joint_names = joint_names

    def move_to(self, positions, duration=3.0):
        if len(positions) != len(self.joint_names):
            rospy.logerr("Trajectory length mismatch")
            return False
        traj = JointTrajectory(joint_names=self.joint_names)
        pt   = JointTrajectoryPoint()
        pt.positions     = positions
        pt.velocities    = [0.0]*len(positions)
        pt.accelerations = [0.0]*len(positions)
        pt.time_from_start = rospy.Duration(duration)
        traj.points.append(pt)
        goal = FollowJointTrajectoryGoal(trajectory=traj)
        self.client.send_goal(goal)
        return self.client.wait_for_result(rospy.Duration(duration+5.0))
# ---------------------------------------------------------------------------


# ----------------------------- HEAD CLIENT ----------------------------------
class PointHeadClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head",
                                                   PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame="map", duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration   = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
# ---------------------------------------------------------------------------


# -------------------------- FETCH ARM WRAPPER -------------------------------
class FetchArmFixed:
    """
    Simple wrapper (no perception):
      - torso, head, gripper via controllers
      - arm via MoveIt pose goals
      - stow/tuck joint presets reused from Fetch examples
    """
    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.arm   = MoveGroupInterface("arm", "base_link")

        # controllers
        self.torso   = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
        self.gripper = FollowTrajectoryClient("gripper_controller",
                                              ["l_gripper_finger_joint", "r_gripper_finger_joint"])

        # presets
        self.joints_arm = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                           "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.pose_tuck  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        self.pose_stow  = [1.32, 0.70,  0.0,-2.00, 0.0,-0.57, 0.0]
        self.pose_mid   = [0.70,-0.30,  0.0,-0.30, 0.0,-0.57, 0.0]

    # torso
    def set_torso(self, height):
        self.torso.move_to([height], duration=2.0)

    # gripper
    def open_gripper(self, gap=0.05):
        self.gripper.move_to([gap, gap], duration=1.0)

    def close_gripper(self):
        self.gripper.move_to([0.0, 0.0], duration=1.0)

    # head
    def look_at(self, x, y, z, frame="map"):
        PointHeadClient().look_at(x, y, z, frame)

    # arm joint presets
    def tuck(self):
        self._move_joints(self.pose_tuck)

    def stow(self):
        self._move_joints(self.pose_stow)

    def intermediate_stow(self):
        self._move_joints(self.pose_mid)

    def _move_joints(self, joint_vals):
        while not rospy.is_shutdown():
            res = self.arm.moveToJointPosition(self.joints_arm, joint_vals, 0.02)
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    # pose goal
    def move_to_pose(self, pose_stamped, ee_link=EE_LINK, plan_only=False):
        res = self.arm.moveToPoseStamped(pose_stamped, ee_link, plan_only=plan_only)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logwarn("MoveIt failed (code %d)", res.error_code.val)
            return False
        return True

    def retreat_up(self, pose_stamped, dz):
        p = copy.deepcopy(pose_stamped)
        p.pose.position.z += dz
        return p
# ---------------------------------------------------------------------------


# ---------------------------- LOCOBOT HELPERS -------------------------------
class LocobotCommander:
    """Same minimal helper as before."""
    def __init__(self):
        rospy.loginfo("Waiting for /clock …")
        rospy.wait_for_message("/clock", Clock)
        rospy.loginfo("/clock publishing – LoCoBot up")

        self.base_pub = rospy.Publisher("/locobot/cmd_vel", Twist, queue_size=10)
        jt = ["waist", "shoulder", "elbow",
              "wrist_angle", "wrist_rotate",
              "left_finger", "right_finger",
              "pan", "tilt"]
        self.joint_pubs = {j: rospy.Publisher(f"/locobot/{j}_controller/command",
                                              Float64, queue_size=10) for j in jt}

    def move_base(self, vx=0.0, vth=0.0, dur=0.0, rate_hz=10):
        twist = Twist()
        twist.linear.x  = vx
        twist.angular.z = vth
        r = rospy.Rate(rate_hz)
        end_t = rospy.Time.now() + rospy.Duration.from_sec(dur)
        while not rospy.is_shutdown() and rospy.Time.now() < end_t:
            self.base_pub.publish(twist)
            r.sleep()
        self.base_pub.publish(Twist())
        rospy.sleep(0.1)

    def _set(self, j, v):
        pub = self.joint_pubs.get(j)
        if pub:
            pub.publish(Float64(v))

    def set_arm(self, waist=0, shoulder=0, elbow=0,
                wrist_angle=0, wrist_rotate=0, grip_open=True):
        self._set("waist", waist)
        self._set("shoulder", shoulder)
        self._set("elbow", elbow)
        self._set("wrist_angle", wrist_angle)
        self._set("wrist_rotate", wrist_rotate)
        g = 0.1 if grip_open else 0.0
        self._set("left_finger", g)
        self._set("right_finger", g)

    def demo(self):
        rospy.loginfo("LoCoBot ⇢ forward 0.3 m/s for 3 s")
        self.move_base(vx=0.3, dur=3.0)
        rospy.loginfo("LoCoBot ⇢ spin 0.6 rad/s for 2 s")
        self.move_base(vth=0.6, dur=2.0)
        rospy.loginfo("LoCoBot ⇢ arm to pick pose")
        self.set_arm(waist=0.5, shoulder=-0.4, elbow=0.4,
                     wrist_angle=1.1, wrist_rotate=0.0, grip_open=True)
        rospy.sleep(2.0)
        rospy.loginfo("LoCoBot ⇢ close gripper")
        self.set_arm(grip_open=False)
        rospy.sleep(1.0)
        rospy.loginfo("LoCoBot demo ✔")
# ---------------------------------------------------------------------------


# ----------------------------- FETCH DEMO -----------------------------------
class FetchFixedDemo:
    def __init__(self):
        self.nav  = MoveBaseClient()
        self.armc = FetchArmFixed()

    def run(self):
        # (Optional) Navigate to pick table
        rospy.loginfo("Moving to table 1…")
        self.nav.goto(2.250, 3.118, 0.0)
        self.nav.goto(2.700, 3.118, 0.0)

        # Torso/head
        self.armc.set_torso(0.4)
        self.armc.look_at(PICK_POSE.pose.position.x,
                          PICK_POSE.pose.position.y,
                          PICK_POSE.pose.position.z, FRAME)

        # Grasp
        self.armc.open_gripper()
        rospy.loginfo("Approaching PICK pose…")
        if not self.armc.move_to_pose(PICK_POSE):
            rospy.logerr("Could not reach PICK pose")
            return
        rospy.sleep(0.5)
        self.armc.close_gripper()
        rospy.loginfo("Grasped.")

        # Retreat
        retreat_pick = self.armc.retreat_up(PICK_POSE, RETREAT_Z)
        self.armc.move_to_pose(retreat_pick)

        # Lower torso for navigation
        self.armc.set_torso(0.0)

        # Navigate to place table
        rospy.loginfo("Moving to table 2…")
        self.nav.goto(4.300, -6.0, 0.0)
        self.nav.goto(4.650, -6.0, 0.0)

        # Place
        self.armc.set_torso(0.4)
        rospy.loginfo("Approaching PLACE pose…")
        if not self.armc.move_to_pose(PLACE_POSE):
            rospy.logerr("Could not reach PLACE pose")
        rospy.sleep(0.5)
        self.armc.open_gripper()
        rospy.loginfo("Released object.")

        # Retreat + tuck
        retreat_place = self.armc.retreat_up(PLACE_POSE, RETREAT_Z)
        self.armc.move_to_pose(retreat_place)
        self.armc.tuck()
        self.armc.set_torso(0.0)

        rospy.loginfo("Fetch fixed pick & place ✔")
# ---------------------------------------------------------------------------


# ------------------------------- MAIN --------------------------------------
def main():
    rospy.init_node("dual_robot_fixed_demo")
    # wait_for_time()

    # Run Fetch and LoCoBot sequentially (simpler). Change to true threads if desired.
    fetch_demo = FetchFixedDemo()
    tfetch     = threading.Thread(target=fetch_demo.run, name="FetchThread")
    tfetch.start()
    tfetch.join()

    loco_demo  = LocobotCommander()
    tloco      = threading.Thread(target=loco_demo.demo, name="LoCoThread")
    tloco.start()
    tloco.join()

    rospy.loginfo("🎉 Both demos finished – exiting")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
