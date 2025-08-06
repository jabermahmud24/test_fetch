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

#po4
# waypoint_PO_4 = np.array([-0.05, -0.7, -0.05, 1.25, 0, -0.7, 1.53])
# waypoint_PO_4 = waypoint_PO_4.tolist()




#### Human uncertainty joint angles #####

waypoint_PO_5 = np.array([-0.0433349609375, 0.9878835678100586, 2.927985906600952, 1.8783597946166992, 2.916097640991211, 0.8950777053833008, -1.237155556678772])
waypoint_PO_5= waypoint_PO_5.tolist()

waypoint_PO_6 = np.array([-0.40880584716796875, 1.0339031219482422, 2.8578062057495117, 1.9370341300964355, -2.9885783195495605, 0.8448395729064941, -1.4442429542541504])
waypoint_PO_6 = waypoint_PO_6.tolist()

waypoint_PO_7 = np.array([-0.6308498382568359, 0.9357285499572754, 2.806417942047119, 1.764845371246338, -2.7040247917175293, 0.7604708671569824, -1.6010924577713013])
waypoint_PO_7= waypoint_PO_7.tolist()


waypoint_PO_8 = np.array([-0.7516508102416992, 0.8651652336120605, 2.768068313598633, 1.6390585899353027, -2.519563674926758, 0.7086987495422363, -1.7149903774261475])
waypoint_PO_8 = waypoint_PO_8.tolist()


waypoint_PO_9 = np.array([-0.8026556968688965, 0.9963207244873047, 2.692136287689209, 1.5846023559570312, -2.424456834793091, 0.5384268760681152, -1.8611021041870117])
waypoint_PO_9= waypoint_PO_9.tolist()


waypoint_PO_10 = np.array([-0.8310341835021973, 1.0396556854248047, 2.6418983936309814, 1.5381994247436523, -2.3615636825561523, 0.4513735771179199, -1.9424030780792236])
waypoint_PO_10= waypoint_PO_10.tolist()

waypoint_PO_11 = np.array([-0.4567427635192871, 1.2114615440368652, 2.897306203842163, 1.990340232849121, -2.9337382316589355, 0.7416796684265137, -1.5458683967590332])
waypoint_PO_11= waypoint_PO_11.tolist()

waypoint_PO_12 = np.array([0.05023765563964844, 1.1681265830993652, 3.008136510848999, 2.2050976753234863, 2.8152382373809814, 1.0335192680358887, -1.2310196161270142])
waypoint_PO_12= waypoint_PO_12.tolist()

waypoint_PO_13 = np.array([0.1994175910949707, 0.8402379751205444, 2.9590489864349365, 1.9692482948303223, 2.7389228343963623, 1.1688933372497559, -1.201874017715454])
waypoint_PO_13= waypoint_PO_13.tolist()




waypoint_PO_14 = np.array([0.31101465225219727, 0.9487671852111816, 3.024243116378784, 2.110757827758789, 2.6062333583831787, 1.2689852714538574, -1.1942040920257568])
waypoint_PO_14= waypoint_PO_14.tolist()

# 0.07363080978393555, 0.876286506652832, 2.902291774749756, 2.0582189559936523, 2.8259761333465576, 1.2118449211120605, -1.2241166830062866

waypoint_PO_15 = np.array([0.4705486297607422, 0.7612379789352417, 2.911879062652588, 1.9259133338928223, 2.45743727684021, 1.315004825592041, -1.0987130403518677])
waypoint_PO_15= waypoint_PO_15.tolist()


waypoint_PO_16 = np.array([0.6392865180969238, 0.5679564476013184, 3.0096704959869385, 1.7399182319641113, 2.314777135848999, 1.3817334175109863, -1.0875916481018066])
waypoint_PO_16= waypoint_PO_16.tolist()
#arm_intermediate_positions= arm_joint_positions












# ##########FOR VIDEO #####################

# #### Human uncertainty joint angles #####

# waypoint_PO_5 = np.array([-0.059441566467285156, 1.2866263389587402, 2.8999907970428467, 1.8699231147766113, 2.7569470405578613, 0.6392865180969238, -1.1593060493469238])
# waypoint_PO_5= waypoint_PO_5.tolist()

# waypoint_PO_6 = np.array([-0.2722816467285156, 1.3272771835327148, 2.8144712448120117, 1.8745245933532715, 2.9601995944976807, 0.5606698989868164, -1.3130875825881958])
# waypoint_PO_6 = waypoint_PO_6.tolist()

# waypoint_PO_7 = np.array([ -0.384645938873291, 1.0630488395690918, 2.783024787902832, 1.5638937950134277, -3.1066946983337402, 0.48051929473876953, -1.4032089710235596])
# waypoint_PO_7= waypoint_PO_7.tolist()


# waypoint_PO_8 = np.array([-0.18867969512939453, 0.9698596000671387, 2.888869285583496, 1.5849862098693848, 3.013505458831787, 0.6231794357299805, -1.2969807386398315])
# waypoint_PO_8 = waypoint_PO_8.tolist()


# waypoint_PO_9 = np.array([-0.2661457061767578, 1.2574810981750488, 2.828660726547241, 2.0609030723571777, 3.023092746734619, 0.807640552520752, -1.3594905138015747])
# waypoint_PO_9= waypoint_PO_9.tolist()


# waypoint_PO_10 = np.array([0.1587667465209961, 1.1957383155822754, 2.974388837814331, 1.871840000152588, 2.60239839553833, 0.7769608497619629, -1.0618982315063477])
# waypoint_PO_10= waypoint_PO_10.tolist()

# waypoint_PO_11 = np.array([-0.2308640480041504, 1.1558547019958496, 2.360413074493408, 1.9435539245605469, 2.6016314029693604, 0.8939270973205566, -0.847140908241272])
# waypoint_PO_11= waypoint_PO_11.tolist()

# waypoint_PO_12 = np.array([-0.40880584716796875, 0.9710097312927246, 2.012582778930664, 1.9282140731811523, 2.665675163269043, 0.9522185325622559, -0.5948010683059692])
# waypoint_PO_12= waypoint_PO_12.tolist()

# waypoint_PO_13 = np.array([-0.329805850982666, 0.8379373550415039, 2.1237964630126953, 1.4940972328186035, 2.686767339706421, 0.6626796722412109, -0.49931076169013977])
# waypoint_PO_13= waypoint_PO_13.tolist()




# waypoint_PO_14 = np.array([-0.6956601142883301, 0.9809808731079102, 1.9159417152404785, 1.8277382850646973, 2.7876267433166504, 0.6952767372131348, -0.6488738656044006])
# waypoint_PO_14= waypoint_PO_14.tolist()

# # 0.07363080978393555, 0.876286506652832, 2.902291774749756, 2.0582189559936523, 2.8259761333465576, 1.2118449211120605, -1.2241166830062866

# waypoint_PO_15 = np.array([-1.0085926055908203, 0.6902914047241211, 1.6843109130859375, 1.817000389099121, 2.980524778366089, 0.6534757614135742, -0.46786415576934814])
# waypoint_PO_15= waypoint_PO_15.tolist()


# waypoint_PO_16 = np.array([-1.0070586204528809, 1.0170292854309082, 1.7241944074630737, 1.680476188659668, 2.517646074295044, 0.6055388450622559, -0.4460049271583557])
# waypoint_PO_16= waypoint_PO_16.tolist()
# #arm_intermediate_positions= arm_joint_positions




###############################################

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0073164403438568115, 0.42031073570251465]
# head_joint_positions = [0.0, 0.0]
# arm_velocities = []

# def read_file_to_tuples(filename):
#     global arm_velocities
#     with open(filename, 'r') as file:
#         for line in file:
#             # Split each line into two decimal numbers
#             parts = line.strip().split()
#             if len(parts) == 8:
#                 try:
#                     # Convert the parts to float and create a tuple
#                     tuple_entry = [float(parts[1])*0.1, float(parts[2])*0.1, float(parts[3])*0.1, float(parts[4])*0.1, float(parts[5])*0.1, float(parts[6])*0.1, float(parts[7])*0.1]
#                     arm_velocities.append(tuple_entry)
#                 except ValueError:
#                     print(f"Skipping line with invalid format: {line}")

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
    
    # filename = '/home/fetch/workspace/Fetch-Controller-API/main/arm_velocity_iteration.txt'  # Replace 'data.txt' with your actual file name
    # read_file_to_tuples(filename)
    
    # df = pd.DataFrame(arm_velocities)

	# Compute the cumulative sum for each row
    


    #########################################



    # ts = 0
    # j=0
    # trajectory = JointTrajectory()
    # trajectory.joint_names = arm_joint_names 
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[j].positions = arm_joint_positions
    # trajectory.points[j].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[j].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[j].time_from_start = rospy.Duration(ts+1.7)
    
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[j+1].positions = waypoint_PO_1
    # trajectory.points[j+1].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[j+1].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[j+1].time_from_start = rospy.Duration(ts+3.4)
    
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[j+2].positions = waypoint_PO_2
    # trajectory.points[j+2].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[j+2].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[j+2].time_from_start = rospy.Duration(ts+5.1)
    
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[j+3].positions = waypoint_PO_3
    # trajectory.points[j+3].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[j+3].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[j+3].time_from_start = rospy.Duration(ts+7.8)
    
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[j+4].positions = waypoint_PO_4
    # trajectory.points[j+4].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[j+4].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[j+4].time_from_start = rospy.Duration(ts+10.5)




    # -0.0433349609375, 0.9878835678100586, 2.927985906600952, 1.8783597946166992, 2.916097640991211, 0.8950777053833008, -1.237155556678772

    # -0.40880584716796875, 1.0339031219482422, 2.8578062057495117, 1.9370341300964355, -2.9885783195495605, 0.8448395729064941, -1.4442429542541504
    
    # -0.6308498382568359, 0.9357285499572754, 2.806417942047119, 1.764845371246338, -2.7040247917175293, 0.7604708671569824, -1.6010924577713013

    # -0.7516508102416992, 0.8651652336120605, 2.768068313598633, 1.6390585899353027, -2.519563674926758, 0.7086987495422363, -1.7149903774261475

    # -0.8026556968688965, 0.9963207244873047, 2.692136287689209, 1.5846023559570312, -2.424456834793091, 0.5384268760681152, -1.8611021041870117

    # -0.8310341835021973, 1.0396556854248047, 2.6418983936309814, 1.5381994247436523, -2.3615636825561523, 0.4513735771179199, -1.9424030780792236



    # -0.4567427635192871, 1.2114615440368652, 2.897306203842163, 1.990340232849121, -2.9337382316589355, 0.7416796684265137, -1.5458683967590332

    # 0.05023765563964844, 1.1681265830993652, 3.008136510848999, 2.2050976753234863, 2.8152382373809814, 1.0335192680358887, -1.2310196161270142

    # 0.31101465225219727, 0.9487671852111816, 3.024243116378784, 2.110757827758789, 2.6062333583831787, 1.2689852714538574, -1.1942040920257568

    # 0.6392865180969238, 0.5679564476013184, 3.0096704959869385, 1.7399182319641113, 2.314777135848999, 1.3817334175109863, -1.0875916481018066



      
    j =0
    ts = 0
    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names 
    while (j < 36 ):
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j].positions = waypoint_PO_5
	    trajectory.points[j].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j].time_from_start = rospy.Duration(ts+1.5)
        
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+1].positions = waypoint_PO_6
	    trajectory.points[j+1].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+1].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+1].time_from_start = rospy.Duration(ts+3+2)
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+2].positions = waypoint_PO_7
	    trajectory.points[j+2].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+2].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+2].time_from_start = rospy.Duration(ts+4.4+4)
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+3].positions = waypoint_PO_8
	    trajectory.points[j+3].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+3].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+3].time_from_start = rospy.Duration(ts+5.9+6)
          
    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+4].positions = waypoint_PO_9
	    trajectory.points[j+4].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+4].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+4].time_from_start = rospy.Duration(ts+7.3+8)
        
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+5].positions = waypoint_PO_10
	    trajectory.points[j+5].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+5].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+5].time_from_start = rospy.Duration(ts+8.2+10)
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+6].positions = waypoint_PO_11
	    trajectory.points[j+6].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+6].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+6].time_from_start = rospy.Duration(ts+10.5+12)
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+7].positions = waypoint_PO_12
	    trajectory.points[j+7].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+7].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+7].time_from_start = rospy.Duration(ts+12.5+14)
          


	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+8].positions = waypoint_PO_13
	    trajectory.points[j+8].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+8].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+8].time_from_start = rospy.Duration(ts+14.5+16)
        
	    
	    # trajectory.points.append(JointTrajectoryPoint())
	    # trajectory.points[j+9].positions = waypoint_PO_14
	    # trajectory.points[j+9].velocities =  [0.0] * len(arm_joint_positions)
	    # trajectory.points[j+9].accelerations = [0.0] * len(arm_joint_positions)
	    # trajectory.points[j+9].time_from_start = rospy.Duration(ts+16.5)
         

	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+9].positions = waypoint_PO_15
	    trajectory.points[j+9].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+9].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+9].time_from_start = rospy.Duration(ts+16.5+18)
        
	    
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+10].positions = waypoint_PO_16
	    trajectory.points[j+10].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+10].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+10].time_from_start = rospy.Duration(ts+18.5+20)
         
	    trajectory.points.append(JointTrajectoryPoint())
	    trajectory.points[j+11].positions = waypoint_PO_5
	    trajectory.points[j+11].velocities =  [0.0] * len(arm_joint_positions)
	    trajectory.points[j+11].accelerations = [0.0] * len(arm_joint_positions)
	    trajectory.points[j+11].time_from_start = rospy.Duration(ts+22.5+22)
         
         
	    
	    ts += 22.5+22
	    j += 12
    


    ##############
        
    # j =0
    # ts = 0
    # trajectory = JointTrajectory()
    # trajectory.joint_names = arm_joint_names 
    # while (j <30):
	#     trajectory.points.append(JointTrajectoryPoint())
	#     trajectory.points[j].positions = arm_joint_positions
	#     trajectory.points[j].velocities =  [0.0] * len(arm_joint_positions)
	#     trajectory.points[j].accelerations = [0.0] * len(arm_joint_positions)
	#     trajectory.points[j].time_from_start = rospy.Duration(ts+2.7)
	    
	#     trajectory.points.append(JointTrajectoryPoint())
	#     trajectory.points[j+1].positions = waypoint_PO_1
	#     trajectory.points[j+1].velocities =  [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+1].accelerations = [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+1].time_from_start = rospy.Duration(ts+4.4)
	    
	#     trajectory.points.append(JointTrajectoryPoint())
	#     trajectory.points[j+2].positions = waypoint_PO_2
	#     trajectory.points[j+2].velocities =  [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+2].accelerations = [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+2].time_from_start = rospy.Duration(ts+6.1)
	    
	#     trajectory.points.append(JointTrajectoryPoint())
	#     trajectory.points[j+3].positions = waypoint_PO_3
	#     trajectory.points[j+3].velocities =  [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+3].accelerations = [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+3].time_from_start = rospy.Duration(ts+7.8)
	    
	#     trajectory.points.append(JointTrajectoryPoint())
	#     trajectory.points[j+4].positions = waypoint_PO_4
	#     trajectory.points[j+4].velocities =  [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+4].accelerations = [0.0] * len(arm_joint_positions)
	#     trajectory.points[j+4].time_from_start = rospy.Duration(ts+9.5)
	#     # ts += 1
	#     ts += 9.5
	#     j += 5
    
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

    # gripper_client.wait_for_result(rospy.Duration(2.0))
    # arm_client.wait_for_result(rospy.Duration(2.0))
    # head_client.wait_for_result(rospy.Duration(2.0))
    rospy.loginfo("...done")