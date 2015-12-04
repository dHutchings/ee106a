#!/usr/bin/env python
import sys
import rospy

import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import numpy as np
import exp_quat_func as eqf
import kin_func_skeleton as kfs
from tf2_msgs.msg import TFMessage
import tf
import time

from kinematics.srv import *

import traceback
rbt = None

usage_str = \
"""
Low-level arm controller.
Handles direct IK planning

Requires baxter.sh and baxter_moveit_noexec.launch to be run beforehand.

Feed this service commands, and it'll move the arm as appropriate.

Flags:
--help, -h      print usage instructions

Takes a kinematics_request.srv, which has as data:

#an array, trans
#an array, rot
#a string, grip
#a string, incrimental
#a string, change_height,
#a string, keep_orient,

#an array, target_rot
#an array, target_trans
#both of these are used to construct the RBT to a target, for the purposes of incrimental movement's movement along a coord frame

#priority: Grip, then incrimental or not.

"""

arm = None
gripper = None

def move_to_coord(trans, rot, keep_oreint=False):
    #coordinates are in baxter's torso!

    global arm
    goal = PoseStamped()
    goal.header.frame_id = "base"

    # x, y, and z position
    goal.pose.position.x = trans[0]
    goal.pose.position.y = trans[1]
    goal.pose.position.z = trans[2]
    
    # Orientation as a quaternion
    goal.pose.orientation.x = rot[0]
    goal.pose.orientation.y = rot[1]
    goal.pose.orientation.z = rot[2]
    goal.pose.orientation.w = rot[3]

    # Set the goal state to the pose you just defined
    arm.set_pose_target(goal)

    # Set the start state for the arm
    arm.set_start_state_to_current_state()

    if keep_oreint:
        # Create a path constraint for the arm
        orien_const = OrientationConstraint()
        orien_const.link_name = 'left'+"_gripper";
        orien_const.header.frame_id = "base";

        #constrain it to be the same as my goal state.  Seems reasonable.

        orien_const.orientation.x = rot[0];
        orien_const.orientation.y = rot[1];
        orien_const.orientation.z = rot[2];
        orien_const.orientation.w = rot[3];
        orien_const.absolute_x_axis_tolerance = 0.1;
        orien_const.absolute_y_axis_tolerance = 0.1;
        orien_const.absolute_z_axis_tolerance = 0.1;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        print(consts)
        arm.set_path_constraints(consts)

    # Plan a path
    arm_plan = arm.plan()

    # Execute the plan
    #arm.execute(arm_plan)
    arm.go(arm_plan,wait=True)

def incrimental_movement(trans,rbt=None,changeHeight=True,keep_oreint=False):
    #move the arm an incrimental distance dx, dy, dz.\
    #relative to a frame which has the RBT to the BASE FRAME

    dx = trans[0]
    dy = trans[1]
    dz = trans[2]

    if rbt == None:
        new_rot = np.eye(3)
    else:
        new_rot = np.array([[rbt[0][0], rbt[0][1], rbt[0][2]], 
            [rbt[1][0], rbt[1][1], rbt[1][2]], [rbt[2][0], rbt[2][1], rbt[2][2]]])
        new_rot = np.linalg.inv(new_rot)

    inc_dist = np.array([[dx],[dy],[dz]])
    inc_trans = np.dot(new_rot, inc_dist)
    
    pose = get_pose()

    #ok, so i don't know where my code thinks I should be.  Let me just fetch where I am

    trans = pose['trans']
    last_pos = np.array([[trans[0]], [trans[1]], [trans[2]] ])


    #print("I think that i'm at \n\r")
    #print(last_pos)

    #print("And I want to move by, in the base frame \n\r")
    #print(inc_trans)
    dest = last_pos + inc_trans
    #pack np array into normal list


    if changeHeight:
        dest_ar = [dest[0][0], dest[1][0] ,dest[2][0]]
    else:
        #get my last height, and shove that into the dest array
        dest_ar = [dest[0][0], dest[1][0] ,last_pos[2][0]]

    #print("My destination is \n\r")
    #print(str(dest_ar) + "\n\r")
    #print(pose['rot'])

    move_to_coord(dest_ar, pose['rot'],keep_oreint)
    #but return it as a col array
    return dest

def get_pose():
    limb = baxter_interface.Limb('left')
    pose = {}

    while len(pose) is 0:
        #sometimes, i'm getting empty poses.  don't know why, but this should take care of it.
        pose = limb.endpoint_pose()
        #print(pose)
        time.sleep(.01)

    pose = {'rot': list(pose['orientation']), 
            'trans' : list(pose['position'])}

    return pose

def actuate_gripper(state):
    if state is True:
        gripper.open(block=False)
    else:
        gripper.close(block=False)

def movment_handle(data):
    #run every time i get a service call
    #print(data)

    

    #anything can be empty.

    #if incrimental is not empty, assume that this is to be incrimental


    if (len(data.trans) is not 3) and (len(data.trans) is not 0):
        return("ERROR: trans is of the wrong length")
    elif len(data.trans) is 3:
        trans = data.trans
        grip = False
    else:
        grip = True
        if data.grip == 'False':
            suck = False
        else:
            suck = True



    if len(data.rot) is 0:
        #assume that no rotation was intended... aka, keep the current rotation I want
        pose = get_pose()
        rot = pose['rot']
    else:
        rot = data.rot
        if len(rot) is not 4:
            return("Error.  ROT should be a quaternion")

    if len(data.incrimental) is 0:
        incrimental = False
    else:
        incrimental = True

        if len(data.target_rot) > 0:
            target_rot = data.target_rot
            target_trans = data.target_trans

            target_rbt = eqf.return_rbt(target_trans,target_rot)
        else:
            target_rbt = np.eye(4)

    if len(data.keep_orient) is 0:
        keep_orient = False
    else:
        keep_orient = True

    if len(data.change_height) is 0:
        changeHeight = True
    else:
        changeHeight = False

    if grip:
        actuate_gripper(suck)

    elif incrimental:
        print("Incrimental Move by " + str(trans))
        #print("changeHeight?" + str(changeHeight))
        incrimental_movement(trans,target_rbt,changeHeight = changeHeight, keep_oreint = keep_orient)
    else:
        print("Absolute Movement to " + str(trans))
        move_to_coord(trans,rot,keep_orient)


    #if bool()

    #construct a response.  Currently, i return a string...1
    resp = kinematics_requestResponse("I_did_stuff")
    print("\n\r")
    return resp





def movement_server():
    print("Initializing node... ")
    rospy.init_node('low_level_arm_controller')
    s = rospy.Service('low_level_arm', kinematics_request, movment_handle)
    print "Ready to move arm"


        
# Initializes all limbs/parts of limbs for later usage in the code. 
def init_IK():

    global arm
    print("Initializing moveit_commander")

    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled

    # Initialize moveit commander
    moveit_commander.roscpp_initialize([sys.argv[0]])

    # Initialize arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander('left_arm')
    arm.set_planner_id('RRTConnectkConfigDefault')
    arm.set_planning_time(5)

    # Initialize grippers
    global gripper
    gripper = baxter_interface.gripper.Gripper('left')

    left_arm_str = 'left_arm'

    print('Calibrating...')
    gripper.calibrate()
    rospy.sleep(2.0)

    # Shutdown callback
    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()

    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    


if __name__ == '__main__':
    if len(sys.argv) == 1:
        movement_server()
        init_IK()
        rospy.spin()
    elif sys.argv[1] == '-h':
        print(usage_str)
    elif sys.argv[1] == '--help':
        print(usage_str)
    else:
        print("invalid arguments")
        print(usage_str)