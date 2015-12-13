#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import numpy as np
import exp_quat_func as eqf
import kin_func_skeleton as kfs
from tf2_msgs.msg import TFMessage
import tf
import curses
import time

import traceback
rbt = None

from kinematics.srv import *

usage_str = \
"""
Inverse kinematics script

Requires baxter.sh and abxter_moveit_noexec.launch to be run beforehand.
Interactive mode allows control of the end-effector position via 
text input or arrow keys.

Keyboard-interactive controls:
[UP, DOWN]      move along y-axis   Up is y+, Down is y-
[LEFT, RIGHT]   move along x-axis.  Right is x+, Left is x-
[Z, X]          move along z-axis
[G]             toggle gripper

Flags:
--help, -h      print usage instructions
-k              run interactive keyboard interface
-t              run interactive text interface
-l      tf_topic    ar_marker_number        Get transforms to a set tag using the tf service.
"""

rbt = None
last_seen = time.time()

homing_marker = 'ar_marker_63'

def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("closed_loop_rel_to_tf")
    print("topic is " + str(tf_topic))

    rospy.Subscriber(tf_topic,TFMessage,callback)

    global tf_listener

    tf_listener = tf.TransformListener()

    print("Construction completed")


def callback(data):
    #run every time i see a TFMessage on tf_topic.
    global tf_listener
    global last_seen
    global trans
    global rot

    tim = data.transforms[0]
    tim = tim.header.stamp

    #print("callback run \n\r")

    #right now, hardcoded to look out for AR_Marker_63.  This may not be the world's greatest Idea...
    if tf_listener.frameExists(homing_marker):

        now = rospy.Time.now()
        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            (trans,rot) = tf_listener.lookupTransform(homing_marker,'base',tim)
            
            trans = trans
            rot = rot

            #print("updated \n\r")

            #(omega,theta) = eqf.quaternion_to_exp(rot)
            #rbt = eqf.return_rbt(trans,rot)


            #print(rbt)
            #print("")
            #print("")
            #print("Period was " + str(time.time() - last_seen))
            if last_seen - time.time() > 10:
                #print("updated rbt, i hadn't seen it in a while \n\r")
                #i haven't seen the tag in a while
                pass
            last_seen = time.time()

            #bring this down to 5hz, the camera rate.  I don't wanna kill the processor on this...
            #time.sleep(0.2)
        #time0 asks for the most recent one
        except:
            #print("error with the update \n\r")
            pass

def service_setup():
    s = rospy.Service('closed_loop', closed_loop_request, service_handle)
    print "Ready to recieve closed loop instructions"



def service_handle(data):
    print(data)

    global homing_marker

    if data.operation == "pick":
        #assume that i've already been moved to the appropriate location
        homing_marker = data.homing_tf_frame
        destination =data.target_tf_frame
        #so i have time to see the AR_tag
        time.sleep(2)
        closed_loop_pick(destination)



    return "I should have done something intelligent"


def closed_loop_pick(destination='othello_piece',):
    #assume that i'm already close.
    global tf_listener
    global last_seen

    while True:

        execute = True
        rotate = False
        exit = raw_input("E to quit.  D for dist to target. B to break, R to rotate arm to best position, Otherwise, i'll move to 0 \n\r")

        if (exit == 'e') or (exit == 'E'):
            rospy.signal_shutdown("Shutdown signal recieved")
            return
        elif(exit == 'd') or (exit == 'D'):
            execute = False
        elif(exit == 'B') or (exit == 'b'):
            break 
        elif(exit == 'R') or (exit == 'r'):
            rotate = True

        try:
            #now = rospy.Time.now()
            #tf_listener.waitForTransform('suction_cup','othello_piece',now,rospy.Duration(2.5))
            #now = rospy.Time.now()
            last_time = tf_listener.getLatestCommonTime('suction_cup','othello_piece')
            (trans,rot) = tf_listener.lookupTransform('suction_cup','othello_piece',last_time)

            print("Saw the othello piece " + str( -last_time.to_sec() + rospy.Time.now().to_sec()) + " secs ago \n\r")

            #get the RBT from the suction_cup to the base so incrimental movement can deal with it.
            last_time = tf_listener.getLatestCommonTime('suction_cup','base')
            (trans_base,rot_base) = tf_listener.lookupTransform('suction_cup','base',last_time)
            
            zoffset = 0.01

        except KeyboardInterrupt:
            sys.exit()

        except:
            print("Could not find trans / rot \n\r")
            traceback.print_exc()
        else:
            print("Error between suction cup and the othello piece is \n\r")
            print(trans)
            print("")


            err_dist = np.linalg.norm(trans[:2]) #error in meters from 

            print("XY ERROR IS " +str(err_dist))

            if execute:

                retract = False

                if rotate:
                    #rotate arm to best orientation for view
                    trans_goal = None

                    last_time = tf_listener.getLatestCommonTime('suction_cup','left_hand_camera')
                    (trans_suc,rot_suc) = tf_listener.lookupTransform('suction_cup','left_hand_camera',last_time)

                    last_time = tf_listener.getLatestCommonTime(homing_marker,destination)
                    (trans_piece,rot_piece) = tf_listener.lookupTransform(homing_marker,destination,last_time)

                    print("Suck")
                    print trans_suc
                    print('piece')
                    print trans_piece

                    #print rot_piece
                    #omega_piece,theta_piece = eqf.quaternion_to_exp(rot_piece)
                    euler_piece = tf.transformations.euler_from_quaternion(rot_piece)  #tf has euler -> quaternion conversions!

                    print("Euler piece is")
                    print(euler_piece)


                    euler_suc = tf.transformations.euler_from_quaternion(rot_suc)  #tf has euler -> quaternion conversions!

                    print("Euler piece is")
                    print(euler_piece)

                    #hypothesis
                    #omega = 0,0,-1
                    #theta = theta_piece - some theta for me.

                    roll = np.pi
                    pitch = 0
                    yaw = 0

                    '''
                    #http://onlineconversion.vbulletin.net/forum/main-forums/convert-and-calculate/3249-euler-angle-quaternion
                    qx = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
                    qy = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)-np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
                    qz = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
                    qw = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)-np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)

                    q = [qx,qy,qz,qw]

                    print("I decided to go to " + str(eqf.quaternion_to_exp(q)))

                    rospy.wait_for_service('low_level_arm')
                    movement_server = rospy.ServiceProxy('low_level_arm', kinematics_request)

                    response = movement_server(trans=None,rot=q)
                    '''
                    continue


                elif trans[2] > 0.03:
                    #i'm very high away, just get closer...
                    trans_goal = (.85*trans[0],.85*trans[1],trans[2]/2 + zoffset)
                    change_height = ''
                    keep_oreint = 'True'
                    grip = ''
                    print("Class 1 move")
                elif err_dist > 0.01: #1cm, 1 inch.
                    #final lateral alignment
                    trans_goal = (trans[0],trans[1],-0.005) #i want to move the suction cupt to 0,0,whatever height i'm at right now of hte othello piece.
                    #the -0.005 b/c i always seem to move up when I don't want to.
                    change_height = 'False'
                    keep_oreint = 'True'
                    grip = ''
                    print("Final Centering move")
                else:
                    #just go down % hit it
                    trans_goal = (0*trans[0],0*trans[1],trans[2] + zoffset )
                    change_height = ''
                    keep_oreint = 'True'
                    grip = 'True'
                    print("Go for it!")

                    retract = True
                    #retract to see if i picked up the thing



                print("I want to move by, in the suction cup frame, \n\r")
                print(str(trans_goal) + "\n\r")

                #setup request.  Now, we just need to figure out what to put into it.
                rospy.wait_for_service('low_level_arm')
                movement_server = rospy.ServiceProxy('low_level_arm', kinematics_request)

                response = movement_server(trans=trans_goal,incrimental='True',grip = grip,keep_orient = keep_oreint,target_trans = trans_base,change_height = change_height, target_rot = rot_base)
                print(response)

                if retract:
                    time.sleep(1)
                    rospy.wait_for_service('low_level_arm')
                    movement_server = rospy.ServiceProxy('low_level_arm', kinematics_request)
                    response = movement_server(trans=[0,0,0.1],incrimental='True',grip = grip)
                    print(response)
                    break

        
# Initializes listener for the TF and then closed_loop_pick
def init():
    listener('/tf')
    service_setup()

    rospy.spin()
    #closed_loop_pick()


if __name__ == '__main__':
    if len(sys.argv) == 1:
        init()
    elif sys.argv[1] == '-h':
        print(usage_str)
    elif sys.argv[1] == '--help':
        print(usage_str)
    else:
        print("invalid arguments")
        print(usage_str)