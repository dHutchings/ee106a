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

from kinematics.srv import *

usage_str = \
"""
Inverse kinematics script

Requires baxter.sh and abxter_moveit_noexec.launch to be run beforehand.
Interactive mode allows control of the end-effector position via 
text input or arrow keys.  Can be done in relation to a set TF frame, or baxter's base.

Keyboard-interactive controls:
[UP, DOWN]      move along y-axis   Up is y+, Down is y-
[LEFT, RIGHT]   move along x-axis.  Right is x+, Left is x-
[Z, X]          move along z-axis
[G]             toggle gripper

Flags:
--help, -h      print usage instructions

no options: normal keyboard control
one option: the option is the tf topic.  Normally, this should be "/tf"
"""

trans = None
rot = None
movement_server = None
tf_listener = None

#def move_to_coord(trans, rot, arm, which_arm='left', keep_oreint=False,base="base"):
#def incrimental_movement(dx,dy,dz,arm,which_arm,rbt=None,last_pos = None,changeHeight=True,keep_oreint=False):



# This command dictates which key decides which Baxter movement.
def keyboard_ctrl():


    
    #HACK HACK HACk.  If there's a tf_listener, there's gonna be a non-none trans/rot
    global trans
    global rot

    screen = curses.initscr()


    inc_var = 0.050


    try:

        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        screen.keypad(1)
        screen.nodelay(1)

        while True:

            #moved inside while so dynamic updates to inc_var can propigate

            event = screen.getch()
            curses.flushinp() 
            #do this so only one keyboard command gets processed at a time!

            grip = ''
            trans_mat = []

            
            if event == curses.KEY_RIGHT:
                trans_mat  = [inc_var,0,0]
            elif event == curses.KEY_LEFT:
                trans_mat  = [-inc_var,0,0]
            elif event == curses.KEY_UP:
                trans_mat = [0,inc_var,0]
            elif event == curses.KEY_DOWN:
                trans_mat = [0,-inc_var,0]
            elif event == 122:  # Z
                trans_mat = [0,0,inc_var]
            elif event == 120:  # X
                trans_mat = [0,0,-inc_var]
            elif event == 103:  # G
                grip = 'True'
            elif event == 104:
                grip = 'False'
            elif event == 100: #D
                screen.nodelay(0) #temporarily turn off non-blocking getch
                #set inc_var to a new value
                print("Enter new value for inc-var, in the form\n\r")
                print("XX.XXd.  You must terminate with a d since we are bad at using curses\n\r")
                event2 = screen.getch()
                string = chr(event2)

                while event2 is not 'd':
                    event2 = chr(screen.getch())
                    string = string + event2
                string = string[:-1]
                inc_var = float(string)

                print("New Inc_var is " + str(inc_var) + '\n\r')
                screen.nodelay(1) #and then turn back on non-blocking getch
            elif event == -1:  #no press on getch, it'll just keep running
                pass
            else:
                print("invalid key: %r \n\r" % event)
            #so we can only spam keyboard commands so fast.  But in general, don't hold down the key.


            if event is not -1: #do this b/c -1 means i've not put in a keyboard command
                #setup request.  Now, we just need to figure out what to put into it.
                rospy.wait_for_service('low_level_arm')
                movement_server = rospy.ServiceProxy('low_level_arm', kinematics_request)

                if rot is None:
                    #if rot is none, there's no TF listener looking at AR tag transforms.  Therefore, i'm moving in baxter base coordinate systems
                    response = movement_server(trans=trans_mat,incrimental='True',grip = grip)
                else:
                    print("Rot is not none \n\r")
                    response = movement_server(trans=trans_mat,incrimental='True',grip = grip,target_trans = trans, target_rot = rot)

                print(str(response) + "\n\r")

            
    finally:
        screen.keypad(0)
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        rospy.signal_shutdown("Shutdown signal recieved")



def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("keyboard_control_rel_to_tf")
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
    if tf_listener.frameExists('ar_marker_63'):

        now = rospy.Time.now()
        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            (trans,rot) = tf_listener.lookupTransform('ar_marker_63','base',tim)
            
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
            time.sleep(0.2)
        #time0 asks for the most recent one
        except:
            #print("error with the update \n\r")
            pass


def get_pose(limb):
    pose = {}

    while len(pose) is 0:
        #sometimes, i'm getting empty poses.  don't know why, but this should take care of it.
        pose = limb.endpoint_pose()
        #print(pose)
        time.sleep(.01)

    pose = {'rot': list(pose['orientation']), 
            'trans' : list(pose['position'])}

    return pose


if __name__ == '__main__':
    print(len(sys.argv))

    if len(sys.argv) > 1:
        if sys.argv[1] == '-h':
            print(usage_str)
        else:
            target_tf_Frame = sys.argv[1]
            listener(target_tf_Frame)
            time.sleep(5)
            keyboard_ctrl()
    else:
        keyboard_ctrl()