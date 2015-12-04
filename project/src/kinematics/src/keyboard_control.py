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
movement_server = None

#def move_to_coord(trans, rot, arm, which_arm='left', keep_oreint=False,base="base"):
#def incrimental_movement(dx,dy,dz,arm,which_arm,rbt=None,last_pos = None,changeHeight=True,keep_oreint=False):



# This command dictates which key decides which Baxter movement.
def keyboard_ctrl():

    rospy.wait_for_service('low_level_arm')
    

    movement_server = rospy.ServiceProxy('low_level_arm', kinematics_request)


    response = movement_server(trans=[.03,.3,-.03],incrimental='True',grip = 'True')

    print(response)

    '''
    #HACK HACK HACk
    global rbt

    screen = curses.initscr()

    if new_rot == None:
        new_rot = np.eye(3)
    # Here we may need to use inverse instead of the array we have. 
    # I am not sure if we will be receiving the rbt from the tag to the camera or vice versa.
    # Right now this assumes the rbt is from the tag to the torso. 
    new_rot = np.array([[new_rot[0][0], new_rot[0][1], new_rot[0][2]], 
        [new_rot[1][0], new_rot[1][1], new_rot[1][2]], [new_rot[2][0], new_rot[2][1], new_rot[2][2]]])

    inc_var = 0.050


    try:

        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        screen.keypad(1)
        screen.nodelay(1)

        while True:

            #moved inside while so dynamic updates to inc_var can propigate

            if rbt == None:
                new_rot = np.eye(3)
            else:
                #print("NEW RBT\n\r")
                new_rot = rbt
                #print(new_rot)
                #print("\n\r")

            new_rot = np.array([[new_rot[0][0], new_rot[0][1], new_rot[0][2]], 
                [new_rot[1][0], new_rot[1][1], new_rot[1][2]], [new_rot[2][0], new_rot[2][1], new_rot[2][2]]])

            new_rot = np.linalg.inv(new_rot)
            #need to invert hte rotation matrix.  While I have the RBT from the tag to the base, I need the rotation matrix that will rotate 
            #cardinal directions the other way 'round'


            event = screen.getch()

            
            if event == curses.KEY_RIGHT:
                trans_mat  = incrimental_movement(inc_var,0,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_LEFT:
                trans_mat  = incrimental_movement(-inc_var,0,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_UP:
                trans_mat  = incrimental_movement(0,inc_var,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_DOWN:
                trans_mat  = incrimental_movement(0,-inc_var,0,arm,which_arm,rbt,trans_mat)
            elif event == 122:  # Z
                trans_mat  = incrimental_movement(0,0,inc_var,arm,which_arm,rbt,trans_mat)
            elif event == 120:  # X
                trans_mat  = incrimental_movement(0,0,-inc_var,arm,which_arm,rbt,trans_mat)
            elif event == 103:  # G
                if gripper_closed:
                    gripper.open(block=False)
                    gripper_closed = False
                else:
                    gripper.close(block=False)
                    gripper_closed = True
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
            elif event == 101: #E
            

                #resets position and orientation values. 
                pose = limb.endpoint_pose()
                pose = {'rot': list(pose['orientation']), 
                        'trans' : list(pose['position'])}
                trans = pose['trans']
                trans_mat = np.array([[trans[0]], [trans[1]], [trans[2]]])
                print("Orientation and position reset \n\r")
            elif event == 102: #F
                pose = limb.endpoint_pose()
                pose = {'rot': list(pose['orientation']), 
                        'trans' : list(pose['position'])}
                trans = pose['trans']
                print( str(trans) + "\n\r")
            elif event == 113: #q:
                exit_soft = True
                break


            

            elif event == 27:   # Esc
                exit_soft = False
                break
            elif event == -1:  #no press on getch, it'll just keep running
                pass
            else:
                print("invalid key: %r \n\r" % event)
            #so we can only spam keyboard commands so fast.  But in general, don't hold down the key.
            time.sleep(0.2)
            
    finally:
        screen.keypad(0)
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        if not exit_soft:
            rospy.signal_shutdown("Shutdown signal recieved")
    '''

'''
def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback
    rospy.Subscriber(tf_topic,TFMessage,callback)

    global tf_listener

    tf_listener = tf.TransformListener()

def callback(data):
    #run every time i see a TFMessage on tf_topic.
    global tf_listener
    global last_seen
    global rbt

    #header = data.header
    #print(data.transforms[0])
    #print(dir(data.transforms[0]))

    tim = data.transforms[0]
    tim = tim.header.stamp
    #print(tim)
    #print('fo')
    if tf_listener.frameExists('ar_marker_63'):

        now = rospy.Time.now()
        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            (trans,rot) = tf_listener.lookupTransform('ar_marker_63','base',tim)
            
            (omega,theta) = eqf.quaternion_to_exp(rot)
            rbt = eqf.return_rbt(trans,rot)

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
            pass
'''

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
    keyboard_ctrl()