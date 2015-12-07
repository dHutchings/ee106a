#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')

from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
import numpy as np

import roslib
import rospy

from tf2_msgs.msg import TFMessage
import tf
import time

import traceback
rbt = None

from headcam.srv import *

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

last_seen = time.time()

import tf
global stop
global dlist

names = []
parents = []
transforms = []
stop = False

def movement_server():
    print("Initializing node... ")
    rospy.init_node('ptrans')
    s = rospy.Service('new_data', tag_persistance, callback2)
    print "Ready to move arm"

def callback2(data):
    child = data.child
    parent = data.parent
    transform = data.transform

    if child not in names:
        names.append(child)
        parents.append(parent)
        transforms.append(transform)
    else:
        index = names.index(child)
        #print(index)
        parents[index] = parent
        transforms[index] = transform

def remove_index(data):
    if data.child in names:
        index = names.index(child)
        names.remove(child)
        parents.pop(index)
        transforms.pop(index)

def init():
    movement_server()
    #then, down here, do a while_true
    r = rospy.Rate(10) # 10hz
    while not stop:
        now = rospy.Time.now()

        for child in names:
            index = names.index(child)
            br = tf.TransformBroadcaster()
            rotation = transforms[index].rotation
            translation = transforms[index].translation
            br.sendTransform((translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            now,
            child, parents[index])

        r.sleep()

if __name__ == '__main__':
  init()
  # global ar_marker
  # if (len(sys.argv) is not 2) and (len(sys.argv) is not 4):
  #   print len(sys.argv)
  #   print "error"
  #   print sys.argv
  # else:
  #   ar_marker = sys.argv[1]
  #   init()