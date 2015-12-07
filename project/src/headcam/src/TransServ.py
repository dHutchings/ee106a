#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')

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
A server for TF Frames.

It is the server for a service 'new_tf_frames', that publishes broadcasts certian TF transforms, untill it is told not to.

a tag_persistance.srv message:

Child: Child (new) TF frame's name
Parent: the parent of the child.  If this is None, child is remove_index
Tranfrom:  The transform from the parent to the child

"""


import tf

names = []
parents = []
transforms = []

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
    if(len(sys.argv) > 1):
        print usage_str
    else:
        init()
