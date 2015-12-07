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
    rospy.init_node('new_tf_frames')
    s = rospy.Service('new_tf_frames', tag_persistance, callback2)
    print "Ready to Adjust Frames"

def callback2(data):
    child = data.child
    parent = data.parent

    
    if parent is '':
        remove_index(data)
    elif child not in names:  #make the order right.  Make sure to remove parents first.
        transform = data.transform
        names.append(child)
        parents.append(parent)
        transforms.append(transform)
    else:
        pass
        transform = data.transform
        #update.
        index = names.index(child)
        #print(index)
        parents[index] = parent
        transforms[index] = transform

    return("success")

def remove_index(data):
    child = data.child
    #if you don't do this, it's gonna go and remove random AR tags...!
    if data.child in names:
        index = names.index(child)
        names.remove(child)
        parents.pop(index)
        transforms.pop(index)

def init():
    movement_server()
    #then, down here, do a while_true
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        #print(transforms)

        for child in names:
            index = names.index(child)
            br = tf.TransformBroadcaster()
            rotation = transforms[index].rotation
            translation = transforms[index].translation
            #print transforms[index]
            #print child
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
