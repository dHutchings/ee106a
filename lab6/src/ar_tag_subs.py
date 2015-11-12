#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf
import os

print(os.path.basename(root))
from srv.NuSrv.srv import *

listener = None

def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """


    #YOUR CODE HERE
    
    (omega, theta) = eqf.quaternion_to_exp(rot)

    rbt = eqf.create_rbt(omega,theta,trans)


    return rbt

def compute_twist(rbt):
    """
    Computes the corresponding twist for the rigid body transform

    Input:
        rbt - (4,4) ndarray 

    Output:
        v - (3,) array
        w - (3,) array
    """
    #YOUR CODE HERE
    
    rot_matrix = rbt[0:3,0:3]
    trans = rbt[0:3,3]
    
    (omega,theta) = eqf.find_omega_theta(rot_matrix)
    v = eqf.find_v(omega,theta,trans)

    v = v.flatten()

    #v = np.array(3,1)
    #w = np.array(3,1)
    #print(v)
    #print(omega)
    return (v,omega)

if __name__=='__main__':
    rospy.init_node('ar_tags_subs')
    if len(sys.argv) < 2:
        print('Use: ar_tag_subs.py [Zumys AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[1]


    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

            
        try:
            #modified so it looks up to origin. (USB_CAM??)

            #camera must be on via run_all already.
            (trans, rot) = listener.lookupTransform('usb_cam', ar_tags['arZ'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between camera and ' + ar_tags['arZ'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between camera and ' + ar_tags['arZ'])
            print twist

            #as required by lab 8 --> publish this!
            #ok.... wait, how?

            print('wait_for_service')
            rospy.wait_for_service('innovation')
            print('done')
            try:
                add_two_ints = rospy.ServiceProxy('foo', NuSrv)
                print('add two ints')
                resp1 = add_two_ints(rbt)
                print resp1
            except rospy.ServiceException, e:
                print sys.exc_info()[0]
                #print "Service call failed: %s"%e

            #nuSrv = 'usb_cam' #origin is usb_cam, right..?
            #print(nuSrv)
            #rospy.Service('innovation',nuSrv,self.triggerUpdate)
            print('sent update!')
        except:
            print sys.exc_info()[0]
            #print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            #print(listener.frameExists(ar_tags['ar0']))
            #print(listener.frameExists(ar_tags['ar1']))
            #print(listener.frameExists(ar_tags['arZ']))
            print ''



        #print(listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'],rospy.Time(0)))

        rate.sleep()
