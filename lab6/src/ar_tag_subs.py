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
    if len(sys.argv) < 4:
        print('Use: ar_tag_subs.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    print(ar_tags)

    print(ar_tags['ar0'])

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #print(dir(listener))
        #print(listener.allFramesAsDot())

        #print(listener.frameExists(ar_tags['arZ']))        

        #print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
        #print(listener.frameExists(ar_tags['ar0']))
        #print(listener.frameExists(ar_tags['ar1']))
        #print(listener.frameExists(ar_tags['arZ']))

        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar1'], ar_tags['arZ'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            #rbt = return_rbt(np.array(trans),np.array(rot))
            print('gab between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            print twist
        except:
            #print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            #print(listener.frameExists(ar_tags['ar0']))
            #print(listener.frameExists(ar_tags['ar1']))
            #print(listener.frameExists(ar_tags['arZ']))
            print ''

        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['ar1'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print twist
        except:
            #print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            #print(listener.frameExists(ar_tags['ar0']))
            #print(listener.frameExists(ar_tags['ar1']))
            #print(listener.frameExists(ar_tags['arZ']))
            print ''
            
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['arZ'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print twist
        except:
            #print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            #print(listener.frameExists(ar_tags['ar0']))
            #print(listener.frameExists(ar_tags['ar1']))
            #print(listener.frameExists(ar_tags['arZ']))
            print ''



        #print(listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'],rospy.Time(0)))

        rate.sleep()
